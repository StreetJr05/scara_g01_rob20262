"""Accuracy / precision test runner (ISO 9283 style pose cycling).

    ros2 run scara_control characterize --config characterization.yaml --out run1.csv

For every cycle c = 1..N the robot visits P1 -> P2 -> ... -> Pn (optionally
passing through an approach point before each one, so every arrival comes
from the same direction). At each arrival it waits `dwell_s`, records the
encoder-based pose (internal measurement) and then the GROUND TRUTH:

  ground_truth: manual   the terminal asks for the measured x y [mm] (from
                         the grid sheet / caliper). Empty line = fill later
                         in the CSV.
  ground_truth: none     only encoder data is logged; fill meas_x_mm /
                         meas_y_mm afterwards (e.g. from the marks the tool
                         left on the sheet or from photos).

Then:  ros2 run scara_control analyze run1.csv
"""
import argparse
import csv
import os
import sys
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory

from scara_control import protocol as P
from scara_control.client import ScaraClient

FIELDS = ['cycle', 'point', 'cmd_x_mm', 'cmd_y_mm', 'cmd_z_mm', 'status',
          'q1_goal_deg', 'q2_goal_deg', 'q1_deg', 'q2_deg', 'enc_x_mm', 'enc_y_mm',
          'meas_x_mm', 'meas_y_mm', 'time']


def default_config():
    return os.path.join(get_package_share_directory('scara_control'), 'config', 'characterization.yaml')


def ask_measurement(label):
    while True:
        txt = input(f'   ground truth {label}  "x y" mm (Enter = later): ').strip()
        if not txt:
            return '', ''
        try:
            x, y = map(float, txt.replace(',', ' ').split()[:2])
            return f'{x:.3f}', f'{y:.3f}'
        except ValueError:
            print('   expected two numbers, e.g. 201.5 99.8')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--config', default=None)
    ap.add_argument('--out', default=time.strftime('characterization_%Y%m%d_%H%M%S.csv'))
    args = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    with open(args.config or default_config()) as f:
        cfg = yaml.safe_load(f)
    points = cfg['points']
    cycles = int(cfg.get('cycles', 10))
    approach = cfg.get('approach')
    dwell = float(cfg.get('dwell_s', 1.0))
    gt = cfg.get('ground_truth', 'manual')
    if len(points) < 5 or cycles < 10:
        print(f'WARNING: the guide asks for >= 5 points and >= 10 repetitions (have {len(points)} x {cycles})')

    rclpy.init()
    c = ScaraClient('scara_characterize', print_events=False)
    try:
        if not c.wait_for_robot(10.0):
            print('No /scara/state. Is the agent running?')
            return 1
        if not c.homed() or cfg.get('home_first', True):
            print('HOME ...', flush=True)
            if not c.home():
                print('homing failed')
                return 1

        with open(args.out, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=FIELDS)
            w.writeheader()
            for cyc in range(1, cycles + 1):
                for i, (x, y, z) in enumerate(points, start=1):
                    if approach:
                        c.goto(*(v / 1e3 for v in approach))
                    ok, info = c.goto(x / 1e3, y / 1e3, z / 1e3)
                    c.spin_for(dwell)
                    s = c.state
                    row = {
                        'cycle': cyc, 'point': i, 'cmd_x_mm': x, 'cmd_y_mm': y, 'cmd_z_mm': z,
                        'status': info if ok else f'FAIL {info}',
                        'q1_goal_deg': f'{s[P.ST_Q1_GOAL] * 57.29578:.3f}',
                        'q2_goal_deg': f'{s[P.ST_Q2_GOAL] * 57.29578:.3f}',
                        'q1_deg': f'{s[P.ST_Q1] * 57.29578:.3f}', 'q2_deg': f'{s[P.ST_Q2] * 57.29578:.3f}',
                        'enc_x_mm': f'{s[P.ST_X] * 1e3:.3f}', 'enc_y_mm': f'{s[P.ST_Y] * 1e3:.3f}',
                        'meas_x_mm': '', 'meas_y_mm': '', 'time': f'{time.time():.3f}',
                    }
                    print(f'cycle {cyc:2d}/{cycles} P{i} cmd=({x:.1f}, {y:.1f}) '
                          f'enc=({row["enc_x_mm"]}, {row["enc_y_mm"]}) {row["status"]}', flush=True)
                    if ok and gt == 'manual':
                        row['meas_x_mm'], row['meas_y_mm'] = ask_measurement(f'P{i} c{cyc}')
                    w.writerow(row)
                    f.flush()
                    if not ok and int(s[P.ST_MODE]) == P.MODE_FAULT:
                        print('robot in FAULT, aborting')
                        return 1
        print(f'saved {args.out}\nnext: ros2 run scara_control analyze {args.out}')
    except KeyboardInterrupt:
        c.send_cmd(P.CMD_STOP)
        c.spin_for(0.2)
        print(f'\ninterrupted, partial data in {args.out}')
    finally:
        c.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
