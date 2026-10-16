"""Point-to-point commands in task space (x, y, z) from Python / ROS 2.

Examples (millimeters, base frame):
    ros2 run scara_control goto 200 100 65            # one point
    ros2 run scara_control goto 200 100 25 --home     # home first, then go (z=25 -> P down)
    ros2 run scara_control goto --file points.yaml    # sequence from YAML (units: mm)
    ros2 run scara_control goto -i                    # interactive: type "x y z" lines
    ros2 run scara_control goto --cmd valve_off       # IO / misc commands

The printed lines show the whole flow:
    target -> (firmware) IK -> target angles -> movement -> reached + error
"""
import argparse
import sys

import rclpy
import yaml

from scara_control import protocol as P
from scara_control.client import ScaraClient

CMDS = {'home': P.CMD_HOME, 'stop': P.CMD_STOP, 'p_up': P.CMD_P_UP, 'p_down': P.CMD_P_DOWN,
        'valve_on': P.CMD_VALVE_ON, 'valve_off': P.CMD_VALVE_OFF, 'reset': P.CMD_RESET_FAULT,
        'hold': P.CMD_HOLD, 'force_homed': P.CMD_FORCE_HOMED}


def go(client, x, y, z, scale):
    print(f'-> target ({x}, {y}, {z}) {"mm" if scale == 1e-3 else "m"}', flush=True)
    ok, info = client.goto(x * scale, y * scale, z * scale)
    s = client.state
    print(f'   {"OK " if ok else "ERR"} {info}; encoders: '
          f'x={s[P.ST_X] * 1e3:.2f} y={s[P.ST_Y] * 1e3:.2f} z={s[P.ST_Z] * 1e3:.1f} mm', flush=True)
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('xyz', nargs='*', type=float, help='x y z')
    ap.add_argument('--m', action='store_true', help='coordinates in meters (default mm)')
    ap.add_argument('--home', action='store_true', help='run HOME before moving')
    ap.add_argument('--file', help='YAML with "points: [[x, y, z], ...]" (units: mm unless "units: m")')
    ap.add_argument('--repeat', type=int, default=1, help='repeat the file sequence N times')
    ap.add_argument('-i', '--interactive', action='store_true')
    ap.add_argument('--cmd', choices=sorted(CMDS), help='send a single command and exit')
    args = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    client = ScaraClient('scara_goto')
    scale = 1.0 if args.m else 1e-3
    rc = 0
    try:
        if not client.wait_for_robot(10.0):
            print('No /scara/state. Is the micro-ROS agent running and the ESP32 connected?')
            return 1

        if args.cmd:
            client.send_cmd(CMDS[args.cmd])
            client.spin_for(0.5)
            return 0

        if args.home:
            print('HOME ...', flush=True)
            if not client.home():
                print('homing failed')
                return 1

        if args.file:
            with open(args.file) as f:
                data = yaml.safe_load(f)
            fscale = 1.0 if data.get('units', 'mm') == 'm' else 1e-3
            for _ in range(args.repeat):
                for pt in data['points']:
                    if not go(client, *pt, fscale):
                        rc = 1
        elif args.interactive:
            print('type "x y z" (mm) or a command (home, stop, p_up, p_down, valve_on, valve_off, reset); '
                  'Ctrl+D to quit')
            for line in sys.stdin:
                parts = line.split()
                if not parts:
                    continue
                if parts[0] in CMDS:
                    client.send_cmd(CMDS[parts[0]])
                    client.spin_for(0.3)
                    continue
                try:
                    go(client, *map(float, parts[:3]), scale)
                except (TypeError, ValueError):
                    print('expected: x y z')
        elif len(args.xyz) == 3:
            rc = 0 if go(client, *args.xyz, scale) else 1
        else:
            ap.print_help()
            rc = 2
        client.spin_for(0.3)   # flush remaining events
    except KeyboardInterrupt:
        client.send_cmd(P.CMD_STOP)
        client.spin_for(0.2)
    finally:
        client.destroy_node()
        rclpy.try_shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
