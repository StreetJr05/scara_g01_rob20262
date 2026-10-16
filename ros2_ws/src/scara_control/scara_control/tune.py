"""PID tuning helper: repeated point-to-point moves + step-response metrics.

    ros2 run scara_control tune                 # J1 test: 40 deg shoulder swing, elbow fixed
    ros2 run scara_control tune --pair j2       # J2 test: ~37 deg elbow move
    ros2 run scara_control tune --pair j1 --cycles 3 --out tune_j1_kp3

The robot must be homed. Change gains live between runs, e.g.
    ros2 param set /scara_bridge j1_kp 3.0

For every move and joint it prints:
  move     commanded joint motion [deg]
  lag      max |reference - position| while the trajectory runs [deg]
  over     overshoot past the goal [deg]  (on the non-moving joint: max deviation)
  settle   time from trajectory end until |error| < tol for good [s]
  final    goal - position at the end of the dwell [deg]
  sat      % of the move with the PWM at its limit
  osc      sign changes of the error after the trajectory ends (ringing)
and an estimate of the duty needed per rad/s (a starting value for jX_kff).
A CSV with the raw state and a PNG with the responses go to --out, by default
<project>/results/tuning/<date>_<pair> (a relative --out name also goes there).
"""
import argparse
import csv
import math
import os
import sys
import time

import rclpy

from scara_control import protocol as P
from scara_control.client import ScaraClient

# Point pairs [mm] inside the workspace (170 < r < 202 mm, see characterization.yaml)
PAIRS = {
    # same radius (r = 189 mm) -> same q2: only the shoulder moves (~40 deg)
    'j1': ((185.0, 40.0), (116.0, 149.6)),
    # same direction, r = 198 -> 175 mm: the elbow moves ~37 deg (shoulder a little)
    'j2': ((193.5, 41.8), (171.1, 37.0)),
    # both joints move
    'both': ((180.0, -60.0), (120.0, 145.0)),
}
DEG = 180.0 / math.pi


def results_dir():
    """<project>/results/tuning: the first parent of the current directory that
    contains both firmware/ and ros2_ws/ (falls back to the current directory)."""
    d = os.path.abspath(os.getcwd())
    while True:
        if os.path.isdir(os.path.join(d, 'firmware')) and os.path.isdir(os.path.join(d, 'ros2_ws')):
            return os.path.join(d, 'results', 'tuning')
        parent = os.path.dirname(d)
        if parent == d:
            return os.path.join(os.getcwd(), 'results', 'tuning')
        d = parent


class Recorder(ScaraClient):
    def __init__(self):
        super().__init__('scara_tune', print_events=False)
        self.log = []

    def _on_state(self, msg):
        super()._on_state(msg)
        self.log.append((time.monotonic(), list(msg.data)))


def segment_metrics(rows, j, tol_deg, max_duty):
    """rows: [(t, state)] of one move. Returns a dict of metrics for joint j."""
    iq, iref, igoal, iu, idq = (P.ST_Q1 + j, P.ST_Q1_REF + j, P.ST_Q1_GOAL + j, P.ST_U1 + j, P.ST_DQ1 + j)
    t0 = rows[0][0]
    t = [r[0] - t0 for r in rows]
    q = [r[1][iq] * DEG for r in rows]
    ref = [r[1][iref] * DEG for r in rows]
    u = [r[1][iu] for r in rows]
    goal = rows[-1][1][igoal] * DEG
    start = q[0]
    move = goal - start

    # trajectory end: first sample where the reference has arrived at the goal
    k_end = next((k for k, r in enumerate(ref) if abs(r - goal) < 1e-3), len(ref) - 1)
    lag = max((abs(r - y) for r, y in zip(ref[:k_end + 1], q[:k_end + 1])), default=0.0)
    if abs(move) > 1.0:
        sgn = 1.0 if move > 0 else -1.0
        over = max(0.0, max(sgn * (y - goal) for y in q))
    else:
        over = max(abs(y - goal) for y in q)   # joint that should stay still
    # settle: last time the error was outside the band, after the trajectory end
    settle = 0.0
    for k in range(len(q) - 1, k_end - 1, -1):
        if abs(goal - q[k]) > tol_deg:
            settle = t[min(k + 1, len(t) - 1)] - t[k_end]
            break
    errs = [goal - y for y in q[k_end:]]
    osc = sum(1 for a, b in zip(errs, errs[1:]) if a * b < 0 and max(abs(a), abs(b)) > 0.1)
    sat = 100.0 * sum(1 for x in u if abs(x) >= max_duty - 0.01) / max(len(u), 1)
    return dict(move=move, lag=lag, over=over, settle=settle, final=goal - q[-1], sat=sat, osc=osc)


def duty_per_speed(rows, j, max_duty):
    """Least-squares slope of |u| - min_duty-ish offset vs |dq| while cruising."""
    pts = [(abs(s[P.ST_DQ1 + j]), abs(s[P.ST_U1 + j])) for _, s in rows
           if abs(s[P.ST_DQ1 + j]) > 0.3 and abs(s[P.ST_U1 + j]) < max_duty - 0.02
           and s[P.ST_U1 + j] * s[P.ST_DQ1 + j] > 0]
    if len(pts) < 8:
        return None
    n = len(pts)
    mx = sum(p[0] for p in pts) / n
    my = sum(p[1] for p in pts) / n
    sxx = sum((p[0] - mx) ** 2 for p in pts)
    if sxx < 1e-6:
        return None
    slope = sum((p[0] - mx) * (p[1] - my) for p in pts) / sxx
    return slope, my - slope * mx


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--pair', choices=sorted(PAIRS), default='j1')
    ap.add_argument('--cycles', type=int, default=2, help='A->B->A round trips')
    ap.add_argument('--dwell', type=float, default=1.5, help='[s] recorded after each arrival')
    ap.add_argument('--z', type=float, default=65.0, help='[mm] keep > 45 so the P stays up')
    ap.add_argument('--tol', type=float, default=0.5, help='[deg] settle band')
    ap.add_argument('--max-duty', type=float, default=0.8, help='jX_max_duty, for the saturation %%')
    ap.add_argument('--out', default=None,
                    help='folder name or path (default: <date>_<pair> under results/tuning)')
    args = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    name = args.out or time.strftime(f'%Y%m%d_%H%M%S_{args.pair}')
    args.out = name if os.path.isabs(name) else os.path.join(results_dir(), name)

    rclpy.init()
    rec = Recorder()
    try:
        if not rec.wait_for_robot():
            print('no /scara/state: is the launch running and the ESP32 online?')
            return 1
        if not rec.homed():
            print('robot not homed: press H in the pendant first')
            return 1

        a, b = PAIRS[args.pair]
        targets = [a] + [b, a] * args.cycles
        moves = []      # (label, rows)
        print(f'pair {args.pair}: {a} <-> {b} mm, {args.cycles} round trips')
        for i, (x, y) in enumerate(targets):
            rec.log.clear()
            ok, info = rec.goto(x * 1e-3, y * 1e-3, args.z * 1e-3)
            rec.spin_for(args.dwell)
            if not ok:
                print(f'  move to ({x}, {y}) failed: {info}')
                return 1
            if i > 0:   # the first move only brings the arm to A
                moves.append((f'{i}: -> ({x:.0f}, {y:.0f})', list(rec.log)))
            print(f'  {i}: ({x:.0f}, {y:.0f}) {info}')
    finally:
        rec.destroy_node()
        rclpy.shutdown()

    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, 'state.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['move', 't'] + ['mode', 'flags', 'q1', 'q2', 'q1_ref', 'q2_ref', 'q1_goal', 'q2_goal',
                                    'x', 'y', 'z', 'u1', 'u2', 'dq1', 'dq2', 'error', 'uptime',
                                    'move_id', 'elbow'])
        for m, (_, rows) in enumerate(moves):
            for t, s in rows:
                w.writerow([m, f'{t - rows[0][0]:.3f}'] + [f'{v:.5f}' for v in s])

    print(f'\n{"move":<22}{"joint":>6}{"move":>8}{"lag":>7}{"over":>7}{"settle":>8}{"final":>8}{"sat%":>6}{"osc":>5}')
    agg = {0: [], 1: []}
    for label, rows in moves:
        for j in (0, 1):
            m = segment_metrics(rows, j, args.tol, args.max_duty)
            agg[j].append(m)
            print(f'{label:<22}{"J" + str(j + 1):>6}{m["move"]:8.1f}{m["lag"]:7.2f}{m["over"]:7.2f}'
                  f'{m["settle"]:8.2f}{m["final"]:8.2f}{m["sat"]:6.0f}{m["osc"]:5d}')
            label = ''
    print('\nworst case per joint (deg, s):')
    for j in (0, 1):
        ms = agg[j]
        print(f'  J{j + 1}: lag {max(m["lag"] for m in ms):.2f}  over {max(m["over"] for m in ms):.2f}  '
              f'settle {max(m["settle"] for m in ms):.2f}  |final| {max(abs(m["final"]) for m in ms):.2f}  '
              f'sat {max(m["sat"] for m in ms):.0f}%  osc {max(m["osc"] for m in ms)}')
        fit = duty_per_speed([r for _, rows in moves for r in rows], j, args.max_duty)
        if fit:
            print(f'       duty needed ~ {fit[1]:.2f} + {fit[0]:.3f} * |speed rad/s|  '
                  f'-> try j{j + 1}_kff ~ {fit[0]:.2f}, j{j + 1}_min_duty ~ {max(fit[1], 0.0):.2f}')

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, axs = plt.subplots(3, 2, figsize=(14, 9), sharex='col')
        for j in (0, 1):
            t_off = 0.0
            for m, (label, rows) in enumerate(moves):
                t = [r[0] - rows[0][0] + t_off for r in rows]
                q = [r[1][P.ST_Q1 + j] * DEG for r in rows]
                ref = [r[1][P.ST_Q1_REF + j] * DEG for r in rows]
                u = [r[1][P.ST_U1 + j] for r in rows]
                axs[0, j].plot(t, ref, 'k--', lw=1, label='reference' if m == 0 else None)
                axs[0, j].plot(t, q, 'C0', lw=1.5, label='position' if m == 0 else None)
                axs[1, j].plot(t, [a - b for a, b in zip(ref, q)], 'C1', lw=1.2)
                axs[2, j].plot(t, u, 'C2', lw=1.2)
                t_off = t[-1] + 0.3
            axs[0, j].set_title(f'J{j + 1}')
            axs[0, j].set_ylabel('angle [deg]')
            axs[0, j].legend(loc='best')
            axs[1, j].set_ylabel('ref - pos [deg]')
            axs[1, j].axhline(args.tol, color='grey', lw=0.6, ls=':')
            axs[1, j].axhline(-args.tol, color='grey', lw=0.6, ls=':')
            axs[2, j].set_ylabel('PWM duty')
            axs[2, j].axhline(args.max_duty, color='r', lw=0.6, ls=':')
            axs[2, j].axhline(-args.max_duty, color='r', lw=0.6, ls=':')
            axs[2, j].set_xlabel('time [s] (moves back to back)')
            for ax in axs[:, j]:
                ax.grid(True, lw=0.3)
        fig.suptitle(f'tune --pair {args.pair}')
        fig.tight_layout()
        fig.savefig(os.path.join(args.out, 'response.png'), dpi=120)
        print(f'\nsaved {args.out}/state.csv and {args.out}/response.png')
    except ImportError:
        print(f'\nsaved {args.out}/state.csv (matplotlib not available, no plot)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
