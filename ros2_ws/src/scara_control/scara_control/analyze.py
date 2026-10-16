"""Accuracy & precision from characterization CSV files (ISO 9283 metrics).

    ros2 run scara_control analyze run1.csv [run2.csv ...] [--source enc] [--outdir results]

For each commanded point P_j with n attained positions p_i:
    barycenter        p_bar = mean(p_i)
    ACCURACY  AP      = || p_bar - p_cmd ||            (ISO 9283 pose accuracy)
    mean error        = mean_i || p_i - p_cmd ||       ("error medio", as asked in the guide)
    PRECISION RP      = l_bar + 3 S_l,  l_i = || p_i - p_bar ||   (ISO 9283 repeatability)
    std x / y         = sample standard deviation per axis

--source meas (default) uses the ground-truth columns; --source enc uses the
encoder-based forward kinematics (internal repeatability only: it can NOT
show calibration errors such as wrong link lengths, so it is not ground truth).
"""
import argparse
import csv
import os
import sys

import numpy as np


def load(files, source):
    pts = {}
    for path in files:
        with open(path) as f:
            for r in csv.DictReader(f):
                if r['status'].startswith('FAIL'):
                    continue
                kx, ky = ('meas_x_mm', 'meas_y_mm') if source == 'meas' else ('enc_x_mm', 'enc_y_mm')
                if not r[kx] or not r[ky]:
                    continue
                key = int(r['point'])
                d = pts.setdefault(key, {'cmd': (float(r['cmd_x_mm']), float(r['cmd_y_mm'])), 'p': []})
                d['p'].append((float(r[kx]), float(r[ky])))
    return pts


def metrics(cmd, p):
    p = np.asarray(p)
    cmd = np.asarray(cmd)
    bar = p.mean(axis=0)
    l = np.linalg.norm(p - bar, axis=1)
    s_l = l.std(ddof=1) if len(p) > 1 else 0.0
    return {
        'n': len(p), 'bar': bar,
        'AP': float(np.linalg.norm(bar - cmd)),
        'APx': float(bar[0] - cmd[0]), 'APy': float(bar[1] - cmd[1]),
        'mean_err': float(np.linalg.norm(p - cmd, axis=1).mean()),
        'max_err': float(np.linalg.norm(p - cmd, axis=1).max()),
        'RP': float(l.mean() + 3 * s_l),
        'sx': float(p[:, 0].std(ddof=1)) if len(p) > 1 else 0.0,
        'sy': float(p[:, 1].std(ddof=1)) if len(p) > 1 else 0.0,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('csv', nargs='+')
    ap.add_argument('--source', choices=['meas', 'enc'], default='meas')
    ap.add_argument('--outdir', default='characterization_results')
    args = ap.parse_args()

    pts = load(args.csv, args.source)
    if not pts:
        print(f'no usable rows for source "{args.source}" (fill meas_x_mm/meas_y_mm or use --source enc)')
        return 1
    os.makedirs(args.outdir, exist_ok=True)

    res = {k: metrics(v['cmd'], v['p']) for k, v in sorted(pts.items())}
    hdr = ('| Point | cmd x, y [mm] | n | mean x, y [mm] | bias x / y [mm] | **AP** accuracy [mm] '
           '| mean error [mm] | max error [mm] | std x / y [mm] | **RP** precision [mm] |')
    lines = [f'Source: {"ground truth" if args.source == "meas" else "encoders (internal)"}', '',
             hdr, '|' + '---|' * 10]
    for k, m in res.items():
        c = pts[k]['cmd']
        lines.append(f'| P{k} | {c[0]:.1f}, {c[1]:.1f} | {m["n"]} | {m["bar"][0]:.2f}, {m["bar"][1]:.2f} '
                     f'| {m["APx"]:+.2f} / {m["APy"]:+.2f} | {m["AP"]:.2f} | {m["mean_err"]:.2f} '
                     f'| {m["max_err"]:.2f} | {m["sx"]:.2f} / {m["sy"]:.2f} | {m["RP"]:.2f} |')
    aps = [m['AP'] for m in res.values()]
    rps = [m['RP'] for m in res.values()]
    mes = [m['mean_err'] for m in res.values()]
    lines += ['', f'Overall: AP mean {np.mean(aps):.2f} mm (worst {max(aps):.2f}), '
                  f'mean error {np.mean(mes):.2f} mm, RP mean {np.mean(rps):.2f} mm (worst {max(rps):.2f})']
    table = '\n'.join(lines)
    print(table)
    with open(os.path.join(args.outdir, f'results_{args.source}.md'), 'w') as f:
        f.write(table + '\n')

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        return 0

    # 1) workspace overview
    fig, ax = plt.subplots(figsize=(7, 7))
    for k, v in sorted(pts.items()):
        p = np.asarray(v['p'])
        ax.scatter(p[:, 0], p[:, 1], s=12, label=f'P{k} attained')
        ax.plot(*v['cmd'], 'k+', ms=14, mew=2)
        ax.annotate(f'P{k}', v['cmd'], textcoords='offset points', xytext=(8, 8))
    ax.plot(0, 0, 'ks')
    ax.set_aspect('equal')
    ax.set_xlabel('x [mm]')
    ax.set_ylabel('y [mm]')
    ax.set_title('Commanded (+) vs attained positions')
    ax.grid(alpha=.3)
    ax.legend(fontsize=8)
    fig.savefig(os.path.join(args.outdir, f'workspace_{args.source}.png'), dpi=150, bbox_inches='tight')

    # 2) zoom per point, with RP circle around the barycenter
    n = len(pts)
    cols = min(n, 3)
    rows = int(np.ceil(n / cols))
    fig, axs = plt.subplots(rows, cols, figsize=(4 * cols, 4 * rows), squeeze=False)
    for ax, (k, v) in zip(axs.flat, sorted(pts.items())):
        m = res[k]
        p = np.asarray(v['p']) - np.asarray(v['cmd'])
        bar = m['bar'] - np.asarray(v['cmd'])
        ax.scatter(p[:, 0], p[:, 1], s=14)
        ax.plot(0, 0, 'k+', ms=14, mew=2, label='commanded')
        ax.plot(*bar, 'rx', ms=10, label='barycenter')
        ax.add_patch(plt.Circle(bar, m['RP'], fill=False, color='r', ls='--', label='RP'))
        ax.set_aspect('equal')
        ax.set_title(f'P{k}: AP={m["AP"]:.2f} RP={m["RP"]:.2f} mm', fontsize=10)
        ax.set_xlabel('dx [mm]')
        ax.set_ylabel('dy [mm]')
        ax.grid(alpha=.3)
    axs.flat[0].legend(fontsize=7)
    for ax in list(axs.flat)[n:]:
        ax.axis('off')
    fig.tight_layout()
    fig.savefig(os.path.join(args.outdir, f'per_point_{args.source}.png'), dpi=150)

    # 3) bar chart
    fig, ax = plt.subplots(figsize=(7, 4))
    idx = np.arange(n)
    ax.bar(idx - 0.2, aps, 0.4, label='AP accuracy')
    ax.bar(idx + 0.2, rps, 0.4, label='RP precision')
    ax.set_xticks(idx, [f'P{k}' for k in sorted(pts)])
    ax.set_ylabel('mm')
    ax.legend()
    ax.grid(axis='y', alpha=.3)
    fig.savefig(os.path.join(args.outdir, f'ap_rp_{args.source}.png'), dpi=150, bbox_inches='tight')
    print(f'\nplots and table saved in {args.outdir}/')
    return 0


if __name__ == '__main__':
    sys.exit(main())
