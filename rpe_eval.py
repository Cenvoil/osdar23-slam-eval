#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Translation RPE for OSDaR23 trajectories (Delta=1 and Delta=10 frames).

External LiDAR odometry writes poses in its own body/world frame (e.g. x-forward),
while the OSDaR23 reference is a camera-centre trajectory (z-forward).  A single
constant rigid transform therefore has to be removed before comparing relative
translations; otherwise a pure 90 deg frame rotation appears as ~141 % RPE.
We estimate that transform once by Kabsch on the matched positions (rotation +
translation, no scale), apply it, and then compute the usual RPE.
"""
import os
import numpy as np
from pathlib import Path

BASE = Path(os.environ.get('SGS_RAIL_BASE', '.'))


def load_tum(path):
    rows = []
    for line in Path(path).read_text().splitlines():
        s = line.strip()
        if not s or s.startswith('#'):
            continue
        t = s.replace(',', ' ').split()
        if len(t) < 4:
            continue
        try:
            rows.append([float(x) for x in t])
        except ValueError:
            continue
    arr = np.asarray(rows, dtype=float)
    return arr[:, 0], arr[:, 1:4]


def quat_wxyz_to_R(q):
    w, x, y, z = q
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.eye(3)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


def quat_xyzw_to_R(q):
    x, y, z, w = q
    return quat_wxyz_to_R([w, x, y, z])


def load_ours(path):
    d = np.load(path)
    t = d['cam_trans'][0]                 # (3,N), w2c t
    q = d['cam_unnorm_rots'][0]           # (4,N)
    gt = d['gt_w2c_all_frames']           # (N,4,4), w2c
    N = t.shape[1]
    centers_wxyz = []
    centers_xyzw = []
    for i in range(N):
        R1 = quat_wxyz_to_R(q[:, i])
        R2 = quat_xyzw_to_R(q[:, i])
        centers_wxyz.append(-R1.T @ t[:, i])
        centers_xyzw.append(-R2.T @ t[:, i])
    gt_centers = []
    for i in range(N):
        Rg = gt[i, :3, :3]
        tg = gt[i, :3, 3]
        gt_centers.append(-Rg.T @ tg)
    return np.asarray(gt_centers), np.asarray(centers_wxyz), np.asarray(centers_xyzw)


def match_time(gt_ts, gt_pos, est_ts, est_pos, tol=0.06):
    order = np.argsort(gt_ts)
    gts = gt_ts[order]
    gpos = gt_pos[order]
    out_g, out_e = [], []
    for t, p in zip(est_ts, est_pos):
        j = int(np.searchsorted(gts, t))
        cand = []
        if j < len(gts):
            cand.append(j)
        if j > 0:
            cand.append(j - 1)
        if not cand:
            continue
        k = min(cand, key=lambda x: abs(gts[x] - t))
        if abs(gts[k] - t) <= tol:
            out_g.append(gpos[k])
            out_e.append(p)
    return np.asarray(out_g), np.asarray(out_e)


def align_se3(est, gt):
    """Rigid alignment est -> gt (no scale). Returns aligned est and rotation."""
    if len(est) < 3:
        return est, np.eye(3)
    mu_e = est.mean(axis=0)
    mu_g = gt.mean(axis=0)
    H = (est - mu_e).T @ (gt - mu_g)
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    t = mu_g - R @ mu_e
    return (R @ est.T).T + t, R


def rpe(gt, est, delta=1):
    if len(gt) <= delta:
        return float('nan')
    dg = gt[delta:] - gt[:-delta]
    de = est[delta:] - est[:-delta]
    den = np.linalg.norm(dg, axis=1)
    num = np.linalg.norm(de - dg, axis=1)
    valid = den > 0.01
    if not np.any(valid):
        return float('nan')
    return float(np.mean(num[valid] / den[valid]) * 100.0)


def rpe_len(gt, est, delta=1):
    """Step-length RPE: frame-rotation invariant, measures speed/scale error."""
    if len(gt) <= delta:
        return float('nan')
    dg = gt[delta:] - gt[:-delta]
    de = est[delta:] - est[:-delta]
    den = np.linalg.norm(dg, axis=1)
    num = np.abs(np.linalg.norm(de, axis=1) - den)
    valid = den > 0.01
    if not np.any(valid):
        return float('nan')
    return float(np.mean(num[valid] / den[valid]) * 100.0)


def rot_angle_deg(R):
    c = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.degrees(np.arccos(c)))


def eval_traj(name, seq, gt_pos, est_pos):
    est_a, R = align_se3(est_pos, gt_pos)
    n0 = min(10, len(gt_pos))
    _, R0 = align_se3(est_pos[:n0], gt_pos[:n0])
    mu_e = est_pos[:n0].mean(axis=0)
    mu_g = gt_pos[:n0].mean(axis=0)
    t0 = mu_g - R0 @ mu_e
    est_a0 = (R0 @ est_pos.T).T + t0
    return (seq, name, len(gt_pos),
            rpe(gt_pos, est_pos, 1), rpe(gt_pos, est_pos, 10),
            rpe(gt_pos, est_a, 1), rpe(gt_pos, est_a, 10),
            rpe(gt_pos, est_a0, 1), rpe(gt_pos, est_a0, 10),
            rpe_len(gt_pos, est_pos, 1), rpe_len(gt_pos, est_pos, 10),
            rot_angle_deg(R), rot_angle_deg(R0))


def ext(name, path, gt_ts, gt_pos, seq):
    ts, pos = load_tum(path)
    g, e = match_time(gt_ts, gt_pos, ts, pos)
    return eval_traj(name, seq, g, e)


def main():
    seqs = {
        '1.2': {
            'gt': BASE / 'benchmarks/data/osdar23/gt_1.2_camcenter.txt',
            'ours': BASE / 'SGS-SLAM/experiments/Railway_d/d8_gk1045_v11ctrl/params.npz',
            'ext': {
                'KISS-ICP': BASE / 'benchmarks/results/kiss_official_osdar12/traj_tum.txt',
                'GenZ-ICP': BASE / 'benchmarks/results/genz_osdar12/traj_tum.txt',
                'FAST-LIO2': BASE / 'benchmarks/results/fastlio_osdar12/osdar12_fastlio2.txt',
            },
        },
        '3.1': {
            'gt': BASE / 'benchmarks/data/osdar23/gt_3_fire_site_3.1_camcenter.txt',
            'ours': BASE / 'SGS-SLAM/experiments/OSDaR23_others/osdar23_3_fire_site_3.1_itM180/params.npz',
            'ext': {
                'KISS-ICP': BASE / 'benchmarks/results/kiss_official_osdar31/traj_tum.txt',
                'GenZ-ICP': BASE / 'benchmarks/results/genz_osdar31/traj_tum.txt',
                'FAST-LIO2': BASE / 'benchmarks/results/fastlio_osdar31/3_fire_site_3.1_fastlio2.txt',
            },
        },
        '5.1': {
            'gt': BASE / 'benchmarks/data/osdar23/gt_5.1_camcenter.txt',
            'ours': BASE / 'SGS-SLAM/experiments/Railway_d/d8_51_v10itM180/params.npz',
            'ext': {
                'KISS-ICP': BASE / 'benchmarks/results/kiss_official_osdar51/traj_tum.txt',
                'GenZ-ICP': BASE / 'benchmarks/results/genz_osdar51/traj_tum.txt',
            },
        },
    }
    print('sequence,method,frames,rpe_d1_raw,rpe_d10_raw,rpe_d1_aligned,rpe_d10_aligned,'
          'rpe_d1_first10,rpe_d10_first10,step_d1,step_d10,frame_rot_deg,first10_rot_deg')
    for seq, cfg in seqs.items():
        gt_ts, gt_pos = load_tum(cfg['gt'])
        g, c_w, c_x = load_ours(cfg['ours'])
        row = eval_traj('Ours (wxyz)', seq, g, c_w)
        print(','.join(str(x) if isinstance(x, str) else
                       (f'{x:.3f}' if isinstance(x, float) else str(x)) for x in row))
        rowx = eval_traj('Ours (xyzw check)', seq, g, c_x)
        print(','.join(str(x) if isinstance(x, str) else
                       (f'{x:.3f}' if isinstance(x, float) else str(x)) for x in rowx))
        for name, path in cfg['ext'].items():
            row = ext(name, path, gt_ts, gt_pos, seq)
            print(','.join(str(x) if isinstance(x, str) else
                           (f'{x:.3f}' if isinstance(x, float) else str(x)) for x in row))


if __name__ == '__main__':
    main()
