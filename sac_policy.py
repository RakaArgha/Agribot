#!/usr/bin/env python3
"""
sac_policy.py  -  Actor SAC dihitung ulang dengan numpy (tanpa ONNX).

Dipakai dua cara:
  1. python3 sac_policy.py      -> VERIFIKASI terhadap ref_act.csv dari MATLAB
  2. from sac_policy import SACPolicy   -> dipakai oleh controller robot nanti

Jalankan dari folder yang berisi subfolder deploy/ :
  deploy/actor_weights.mat, deploy/ref_obs.csv, deploy/ref_act.csv, deploy/deploy_info.txt

Butuh: pip install numpy scipy
"""
import os
import numpy as np
from scipy.io import loadmat

DEPLOY = 'deploy'


def baca_info(path=os.path.join(DEPLOY, 'deploy_info.txt')):
    info = {}
    with open(path) as f:
        for baris in f:
            if '=' in baris:
                k, v = baris.strip().split('=', 1)
                try:
                    info[k] = float(v)
                except ValueError:
                    info[k] = v
    return info


class SACPolicy:
    """Jalur deterministik: fc_1 -> ReLU -> fc_body -> ReLU -> fc_mean -> skala."""

    def __init__(self, deploy_dir=DEPLOY, mode='tanh_scale'):
        w = loadmat(os.path.join(deploy_dir, 'actor_weights.mat'))
        self.W1, self.b1 = w['W_fc_1'],    w['b_fc_1'].ravel()
        self.W2, self.b2 = w['W_fc_body'], w['b_fc_body'].ravel()
        self.Wm, self.bm = w['W_fc_mean'], w['b_fc_mean'].ravel()

        info = baca_info(os.path.join(deploy_dir, 'deploy_info.txt'))
        self.lb = np.array([info['v_min'], -info['delta_max_virtual']])
        self.ub = np.array([info['v_max'],  info['delta_max_virtual']])
        self.mode = mode

    def mean(self, obs):
        h = np.maximum(0.0, self.W1 @ obs + self.b1)
        h = np.maximum(0.0, self.W2 @ h + self.b2)
        return self.Wm @ h + self.bm

    def skala(self, mu, mode=None):
        mode = mode or self.mode
        if mode == 'tanh_scale':      # tanh lalu dipetakan ke [lb, ub]
            return self.lb + (np.tanh(mu) + 1.0) * 0.5 * (self.ub - self.lb)
        if mode == 'clip':            # mean langsung, dipotong ke batas
            return np.clip(mu, self.lb, self.ub)
        if mode == 'raw':             # mean apa adanya
            return mu
        raise ValueError(mode)

    def act(self, obs):
        """obs: array 23 elemen -> np.array([v, delta_virtual])"""
        return self.skala(self.mean(np.asarray(obs, dtype=float)))


if __name__ == '__main__':
    obs = np.loadtxt(os.path.join(DEPLOY, 'ref_obs.csv'), delimiter=',')
    ref = np.loadtxt(os.path.join(DEPLOY, 'ref_act.csv'), delimiter=',')
    pol = SACPolicy()

    mu = np.array([pol.mean(o) for o in obs])
    print(f"Sampel: {len(obs)} | batas v [{pol.lb[0]:.3f}, {pol.ub[0]:.3f}]"
          f" | batas delta +-{pol.ub[1]:.4f} rad\n")
    print(f"{'mode':12s} {'err maks v':>12s} {'err maks delta':>15s}")
    hasil = {}
    for mode in ['tanh_scale', 'clip', 'raw']:
        a = np.array([pol.skala(m, mode) for m in mu])
        err = np.abs(a - ref).max(axis=0)
        hasil[mode] = err.max()
        print(f"{mode:12s} {err[0]:12.2e} {err[1]:15.2e}")

    terbaik = min(hasil, key=hasil.get)
    print()
    if hasil[terbaik] < 1e-4:
        print(f">>> COCOK: mode '{terbaik}' identik dengan MATLAB "
              f"(error maks {hasil[terbaik]:.1e}). Porting BENAR.")
    else:
        print(f">>> TIDAK ADA YANG COCOK (terbaik '{terbaik}', error {hasil[terbaik]:.3f}).")
        print("    Kirim output ini ke Claude -- JANGAN dipakai di robot dulu.")