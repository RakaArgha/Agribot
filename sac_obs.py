#!/usr/bin/env python3
"""
sac_obs.py  -  Port get_observation_v6.m ke Python (numpy).

Dipakai dua cara:
  1. python3 sac_obs.py     -> VERIFIKASI terhadap deploy/obs_ref.csv
  2. from sac_obs import ObservationBuilder   -> dipakai controller robot

CATATAN INDEKS: MATLAB mulai dari 1, Python dari 0. Supaya bisa dibandingkan
langsung dengan MATLAB, nilai idx di file ini TETAP dihitung mulai 1, dan
baru dikurangi 1 saat mengambil titik dari array (P[i-1]).
"""
import os
import math
import numpy as np

DEPLOY = 'deploy'

N_LOOKAHEAD = 10
STRIDE      = 5
WINDOW_BACK = 2
WINDOW_FWD  = 5
SKALA_WP    = 5.0


def panjang_path_asli(P):
    """Index (mulai 1) titik terakhir yang bukan padding (padding = titik diulang)."""
    M = P.shape[0]
    for i in range(M, 1, -1):
        dx = P[i-1, 0] - P[i-2, 0]
        dy = P[i-1, 1] - P[i-2, 1]
        if dx*dx + dy*dy > 1e-12:
            return i
    return 1


def get_observation(x, y, theta, P, idx_prev, N=None):
    """Identik dengan get_observation_v6.m. Mengembalikan (obs[23], idx_now)."""
    if N is None:
        N = panjang_path_asli(P)

    idx_prev = int(round(idx_prev))
    idx_prev = min(max(idx_prev, 1), N)

    lo = max(1, idx_prev - WINDOW_BACK)
    hi = min(N, idx_prev + WINDOW_FWD)

    best_d, best_i = math.inf, idx_prev
    for i in range(lo, hi + 1):
        d = (P[i-1, 0] - x)**2 + (P[i-1, 1] - y)**2
        if d < best_d:                      # '<' : titik pertama yang menang, sama dg MATLAB
            best_d, best_i = d, i
    idx_now = best_i
    dist_nearest = math.sqrt(best_d)

    if idx_now < N:
        seg_dx = P[idx_now, 0] - P[idx_now-1, 0]
        seg_dy = P[idx_now, 1] - P[idx_now-1, 1]
    elif N > 1:
        seg_dx = P[N-1, 0] - P[N-2, 0]
        seg_dy = P[N-1, 1] - P[N-2, 1]
    else:
        seg_dx, seg_dy = 1.0, 0.0
    theta_path = math.atan2(seg_dy, seg_dx)

    heading_err = theta - theta_path
    heading_err = math.atan2(math.sin(heading_err), math.cos(heading_err))

    tx = x - P[idx_now-1, 0]
    ty = y - P[idx_now-1, 1]
    cte = -math.sin(theta_path)*tx + math.cos(theta_path)*ty

    rel_wp = np.zeros(2*N_LOOKAHEAD)
    ct, st = math.cos(-theta), math.sin(-theta)
    for k in range(1, N_LOOKAHEAD + 1):
        idx_k = min(idx_now + k*STRIDE, N)
        gx = P[idx_k-1, 0] - x
        gy = P[idx_k-1, 1] - y
        rel_wp[2*k-2] = (ct*gx - st*gy) / SKALA_WP
        rel_wp[2*k-1] = (st*gx + ct*gy) / SKALA_WP

    obs = np.concatenate(([cte, heading_err/math.pi, dist_nearest], rel_wp))
    return obs, idx_now


class ObservationBuilder:
    """Menyimpan path dan idx_prev antar langkah, seperti Unit Delay di Simulink."""

    def __init__(self, path_csv=os.path.join(DEPLOY, 'path_padded.csv')):
        self.P = np.loadtxt(path_csv, delimiter=',')
        self.N = panjang_path_asli(self.P)
        self.reset()

    def reset(self):
        self.idx_prev = 1          # = idx0 di MATLAB

    def __call__(self, x, y, theta):
        obs, self.idx_prev = get_observation(x, y, theta, self.P, self.idx_prev, self.N)
        return obs


if __name__ == '__main__':
    P   = np.loadtxt(os.path.join(DEPLOY, 'path_padded.csv'), delimiter=',')
    ref = np.loadtxt(os.path.join(DEPLOY, 'obs_ref.csv'),     delimiter=',')
    N = panjang_path_asli(P)
    print(f"Path: {P.shape[0]} baris, titik bermakna N = {N} | langkah uji: {len(ref)}")

    # --- uji 1: tiap langkah pakai idx_prev dari MATLAB (menguji rumus) ---
    err_obs, idx_salah = 0.0, 0
    for row in ref:
        x, y, th, ip = row[0], row[1], row[2], row[3]
        obs_m, idx_m = row[4:27], int(row[27])
        obs_p, idx_p = get_observation(x, y, th, P, ip, N)
        err_obs = max(err_obs, np.abs(obs_p - obs_m).max())
        idx_salah += (idx_p != idx_m)

    # --- uji 2: berantai, idx_prev dibawa sendiri (menguji "Unit Delay") ---
    ob = ObservationBuilder(os.path.join(DEPLOY, 'path_padded.csv'))
    err_rantai = 0.0
    for row in ref:
        err_rantai = max(err_rantai, np.abs(ob(row[0], row[1], row[2]) - row[4:27]).max())

    print(f"Uji 1 (per langkah): error obs maks = {err_obs:.2e} | idx berbeda = {idx_salah}")
    print(f"Uji 2 (berantai)   : error obs maks = {err_rantai:.2e}")
    if err_obs < 1e-9 and idx_salah == 0 and err_rantai < 1e-9:
        print(">>> COCOK: observasi Python identik dengan MATLAB. Porting BENAR.")
    else:
        print(">>> TIDAK COCOK. Kirim output ini ke Claude -- JANGAN dipakai di robot dulu.")