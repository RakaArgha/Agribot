#!/usr/bin/env python3
"""
sac_sil.py  -  Robot VIRTUAL di Python (software-in-the-loop).

Menggabungkan:  pose -> sac_obs -> sac_policy -> rate limiter -> kinematika -> pose
Semua di laptop, tanpa MATLAB dan tanpa robot.

Kalau hasilnya mirip testing_v4.m di MATLAB (sampai tujuan, CTE puncak ~0.29 m
di Stage 6), seluruh rantai Python terbukti benar dari ujung ke ujung.

Jalankan dari ~/bismillah :   python3 sac_sil.py
Grafik (opsional):            sudo apt install python3-matplotlib
"""
import os
import math
import numpy as np
from sac_policy import SACPolicy, baca_info
from sac_obs import ObservationBuilder

DEPLOY = 'deploy'
info = baca_info(os.path.join(DEPLOY, 'deploy_info.txt'))
L       = 2.0 * info['L_half']
Ts      = info['Ts']
N_GHOST = int(info['n_ghost'])
RATE    = info['steer_rate_max']          # rad/s, sama dg Rate Limiter Simulink


def kinematika_dual(v, df, dr, x, y, th):
    """Port dual_ackermann_kinematics_v4.m (dengan perbaikan sign(b))."""
    tf, tr = math.tan(df), math.tan(dr)
    den = tf - tr
    if abs(den) < 1e-6:
        om, vx, vy = 0.0, v*math.cos(df), v*math.sin(df)
    else:
        b = L / den
        a = L/2 - b*tf
        rc = math.hypot(a, b)
        om = math.copysign(1.0, b) * v / rc
        vx, vy = om*b, -om*a
    c, s = math.cos(th), math.sin(th)
    x += (vx*c - vy*s) * Ts
    y += (vx*s + vy*c) * Ts
    th = math.atan2(math.sin(th + om*Ts), math.cos(th + om*Ts))
    return x, y, th


def main():
    pol = SACPolicy()
    ob  = ObservationBuilder()
    P   = ob.P
    N_real = ob.N - N_GHOST
    gx, gy = P[N_real-1]
    path_asli = P[:N_real]
    print(f"Path asli: {N_real} titik | tujuan ({gx:.2f}, {gy:.2f}) | L = {L:.2f} m")

    x = y = th = 0.0
    delta_prev = 0.0
    log = []
    for k in range(600):
        obs = ob(x, y, th)
        v, delta_cmd = pol.act(obs)

        # Rate Limiter: delta maksimal berubah RATE*Ts per langkah
        step = RATE * Ts
        delta = min(max(delta_cmd, delta_prev - step), delta_prev + step)
        delta_prev = delta

        cte = np.min(np.hypot(path_asli[:, 0]-x, path_asli[:, 1]-y))
        log.append([k*Ts, x, y, th, v, delta, cte])

        x, y, th = kinematika_dual(v, delta, -delta, x, y, th)

        if math.hypot(x-gx, y-gy) < 0.3:
            print(f"SUKSES: sampai tujuan di t = {(k+1)*Ts:.1f} s")
            break
        if cte > 0.5:
            print(f"GAGAL: keluar jalur di t = {k*Ts:.1f} s (CTE {cte:.2f} m)")
            break
    else:
        print("GAGAL: tidak sampai dalam 60 s")

    log = np.array(log)
    np.savetxt('sil_log.csv', log, delimiter=',',
               header='t,x,y,theta,v,delta,cte', comments='')
    d_delta = np.diff(log[:, 5])
    print("---------- METRIK (Python SIL) ----------")
    print(f"CTE rata-rata   : {log[:,6].mean():.3f} m")
    print(f"CTE maksimum    : {log[:,6].max():.3f} m")
    print(f"RMSE CTE        : {math.sqrt((log[:,6]**2).mean()):.3f} m")
    print(f"Waktu tempuh    : {log[-1,0]:.1f} s")
    print(f"Kehalusan setir : {math.sqrt((d_delta**2).mean()):.4f} rad")
    print("-----------------------------------------")

    try:
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(3, 1, figsize=(9, 8))
        ax[0].plot(path_asli[:, 0], path_asli[:, 1], 'k--', label='Path')
        ax[0].plot(log[:, 1], log[:, 2], 'b-', lw=2, label='Robot (Python)')
        ax[0].axis('equal'); ax[0].grid(); ax[0].legend(); ax[0].set_title('Trajectory - Python SIL')
        ax[1].plot(log[:, 0], log[:, 4], 'g', label='v')
        ax[1].plot(log[:, 0], log[:, 5], 'm', label='delta'); ax[1].grid(); ax[1].legend()
        ax[2].plot(log[:, 0], log[:, 6], 'r'); ax[2].grid(); ax[2].set_ylabel('CTE (m)')
        plt.tight_layout(); plt.savefig('sil_plot.png', dpi=120); plt.show()
    except ImportError:
        print("(matplotlib belum terpasang -- data tetap tersimpan di sil_log.csv)")


if __name__ == '__main__':
    main()