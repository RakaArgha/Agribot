#!/usr/bin/env python3
"""
can_sniff.py  -  Rekam semua pesan CAN di kedua kanal Kvaser.

TUJUAN: menemukan apakah pengendali motor penggerak mengirim balik data
kecepatan/encoder. alljoy.py hanya MENGIRIM; kita perlu tahu apa yang DITERIMA.

CARA PAKAI (2 terminal):
  Terminal 1:  python3 alljoy.py          (biarkan jalan, tekan Y lalu B)
  Terminal 2:  python3 can_sniff.py 20    (rekam 20 detik)
  Selama rekaman: gerakkan robot PELAN dengan joystick (atau roda diangkat).

Kalau Kvaser menolak dibuka dua kali, matikan alljoy.py dan rekam saat
robot diam -- pesan umpan balik biasanya tetap terkirim periodik.
"""
import sys, time, csv
from collections import defaultdict
import can

DURASI = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0

# ID yang DIKIRIM alljoy.py (bukan umpan balik)
ID_ALLJOY = {0x123, 0x18705501, 0x18705502, 0x18705503, 0x18705504}

buses = {
    0: can.interface.Bus(interface='kvaser', channel=0, bitrate=250000),
    1: can.interface.Bus(interface='kvaser', channel=1, bitrate=250000),
}

stat = defaultdict(lambda: {'n': 0, 't0': None, 't1': None, 'contoh': []})
log = []
print(f"Merekam {DURASI:.0f} detik di kanal 0 dan 1 ...")
t_end = time.time() + DURASI

while time.time() < t_end:
    for ch, bus in buses.items():
        msg = bus.recv(timeout=0.005)
        if msg is None:
            continue
        key = (ch, msg.arbitration_id)
        s = stat[key]
        s['n'] += 1
        s['t0'] = s['t0'] or msg.timestamp
        s['t1'] = msg.timestamp
        data = bytes(msg.data).hex(' ')
        if len(s['contoh']) < 5 or data != s['contoh'][-1]:
            s['contoh'].append(data)
            s['contoh'] = s['contoh'][-8:]
        log.append([f"{msg.timestamp:.4f}", ch, hex(msg.arbitration_id),
                    msg.is_extended_id, msg.dlc, data])

for bus in buses.values():
    bus.shutdown()

with open('can_log.csv', 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['t', 'kanal', 'id', 'extended', 'dlc', 'data'])
    w.writerows(log)

print("\n==================== RINGKASAN ====================")
print(f"{'kanal':>5} {'ID':>12} {'jumlah':>7} {'Hz':>6}  sumber      contoh data (berubah-ubah)")
for (ch, mid), s in sorted(stat.items()):
    dt = (s['t1'] - s['t0']) if s['n'] > 1 else 0
    hz = (s['n'] - 1) / dt if dt > 0 else 0
    sumber = 'alljoy' if mid in ID_ALLJOY else '>>> LAIN <<<'
    print(f"{ch:>5} {hex(mid):>12} {s['n']:>7} {hz:>6.1f}  {sumber:12s}{s['contoh'][0]}")
    for c in s['contoh'][1:4]:
        print(f"{'':>45}{c}")
print("===================================================")
print("Semua pesan tersimpan di can_log.csv")
print("Baris bertanda '>>> LAIN <<<' adalah kandidat umpan balik encoder.")