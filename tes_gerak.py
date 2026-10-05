#!/usr/bin/env python3
"""
tes_gerak.py  -  Mencari penyebab gerakan tersendat.

Mengirim perintah MAJU dengan RPM tetap, TANPA joystick dan TANPA logika mode.
Sesederhana mungkin, supaya kalau di sini pun tersendat, penyebabnya pasti
di CAN/motor, bukan di logika alljoy.py.

Menguji 3 hal yang berbeda antara program senior (mulus) dan program sekarang:
  A. periode pengiriman  (0.05 s sekarang -> coba 0.02, 0.01, 0.005)
  B. heartbeat           (naik tiap kirim, atau tetap)
  C. pengiriman gagal    (alljoy.py menyembunyikannya dengan 'except: pass')

CARA PAKAI:
    python3 tes_gerak.py            # periode 0.01 s, RPM 10, 5 detik
    python3 tes_gerak.py 0.02       # ganti periode
    python3 tes_gerak.py 0.01 20    # periode + RPM
    python3 tes_gerak.py 0.01 10 hb-tetap    # heartbeat tidak naik

KESELAMATAN: robot akan MAJU selama 5 detik lalu berhenti sendiri.
Pastikan area kosong. Ctrl+C kapan saja untuk berhenti.
"""
import sys, time
import can

PERIODE = float(sys.argv[1]) if len(sys.argv) > 1 else 0.01
RPM     = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
HB_MODE = sys.argv[3] if len(sys.argv) > 3 else 'hb-naik'
DURASI  = 10.0

ID_MOTOR = [0x18705501, 0x18705502, 0x18705503, 0x18705504]
ID_KEMUDI = 0x123
CENTER = 180.0


def checksum(d):
    c = sum(d[:6])
    return (c >> 8) & 0xFF, c & 0xFF


def paket_motor(speed, hb):
    arah = 0x8000 if speed < 0 else 0x4000
    cmd = arah | (int(abs(speed)) & 0x3FFF)
    pre = [(cmd >> 8) & 0xFF, cmd & 0xFF, 0, 0, 0, hb]
    return pre + list(checksum(pre))


def paket_kemudi(enable):
    s = int(round(CENTER * 10))
    return [(s >> 8) & 0xFF, s & 0xFF, (s >> 8) & 0xFF, s & 0xFF, 0, 0, 0,
            0x01 if enable else 0x00]


def kirim_semua(bus0, bus1, rpm, hb, stat):
    try:
        bus1.send(can.Message(arbitration_id=ID_KEMUDI, data=paket_kemudi(False),
                              is_extended_id=False), timeout=0.05)
        stat['ok'] += 1
    except Exception as e:
        stat['gagal'] += 1
        stat['pesan'] = str(e)
    d = paket_motor(rpm, hb)
    for mid in ID_MOTOR:
        try:
            bus0.send(can.Message(arbitration_id=mid, data=d, is_extended_id=True),
                      timeout=0.05)
            stat['ok'] += 1
        except Exception as e:
            stat['gagal'] += 1
            stat['pesan'] = str(e)


def main():
    bus0 = can.interface.Bus(interface='kvaser', channel=0, bitrate=250000)
    bus1 = can.interface.Bus(interface='kvaser', channel=1, bitrate=250000)
    print(f"periode={PERIODE*1000:.0f} ms ({1/PERIODE:.0f} Hz) | RPM={RPM} | "
          f"heartbeat={HB_MODE} | durasi={DURASI} s")
    print("Robot akan MAJU. Ctrl+C untuk berhenti.\n")
    time.sleep(1.0)

    stat = {'ok': 0, 'gagal': 0, 'pesan': ''}
    hb = 0x01
    jitter = []
    t0 = time.time()
    t_next = t0
    try:
        while time.time() - t0 < DURASI:
            t_kirim = time.time()
            kirim_semua(bus0, bus1, RPM, hb, stat)
            if HB_MODE == 'hb-naik':
                hb = (hb + 1) % 0xFF or 0x01
            elif HB_MODE == 'hb-aman':
                hb = hb + 1 if hb < 100 else 1
            jitter.append(time.time() - t_kirim)
            t_next += PERIODE
            tidur = t_next - time.time()
            if tidur > 0:
                time.sleep(tidur)
    except KeyboardInterrupt:
        print("\ndihentikan pengguna")
    finally:
        for _ in range(20):                       # berhenti: RPM 0, enable off
            kirim_semua(bus0, bus1, 0.0, hb, {'ok': 0, 'gagal': 0, 'pesan': ''})
            time.sleep(0.01)
        bus1.send(can.Message(arbitration_id=ID_KEMUDI, data=paket_kemudi(False),
                              is_extended_id=False))
        bus0.shutdown(); bus1.shutdown()

    n = len(jitter)
    print(f"\nPaket terkirim OK : {stat['ok']}")
    print(f"Paket GAGAL       : {stat['gagal']}   <-- harus 0")
    if stat['gagal']:
        print(f"  contoh error    : {stat['pesan']}")
    print(f"Siklus            : {n} dalam {DURASI:.0f} s = {n/DURASI:.0f} Hz")
    print(f"Waktu kirim 1 siklus: rata2 {1000*sum(jitter)/n:.2f} ms | "
          f"maks {1000*max(jitter):.2f} ms")
    if 1000*max(jitter) > PERIODE*1000:
        print("  >>> Pengiriman lebih lama dari periode: bus/CPU tidak sanggup secepat ini.")


if __name__ == '__main__':
    main()
