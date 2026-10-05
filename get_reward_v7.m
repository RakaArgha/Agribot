function [reward, dist_to_end] = get_reward_v7(x, y, obs, v, delta, prev_delta, dist_to_end_prev, pathXY) %#codegen
% GET_REWARD_V7  Reward path-following Ackermann untuk SAC.
%
% PORT IDENTIK dengan get_reward_v6 -> tidak perlu wiring ulang di Simulink.
% (dist_to_end_prev tetap jadi input agar port sama, TAPI SUDAH TIDAK DIPAKAI.
%  Ini disengaja: progress sekarang dihitung dari proyeksi kecepatan, bukan
%  selisih arc-length antar step, sehingga tidak bisa "diledakkan" oleh
%  loncatan indeks nearest-point / unit delay yang tidak sinkron.)
%
% PERBEDAAN UTAMA vs v6
%  1. SKALA. Semua suku dirancang |r| <= ~1.3 per step (v6 bisa +20/step).
%     SAC menyetel temperatur entropi alpha relatif terhadap skala Q; reward
%     puluhan ribu per episode membuat suku entropi tidak berarti -> eksplorasi
%     mati dan critic tidak pernah konvergen (gejala: Episode Q0 = -49 padahal
%     Episode Reward = 51223).
%  2. PROGRESS = v*cos(heading_err)*Ts (perpindahan nyata sepanjang lintasan),
%     bukan dist_to_end_prev - dist_to_end. Kontinu, terbatas oleh fisika,
%     tidak bisa di-farm sambil diam.
%  3. GATING KORIDOR. Progress hanya dibayar penuh kalau dekat garis
%     (faktor exp(-(CTE/0.6)^2)). Ngebut keluar jalur = tidak dibayar.
%     Berbeda dengan align_factor v3 yang menyebabkan reward hacking: di sini
%     yang di-gate adalah PERPINDAHAN, jadi diam = 0 apa pun nilai gate-nya.
%  4. BATAS KECEPATAN ADAPTIF. v_ref diturunkan dari kelengkungan lintasan di
%     depan. Robot dihukum kalau ngebut masuk tikungan (R_min = 1.14 m,
%     tidak mungkin belok 90 derajat pada v maksimum).
%  5. TERMINAL. Bonus goal +100 dan penalti gagal -30 dijumlahkan SETELAH clamp.
%     Gunakan bersama check_isdone_v4.m (ambang CTE harus sama!).

%% ================= PARAMETER — SESUAIKAN DENGAN SETUP ANDA =================
Ts        = 0.05;   % <<< WAJIB: sample time RL Agent (detik). Salah di sini = skala reward salah.
V_MAX     = 1.0;    % m/s  kecepatan maksimum (skala aksi v)
DELTA_MAX = 0.52;   % rad  sudut setir maksimum
L_WB      = 0.65;   % m    wheelbase
CTE_FAIL  = 1.5;    % m    ambang gagal  <<< HARUS SAMA dengan check_isdone_v4.m
GATE_W    = 0.6;    % m    lebar koridor berbayar
GOAL_RAD  = 0.2;    % m    radius selesai
LOOKAHEAD = 1.2;    % m    jarak untuk mengukur kelengkungan di depan

K_PROG    = 25.0;   % bobot progress (total sepanjang lintasan ~ K_PROG * panjang_path)
K_CTE     = 1.5;
K_HEAD    = 0.8;
K_RATE    = 0.3;
K_STEER   = 0.05;
K_SPEED   = 3.0;
R_TIME    = -0.05;  % penalti waktu: diam tetap rugi
R_GOAL    = 100.0;
R_FAIL    = -30.0;
CLAMP     = 3.0;

%% ================= 1. ERROR DARI OBSERVASI =================
CTE = obs(1);
he  = atan2(sin(obs(2)), cos(obs(2)));   % bungkus ke [-pi, pi]

%% ================= 2. PANJANG PATH VALID (buang padding) =================
% pad_path.m mem-padding pathXY sampai 300 baris. Kalau padding ikut terhitung,
% nearest-point bisa mendarat di daerah padding dan dist_to_end loncat-loncat.
N = size(pathXY, 1);
dseg = sqrt(sum(diff(pathXY, 1, 1).^2, 2));   % (N-1) x 1

d0 = 1e-9;
if ~isempty(dseg)
    d0 = max(dseg(1), 1e-9);
end

M = 1;                                   % jumlah titik path yang valid
for i = 1:(N-1)
    if dseg(i) > 1e-9 && dseg(i) < 5*d0  % segmen wajar -> masih path asli
        M = i + 1;
    else                                 % duplikat (padding ulang) atau loncat besar (padding nol)
        break;
    end
end

%% ================= 3. NEAREST POINT + SISA ARC-LENGTH =================
best = inf;
idx  = 1;
for i = 1:N
    if i <= M
        dd = (pathXY(i,1) - x)^2 + (pathXY(i,2) - y)^2;
        if dd < best
            best = dd;
            idx  = i;
        end
    end
end

s_rem = 0;
for i = 1:(N-1)
    if i >= idx && i <= (M-1)
        s_rem = s_rem + dseg(i);
    end
end

% koreksi sub-segmen (proyeksi) supaya dist_to_end kontinu, tidak bertangga
if idx < M
    ax = pathXY(idx,1);   ay = pathXY(idx,2);
    ex = pathXY(idx+1,1) - ax;
    ey = pathXY(idx+1,2) - ay;
    seg2 = ex*ex + ey*ey;
    if seg2 > 1e-12
        t = ((x - ax)*ex + (y - ay)*ey) / seg2;
        t = max(0, min(1, t));
        s_rem = s_rem - t*sqrt(seg2);
    end
end
dist_to_end = max(0, s_rem);

%% ================= 4. KELENGKUNGAN DI DEPAN -> KECEPATAN AMAN =================
% Dihitung sebagai MAKS perubahan-arah per satuan panjang, dipindai pada
% beberapa sub-jendela di depan robot. (Jangan pakai kelengkungan 3-titik
% jarak jauh: pada sudut siku, tiga titik yang mengapit sudut justru terlihat
% hampir segaris dan tikungan tidak terdeteksi.)
kb    = max(1, round(0.3 / d0));    % lebar sub-jendela ~0.3 m
kappa = 0;
for m = 0:3                         % pindai sampai ~1.5 m ke depan
    j1 = min(M, idx + m*kb);
    j2 = min(M, idx + (m+1)*kb);
    j3 = min(M, idx + (m+2)*kb);
    if j3 > j2 && j2 > j1
        psi1 = atan2(pathXY(j2,2)-pathXY(j1,2), pathXY(j2,1)-pathXY(j1,1));
        psi2 = atan2(pathXY(j3,2)-pathXY(j2,2), pathXY(j3,1)-pathXY(j2,1));
        dpsi = atan2(sin(psi2-psi1), cos(psi2-psi1));
        arc  = sqrt((pathXY(j2,1)-pathXY(j1,1))^2 + (pathXY(j2,2)-pathXY(j1,2))^2) ...
             + sqrt((pathXY(j3,1)-pathXY(j2,1))^2 + (pathXY(j3,2)-pathXY(j2,2))^2);
        if arc > 1e-6
            kk = abs(dpsi) / arc;
            if kk > kappa
                kappa = kk;
            end
        end
    end
end
coder.unusedVariable(LOOKAHEAD);

kappa_max = tan(DELTA_MAX) / L_WB;        % ~0.885 1/m  (R_min ~ 1.14 m)
ratio     = min(1, kappa / max(kappa_max, 1e-9));
v_ref     = V_MAX * (1 - 0.7*ratio);      % tikungan tajam -> boleh 30% v_max saja

%% ================= 5. SUKU-SUKU REWARD (semua O(1)) =================
gate = exp(-(CTE/GATE_W)^2);              % 1 di garis, ->0 di luar koridor
ds   = v * cos(he) * Ts;                  % perpindahan sepanjang lintasan (m/step)

if ds > 0
    r_prog = K_PROG * ds * gate;          % maju hanya dibayar kalau dekat garis
else
    r_prog = K_PROG * ds;                 % mundur SELALU dihukum penuh (tanpa gate)
end

r_cte   = -K_CTE   * (CTE / CTE_FAIL)^2;              % kuadratik: 0.3 m murah, 1.2 m mahal
r_head  = -K_HEAD  * (1 - cos(he));                   % halus, 0..2, tidak meledak
ddelta  =  (delta - prev_delta) / (0.25*DELTA_MAX);
r_rate  = -K_RATE  * min(4, ddelta^2);                % dibatasi: menikung memang perlu setir cepat
r_steer = -K_STEER * (delta / DELTA_MAX)^2;           % kecil: cegah setir mentok permanen
r_speed = -K_SPEED * (max(0, v - v_ref) / V_MAX)^2;   % hukum ngebut masuk tikungan
r_time  =  R_TIME;

shaped = r_prog + r_cte + r_head + r_rate + r_steer + r_speed + r_time;
shaped = max(-CLAMP, min(CLAMP, shaped));

%% ================= 6. TERMINAL (di luar clamp) =================
r_term = 0;
if dist_to_end < GOAL_RAD
    r_term = R_GOAL;
elseif abs(CTE) > CTE_FAIL
    r_term = R_FAIL;
end

reward = shaped + r_term;

% input sengaja tidak dipakai (dipertahankan agar port Simulink tetap sama)
coder.unusedVariable(dist_to_end_prev);
end