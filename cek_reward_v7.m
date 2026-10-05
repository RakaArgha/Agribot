%% CEK_REWARD_V7  Verifikasi reward SEBELUM training (hemat 36 menit per iterasi)
%
% Jalankan:  >> cek_reward_v7
%
% Tujuan: membuktikan secara numerik bahwa urutan reward per-step sudah benar
%
%     LURUS DI GARIS  >  MENIKUNG BENAR  >  DIAM  >  NGEBUT KELUAR JALUR
%
% Kalau urutan ini SALAH, jangan training. Perbaiki bobot dulu di sini.
% Ini pelajaran dari v3/v4/v5/v6: empat iterasi reward dibakar dengan cara
% "ubah bobot -> training 36 menit -> lihat gagal". Tabel di bawah menjawab
% pertanyaan yang sama dalam 2 detik.

clear; clc;

%% ---------------- Path uji: L 90 derajat, resolusi 0.05 m ----------------
res = 0.05;
seg1 = [(0:res:5)',        zeros(numel(0:res:5),1)];          % lurus x: 0->5
seg2 = [5*ones(numel(res:res:5),1), (res:res:5)'];            % naik  y: 0->5
P = [seg1; seg2];
P = [P; repmat(P(end,:), 300-size(P,1), 1)];                  % padding ala pad_path.m

%% ---------------- Skenario uji ----------------
% kolom: nama, x, y, CTE, heading_err, v, delta, prev_delta
S = {
%  nama                          x      y     CTE    he      v     delta  prev
  'Lurus di garis (v maks)'      2.00,  0.00,  0.00,  0.00,  1.00,  0.00,  0.00
  'Mendekat ke garis'            2.00,  0.20,  0.20, -0.15,  0.90,  0.10,  0.08
  'DIAM sebelum tikungan'        4.30,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00
  'Menikung benar (pelan)'       5.10,  0.60,  0.30,  0.30,  0.30,  0.42,  0.40
  'Menikung agak cepat'          5.10,  0.60,  0.25,  0.25,  0.55,  0.45,  0.40
  'Pelan msk tikungan (benar)'   4.30,  0.00,  0.00,  0.00,  0.30,  0.10,  0.05
  'NGEBUT msk tikungan (salah)'  4.30,  0.00,  0.00,  0.00,  1.00,  0.00,  0.00
  'Menerobos tikungan (lurus)'   5.80,  0.30,  0.80,  0.90,  1.00,  0.00,  0.00
  'Keluar jalur (gagal)'         6.60,  0.30,  1.60,  1.20,  1.00,  0.00,  0.00
  'Mundur di garis'              2.00,  0.00,  0.00,  3.14,  0.50,  0.00,  0.00
  'Setir goyang (jitter)'        2.00,  0.00,  0.00,  0.00,  1.00,  0.30, -0.30
};

fprintf('\n%-30s %8s %10s\n', 'SKENARIO', 'REWARD', 'dist_end');
fprintf('%s\n', repmat('-', 1, 50));

R = zeros(size(S,1),1);
for k = 1:size(S,1)
    obs = zeros(23,1);
    obs(1) = S{k,4};    % CTE
    obs(2) = S{k,5};    % heading_err
    [R(k), de] = get_reward_v7(S{k,2}, S{k,3}, obs, S{k,6}, S{k,7}, S{k,8}, 0, P);
    fprintf('%-30s %8.3f %10.3f\n', S{k,1}, R(k), de);
end

%% ---------------- Uji urutan (yang benar-benar menentukan perilaku) ----------------
fprintf('\n=== UJI URUTAN ===\n');
lurus    = R(1);
diam     = R(3);
menikung = R(4);
pelan    = R(6);
ngebut   = R(7);
nerobos  = R(8);

ok1 = lurus    > menikung;
ok2 = menikung > diam;
ok3 = diam     > nerobos;
ok4 = pelan    > ngebut;

fprintf('  lurus (%.3f) > menikung (%.3f)        : %s\n', lurus, menikung, tf(ok1));
fprintf('  menikung (%.3f) > diam (%.3f)         : %s\n', menikung, diam, tf(ok2));
fprintf('  diam (%.3f) > nerobos (%.3f)          : %s\n', diam, nerobos, tf(ok3));
fprintf('  pelan-msk-tikungan (%.3f) > ngebut (%.3f) : %s\n', pelan, ngebut, tf(ok4));
fprintf('  margin menikung-vs-diam               : %+.3f per step\n', menikung - diam);

if ok1 && ok2 && ok3 && ok4 && (menikung - diam) > 0.05
    fprintf('\n  >> LOLOS. Reward layak dipakai training.\n');
else
    fprintf(['\n  >> BELUM LOLOS. Naikkan K_PROG atau turunkan K_CTE/K_RATE\n' ...
             '     di get_reward_v7.m, lalu jalankan ulang skrip ini.\n']);
end

%% ---------------- Perkiraan reward episode ----------------
Ts = 0.05;  V = 1.0;  panjang = 10.0;
n_lurus = panjang / (V*Ts);
fprintf('\n=== PERKIRAAN SKALA EPISODE ===\n');
fprintf('  Episode BERHASIL (ideal) : ~ %.0f  (progress ~%.0f + goal 100)\n', ...
        25*panjang + 100 - 0.05*n_lurus, 25*panjang);
fprintf('  Episode DIAM saja        : ~ %.0f\n', -0.05*1200);
fprintf('  Episode keluar jalur     : ~ -30 sampai -60 (episode langsung putus)\n');
fprintf(['\n  Kalau kurva Training Monitor nanti menembus 10^3 atau 10^4,\n' ...
         '  itu BUKAN keberhasilan -- itu reward hacking. Hentikan training.\n\n']);

function s = tf(b)
    if b, s = 'OK'; else, s = 'GAGAL'; end
end