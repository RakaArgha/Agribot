% =========================================================================
% PENGUJIAN SAC v2 - Path Following dengan Lookahead
% =========================================================================
clear; clc; close all;
disp('--- 1. Memulihkan Parameter Path & AI ---');

% Path harus SAMA persis dengan yang dipakai saat training terakhir
seg1_len = 5; seg2_len = 0; spacing = 0.1;
pathXY_raw = generate_L_path(seg1_len, seg2_len, spacing);
MAX_PATH_SIZE = 300;
pathXY_padded = pad_path(pathXY_raw, MAX_PATH_SIZE);

x0 = 0; y0 = 0; theta0 = 0;
dist0 = sqrt((pathXY_padded(end,1)-x0)^2 + (pathXY_padded(end,2)-y0)^2);

assignin('base', 'pathXY_padded', pathXY_padded);
assignin('base', 'x0', x0);
assignin('base', 'y0', y0);
assignin('base', 'theta0', theta0);
assignin('base', 'dist0', dist0);
assignin('base', 'Ts', 0.1);

% Pilih file agen SAC (.mat)
[nama_file, lokasi_folder] = uigetfile('*.mat', 'Pilih File Agen SAC');
if isequal(nama_file, 0), disp('Dibatalkan.'); return; end
load(fullfile(lokasi_folder, nama_file));

if exist('agentObj', 'var')
    % PENTING: matikan exploration noise supaya hasil tidak jitter
    agentObj.UseExplorationPolicy = false;
    assignin('base', 'agentObj', agentObj);
else
    error('Variabel agentObj tidak ditemukan di file .mat ini.');
end
disp('Data dan Agen AI siap (exploration OFF untuk hasil mulus)!');

disp('--- 2. Menjalankan Simulink ---');
mdl = "Ackermann_Env";
if ~bdIsLoaded(mdl), open_system(mdl); end
out = sim(mdl, 'StopTime', '60');
disp('Simulasi Selesai!');

disp('--- 3. Membuat Grafik ---');
X_robot = out.x.Data(:);
Y_robot = out.y.Data(:);
v_robot = out.v.Data(:);
delta_robot = out.delta.Data(:);
waktu = out.tout;

figure('Name', 'Laporan Uji Coba AGRIBOT v2', 'Position', [100, 100, 800, 600]);
subplot(2,1,1);
plot(pathXY_raw(:,1), pathXY_raw(:,2), 'k--', 'LineWidth', 1); hold on;
plot(X_robot, Y_robot, 'b-', 'LineWidth', 2);
plot(pathXY_raw(end,1), pathXY_raw(end,2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
plot(X_robot(1), Y_robot(1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
grid on; axis equal;
title('Grafik Lintasan (Trajectory) vs Path Target');
xlabel('Posisi X'); ylabel('Posisi Y');
legend('Path Target', 'Jalur Robot', 'Titik Akhir', 'Start', 'Location', 'best');

subplot(2,1,2);
plot(waktu, v_robot, 'g-', 'LineWidth', 1.5); hold on;
plot(waktu, delta_robot, 'm-', 'LineWidth', 1.5);
grid on;
title('Sinyal Kendali (Gas & Setir)');
xlabel('Waktu (detik)'); ylabel('Aksi');
legend('Kecepatan (v)', 'Setir (\delta)', 'Location', 'best');

disp('Grafik Sukses Dibuat!');