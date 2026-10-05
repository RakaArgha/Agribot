% =========================================================================
% PENGUJIAN AGRIBOT - JALUR BERSIH
% =========================================================================
clear; clc; close all;

disp('--- 1. Memulihkan Parameter & AI ---');
Ts = 0.1; TX = 8; TY = 5; x0 = 0; y0 = 0; theta0 = 0;
assignin('base', 'Ts', Ts); assignin('base', 'TX', TX); assignin('base', 'TY', TY);
assignin('base', 'x0', x0); assignin('base', 'y0', y0); assignin('base', 'theta0', theta0);

% Memilih file agen SAC (.mat) secara interaktif
[nama_file, lokasi_folder] = uigetfile('*.mat', 'Pilih File Agen SAC');
if isequal(nama_file, 0), disp('Dibatalkan.'); return; end
load(fullfile(lokasi_folder, nama_file));

% Memastikan agentObj ada di workspace
if exist('agentObj', 'var')
    assignin('base', 'agentObj', agentObj); 
elseif exist('actorObj', 'var') && exist('criticObj', 'var') && exist('agentOpts', 'var')
    agentObj = rlSACAgent(actorObj, criticObj, agentOpts);
    assignin('base', 'agentObj', agentObj);
end
disp('Data dan Agen AI siap!');

disp('--- 2. Menjalankan Simulink Secara Langsung ---');
mdl = "Ackermann_Env";
if ~bdIsLoaded(mdl), open_system(mdl); end
out = sim(mdl, 'StopTime', '50');
disp('Simulasi Selesai!');

disp('--- 3. Membuat Grafik ---');
X_robot = out.x.Data(:);
Y_robot = out.y.Data(:);
v_robot = out.v.Data(:);
delta_robot = out.delta.Data(:);
waktu = out.tout;

% Bikin Grafik Figure
figure('Name', 'Laporan Uji Coba AGRIBOT', 'Position', [100, 100, 800, 600]);

subplot(2,1,1);
plot(X_robot, Y_robot, 'b-', 'LineWidth', 2); hold on;
plot(TX, TY, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r'); 
plot(X_robot(1), Y_robot(1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g'); 
grid on; axis equal; 
title('Grafik Lintasan (Trajectory)');
xlabel('Posisi X'); ylabel('Posisi Y');
legend('Jalur Robot', 'Target', 'Start', 'Location', 'best');

subplot(2,1,2);
plot(waktu, v_robot, 'g-', 'LineWidth', 1.5); hold on;
plot(waktu, delta_robot, 'm-', 'LineWidth', 1.5);
grid on; 
title('Sinyal Kendali (Gas & Setir)');
xlabel('Waktu (detik)'); ylabel('Aksi');
legend('Kecepatan (v)', 'Setir (\delta)', 'Location', 'best');

disp('Grafik Sukses Dibuat!');