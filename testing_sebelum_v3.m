% =========================================================================
% PENGUJIAN SAC v3 - Curriculum-based, konsisten dengan training_v3.m
% =========================================================================
clear; clc; close all;

STAGE = 1;  % <<< SAMAKAN dengan stage yang mau dites

[seg1_len, seg2_len, spacing, slip_enabled, agent_filename, ~] = get_curriculum_config(STAGE);

pathXY_raw = generate_L_path(seg1_len, seg2_len, spacing);
MAX_PATH_SIZE = 300;
pathXY_padded = pad_path(pathXY_raw, MAX_PATH_SIZE);

fprintf('=== TESTING STAGE %d ===\n', STAGE);
fprintf('Titik akhir path: (%.2f, %.2f)\n', pathXY_raw(end,1), pathXY_raw(end,2));

x0 = 0; y0 = 0; theta0 = 0;
dist0 = sqrt((pathXY_padded(end,1)-x0)^2 + (pathXY_padded(end,2)-y0)^2);
if slip_enabled
    slip_factor = 0.85;   % nilai tetap (tengah-tengah) untuk testing, bukan acak
else
    slip_factor = 1.0;
end
fprintf('slip_enabled=%d, slip_factor dipakai=%.2f\n', slip_enabled, slip_factor);

assignin('base', 'pathXY_padded', pathXY_padded);
assignin('base', 'x0', x0);
assignin('base', 'y0', y0);
assignin('base', 'theta0', theta0);
assignin('base', 'dist0', dist0);
assignin('base', 'slip_factor', slip_factor);
assignin('base', 'Ts', 0.1);

default_path = fullfile(pwd, 'SavedAgents', agent_filename);
if isfile(default_path)
    fprintf('Agent ditemukan otomatis: %s\n', agent_filename);
    load(default_path);
else
    fprintf('Agent default (%s) tidak ditemukan, silakan pilih manual.\n', agent_filename);
    [nama_file, lokasi_folder] = uigetfile('*.mat', 'Pilih File Agen SAC');
    if isequal(nama_file, 0), disp('Dibatalkan.'); return; end
    load(fullfile(lokasi_folder, nama_file));
end

if ~exist('agentObj', 'var'), error('agentObj tidak ditemukan di file .mat ini.'); end
agentObj.UseExplorationPolicy = false;   % matikan noise -> hasil tidak jitter
assignin('base', 'agentObj', agentObj);

disp('--- Menjalankan Simulink ---');
mdl = "Ackermann_Env";
if ~bdIsLoaded(mdl), open_system(mdl); end
out = sim(mdl, 'StopTime', '60');
disp('Simulasi selesai!');

X_robot = out.x.Data(:); Y_robot = out.y.Data(:);
v_robot = out.v.Data(:); delta_robot = out.delta.Data(:);
waktu = out.tout;

% Potong data setelah robot pertama kali sampai radius 0.2m dari target,
% supaya grafik tidak menampilkan robot "nyasar terus" setelah tugas selesai.
dist_to_target = sqrt((pathXY_raw(end,1)-X_robot).^2 + (pathXY_raw(end,2)-Y_robot).^2);
idx_arrive = find(dist_to_target < 0.2, 1, 'first');
if ~isempty(idx_arrive)
    fprintf('Robot sampai target pada t = %.2f detik (step %d)\n', waktu(idx_arrive), idx_arrive);
    X_robot = X_robot(1:idx_arrive); Y_robot = Y_robot(1:idx_arrive);
    v_robot = v_robot(1:idx_arrive); delta_robot = delta_robot(1:idx_arrive);
    waktu = waktu(1:idx_arrive);
else
    fprintf('PERINGATAN: robot TIDAK PERNAH sampai radius 0.2m dari target dalam %d detik simulasi.\n', out.tout(end));
end

figure('Name', sprintf('Uji Coba AGRIBOT - Stage %d (slip=%.2f)', STAGE, slip_factor), ...
       'Position', [100, 100, 800, 600]);
subplot(2,1,1);
plot(pathXY_raw(:,1), pathXY_raw(:,2), 'k--', 'LineWidth', 1); hold on;
plot(X_robot, Y_robot, 'b-', 'LineWidth', 2);
plot(pathXY_raw(end,1), pathXY_raw(end,2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
plot(X_robot(1), Y_robot(1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
grid on; axis equal;
title(sprintf('Trajectory Stage %d', STAGE));
xlabel('Posisi X'); ylabel('Posisi Y');
legend('Path Target', 'Jalur Robot', 'Titik Akhir', 'Start', 'Location', 'best');

subplot(2,1,2);
plot(waktu, v_robot, 'g-', 'LineWidth', 1.5); hold on;
plot(waktu, delta_robot, 'm-', 'LineWidth', 1.5);
grid on;
title('Sinyal Kendali (Gas & Setir)');
xlabel('Waktu (detik)'); ylabel('Aksi');
legend('Kecepatan (v)', 'Setir (\delta)', 'Location', 'best');