% =========================================================================
% CEK_KINEMATIKA_DUAL  Uji open-loop dual_ackermann_kinematics_v3.m
% TANPA Simulink, TANPA RL agent. Tujuan: pastikan PLANT-nya sendiri benar
% sebelum menyalahkan reward/training atas osilasi yang terlihat di
% testing_v4.m.
%
% CARA BACA HASIL:
%   Dengan delta_f tetap dan delta_r = -delta_f (mode simetris), robot
%   HARUS bergerak melingkar sempurna dengan radius = R_min_dual dari
%   robot_params(). Kalau radius yang terukur meleset jauh dari itu,
%   BUG ADA DI dual_ackermann_kinematics_v3.m, bukan di RL.
% =========================================================================
clear; clc; close all;

p = robot_params();
Ts = 0.1;
T_total = 15;   % detik
N = round(T_total/Ts);

v = 0.5;                    % kecepatan tetap, m/s
delta_f = -0.3;               % sudut tetap, rad (~17 deg, jauh dari limit)
delta_r = -delta_f;          % mode SIMETRIS

x = 0; y = 0; theta = 0;
X = zeros(N,1); Y = zeros(N,1); TH = zeros(N,1);

for k = 1:N
    [x, y, theta] = dual_ackermann_kinematics_v4(v, delta_f, delta_r, x, y, theta);
    X(k) = x; Y(k) = y; TH(k) = theta;
end

% --- radius seharusnya, dari geometri (BUKAN dari simulasi) ---
R_teori = abs(p.L / (2*tan(delta_f)));

% --- radius terukur dari lintasan: fit lingkaran sederhana ---
% pusat lingkaran = titik yang berjarak sama ke titik awal & titik akhir,
% tegak lurus arah gerak awal (metode cepat, cukup untuk sanity check)
cx = 0; cy = R_teori * sign(delta_f);   % prediksi pusat kalau plant benar
R_terukur = mean(sqrt((X-cx).^2 + (Y-cy).^2));

fprintf('=== CEK KINEMATIKA DUAL ACKERMANN (open-loop) ===\n');
fprintf('delta_f = %.3f rad (%.1f deg), delta_r = %.3f rad\n', delta_f, rad2deg(delta_f), delta_r);
fprintf('R_min_dual dari robot_params() = %.4f m\n', p.R_min_dual);
fprintf('R_teori untuk delta_f ini      = %.4f m\n', R_teori);
fprintf('R_terukur dari simulasi        = %.4f m\n', R_terukur);
fprintf('Selisih                        = %.4f m (%.1f%%)\n', ...
    abs(R_terukur-R_teori), 100*abs(R_terukur-R_teori)/R_teori);

if abs(R_terukur-R_teori)/R_teori > 0.05
    fprintf(2, '>>> BUG: radius meleset >5%%. Cek ulang dual_ackermann_kinematics_v3.m\n');
else
    fprintf('>>> PLANT BENAR. Radius sesuai teori. Masalah osilasi BUKAN dari kinematika.\n');
end

figure('Name','Cek Kinematika Dual Ackermann','Position',[100 100 700 700]);
plot(X, Y, 'b.-', 'LineWidth', 1.5); hold on; axis equal; grid on;
viscircles([cx cy], R_teori, 'Color','r','LineStyle','--');
plot(cx, cy, 'r+', 'MarkerSize', 12);
title(sprintf('Lintasan seharusnya lingkaran R=%.3f m', R_teori));
xlabel('X (m)'); ylabel('Y (m)');
legend('Lintasan robot','Lingkaran teori','Pusat teori','Location','best');