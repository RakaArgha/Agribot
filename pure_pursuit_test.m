% =========================================================================
% DIAGNOSTIK + BASELINE: Pure Pursuit di path & kinematika YANG SAMA
% Tujuan: membuktikan apakah tikungan 90 derajat ini BISA dilalui sama sekali
% oleh robot Ackermann ini -- terlepas dari soal RL/reward.
%
% Kalau Pure Pursuit BISA  -> masalahnya murni di reward/training SAC.
% Kalau Pure Pursuit GAGAL -> masalahnya di GEOMETRI PATH (sudut terlalu tajam
%                             untuk radius belok minimum robot), harus
%                             dihaluskan sudutnya, bukan diutak-atik reward.
% =========================================================================
clear; clc; close all;

L = 0.65;  max_steer = 0.52;  dt = 0.1;
R_min = L / tan(max_steer);
fprintf('Radius belok minimum robot = %.3f m\n', R_min);
fprintf('=> Simpangan minimum tak terhindarkan di sudut 90 deg = %.3f m\n\n', R_min*(sqrt(2)-1));

seg1_len = 5; seg2_len = 5; spacing = 0.1;
pathXY = generate_L_path(seg1_len, seg2_len, spacing);
M = size(pathXY,1);

% --- Parameter Pure Pursuit ---
Ld = 1.2;        % lookahead distance (m) - coba 0.8 / 1.2 / 2.0 untuk lihat efeknya
v_const = 0.5;   % kecepatan konstan (m/s)

x = 0; y = 0; theta = 0;
X_log = []; Y_log = []; D_log = []; CTE_log = [];

for k = 1:1200
    % cari titik terdekat
    d_all = sqrt((pathXY(:,1)-x).^2 + (pathXY(:,2)-y).^2);
    [~, idx_near] = min(d_all);

    % cari titik lookahead: titik pertama setelah idx_near yang jaraknya >= Ld
    idx_goal = idx_near;
    for j = idx_near:M
        if d_all(j) >= Ld
            idx_goal = j; break;
        end
        idx_goal = j;
    end
    gx = pathXY(idx_goal,1); gy = pathXY(idx_goal,2);

    % ubah ke frame lokal robot
    dx = gx - x; dy = gy - y;
    lx =  cos(-theta)*dx - sin(-theta)*dy;
    ly =  sin(-theta)*dx + cos(-theta)*dy;

    % hukum kendali pure pursuit
    Ld_actual = sqrt(lx^2 + ly^2);
    if Ld_actual < 1e-6
        delta = 0;
    else
        curvature = 2*ly / (Ld_actual^2);
        delta = atan(L * curvature);
    end
    delta = max(-max_steer, min(max_steer, delta));

    % update kinematika (sama persis dengan ackermann_kinematics_v2, slip=1)
    x = x + v_const*cos(theta)*dt;
    y = y + v_const*sin(theta)*dt;
    theta = theta + (v_const/L)*tan(delta)*dt;
    theta = atan2(sin(theta), cos(theta));

    % cross-track error terhadap titik terdekat
    d_new = sqrt((pathXY(:,1)-x).^2 + (pathXY(:,2)-y).^2);
    CTE_log(end+1) = min(d_new);

    X_log(end+1) = x; Y_log(end+1) = y; D_log(end+1) = delta;

    if sqrt((pathXY(end,1)-x)^2 + (pathXY(end,2)-y)^2) < 0.2
        fprintf('PURE PURSUIT: SAMPAI TARGET pada step %d (t=%.1f detik)\n', k, k*dt);
        break;
    end
end

if sqrt((pathXY(end,1)-x)^2 + (pathXY(end,2)-y)^2) >= 0.2
    fprintf('PURE PURSUIT: TIDAK SAMPAI target dalam %d step.\n', k);
end
fprintf('CTE maksimum   = %.3f m\n', max(CTE_log));
fprintf('CTE rata-rata  = %.3f m\n', mean(CTE_log));

figure('Name','Diagnostik Pure Pursuit','Position',[100 100 800 600]);
subplot(2,1,1);
plot(pathXY(:,1), pathXY(:,2), 'k--', 'LineWidth',1); hold on;
plot(X_log, Y_log, 'b-', 'LineWidth',2);
plot(pathXY(end,1), pathXY(end,2), 'ro','MarkerFaceColor','r','MarkerSize',8);
plot(0,0,'go','MarkerFaceColor','g','MarkerSize',8);
axis equal; grid on;
title(sprintf('Pure Pursuit (Ld=%.1fm, v=%.1f m/s) | CTE maks=%.2fm', Ld, v_const, max(CTE_log)));
xlabel('Posisi X'); ylabel('Posisi Y');
legend('Path Target','Jalur Robot','Titik Akhir','Start','Location','best');

subplot(2,1,2);
plot((1:numel(D_log))*dt, D_log, 'm-','LineWidth',1.5); hold on;
plot((1:numel(CTE_log))*dt, CTE_log, 'c-','LineWidth',1.5);
yline(max_steer,'r--'); yline(-max_steer,'r--');
grid on; title('Sudut Setir & Cross-Track Error');
xlabel('Waktu (detik)'); ylabel('rad / meter');
legend('delta (rad)','CTE (m)','batas steer','Location','best');