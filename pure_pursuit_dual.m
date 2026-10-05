% =========================================================================
% PURE_PURSUIT_DUAL  Baseline geometris untuk dual Ackermann.
%
% KENAPA INI PENTING
%   Pure Pursuit tidak belajar apa-apa -- murni geometri. Jadi CTE yang
%   dihasilkannya adalah ukuran "seberapa sulit path ini bagi robot dengan
%   kinematika ini", terlepas dari kualitas RL.
%
%   Kalau SAC menghasilkan CTE ~sama dengan Pure Pursuit -> SAC sudah
%   mendekati batas fisik, tidak ada gunanya menaikkan w_cte lagi.
%   Kalau Pure Pursuit JAUH lebih kecil -> SAC masih punya ruang perbaikan.
%
% RUMUS UNTUK DUAL ACKERMANN (beda dari single!)
%   Single: delta = atan(L*2*sin(alpha)/Ld)
%   Dual  : robot berputar terhadap ICR di TENGAH bodi, radius belok
%           R = L/(2*tan(delta)). Dari geometri pure pursuit R = Ld/(2*sin(alpha)),
%           sehingga:
%               L/(2*tan(delta)) = Ld/(2*sin(alpha))
%               delta = atan( L*sin(alpha) / Ld )
%           (tanpa faktor 2 yang ada di versi single -- inilah yang membuat
%            dual Ackermann butuh sudut lebih kecil untuk tikungan yang sama)
% =========================================================================
clear; clc; close all;

STAGE = 4;          % <<< stage mana yang mau diuji
Ld    = 0.8;        % lookahead distance (m) -- parameter utama Pure Pursuit
v_const = 0.85;     % kecepatan tetap, disamakan dengan v rata-rata SAC

cfg = get_curriculum_config(STAGE);
p   = robot_params();
path = cfg.pathXY_raw;
Ts   = cfg.Ts;

fprintf('=== PURE PURSUIT (dual Ackermann) STAGE %d ===\n', STAGE);
fprintf('%s | Ld = %.2f m | v = %.2f m/s\n', cfg.deskripsi, Ld, v_const);

x = 0; y = 0; theta = 0;
N_MAX = 2000;
X = zeros(N_MAX,1); Y = zeros(N_MAX,1); D = zeros(N_MAX,1);
n = 0;

for k = 1:N_MAX
    % --- cari titik lookahead: titik path pertama yang jaraknya > Ld ---
    d_all = sqrt((path(:,1)-x).^2 + (path(:,2)-y).^2);
    [~, idx_near] = min(d_all);
    idx_goal = idx_near;
    while idx_goal < size(path,1) && d_all(idx_goal) < Ld
        idx_goal = idx_goal + 1;
    end
    gx = path(idx_goal,1); gy = path(idx_goal,2);

    % --- alpha: sudut ke titik lookahead relatif heading robot ---
    alpha = atan2(gy - y, gx - x) - theta;
    alpha = atan2(sin(alpha), cos(alpha));

    Ld_act = sqrt((gx-x)^2 + (gy-y)^2);
    if Ld_act < 1e-6, Ld_act = 1e-6; end

    % --- RUMUS DUAL ACKERMANN (lihat penjelasan di header) ---
    delta = atan( p.L * sin(alpha) / Ld_act );
    delta = max(-p.delta_max, min(p.delta_max, delta));

    % --- majukan robot pakai kinematika yang sama dengan Simulink ---
    [x, y, theta] = dual_ackermann_kinematics_v4(v_const, delta, -delta, x, y, theta);

    n = n + 1;
    X(n)=x; Y(n)=y; D(n)=delta;

    % --- berhenti kalau sampai tujuan ---
    if sqrt((x-path(end,1))^2 + (y-path(end,2))^2) < 0.3
        fprintf('SUKSES: sampai tujuan pada t = %.2f s (%d step)\n', n*Ts, n);
        break;
    end
end
X=X(1:n); Y=Y(1:n); D=D(1:n); t=(1:n)'*Ts;
if sqrt((X(end)-path(end,1))^2 + (Y(end)-path(end,2))^2) >= 0.3
    fprintf(2,'GAGAL sampai tujuan dalam %d step.\n', n);
end

% --- metrik, format sama dengan testing_v4.m supaya bisa dibandingkan ---
cte = zeros(n,1);
for i = 1:n
    cte(i) = min(sqrt((path(:,1)-X(i)).^2 + (path(:,2)-Y(i)).^2));
end
d_delta = diff(D);
fprintf('--------- METRIK PURE PURSUIT STAGE %d ---------\n', STAGE);
fprintf('CTE rata-rata   : %.3f m\n', mean(cte));
fprintf('CTE maksimum    : %.3f m\n', max(cte));
fprintf('RMSE CTE        : %.3f m\n', sqrt(mean(cte.^2)));
fprintf('Waktu tempuh    : %.2f s\n', t(end));
fprintf('Kehalusan setir : %.4f rad (RMS delta-delta)\n', sqrt(mean(d_delta.^2)));
fprintf('delta maksimum  : %.3f rad (batas %.3f)\n', max(abs(D)), p.delta_max);
fprintf('-----------------------------------------------\n');

figure('Name',sprintf('Pure Pursuit Stage %d',STAGE),'Position',[100 100 900 700]);
subplot(3,1,1);
plot(path(:,1),path(:,2),'k--','LineWidth',1); hold on;
plot(X,Y,'b-','LineWidth',2);
plot(path(end,1),path(end,2),'ro','MarkerFaceColor','r');
plot(X(1),Y(1),'go','MarkerFaceColor','g');
grid on; axis equal;
title(sprintf('Pure Pursuit - Stage %d (%s)', STAGE, cfg.deskripsi));
xlabel('X (m)'); ylabel('Y (m)');
legend('Path Target','Jalur Robot','Tujuan','Start','Location','best');

subplot(3,1,2);
plot(t,D,'m-','LineWidth',1.5); grid on;
yline(p.delta_max,'r--'); yline(-p.delta_max,'r--');
title('Sudut Kemudi'); xlabel('t (s)'); ylabel('\delta (rad)');

subplot(3,1,3);
plot(t,cte,'r-','LineWidth',1.5); grid on;
title('Cross-Track Error'); xlabel('t (s)'); ylabel('CTE (m)');