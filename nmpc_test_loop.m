% =========================================================================
% NMPC BASELINE - Loop simulasi native MATLAB (tanpa Simulink)
% Jalankan nmpc_baseline_setup.m dulu SEKALI sebelum ini.
% =========================================================================
clear; clc; close all;
load('nlobj_baseline.mat', 'nlobj', 'L');

seg1_len = 5; seg2_len = 5; spacing = 0.1;
pathXY = generate_L_path(seg1_len, seg2_len, spacing);
M = size(pathXY, 1);

x  = [0; 0; 0];   % state awal: x, y, theta
u  = [0.3; 0];    % input awal
Ts = 0.1;
Tsim = 30;
nSteps = round(Tsim/Ts);

X_log = zeros(nSteps,1); Y_log = zeros(nSteps,1);
V_log = zeros(nSteps,1); Delta_log = zeros(nSteps,1);
Hp = nlobj.PredictionHorizon;

for k = 1:nSteps
    % cari titik path terdekat
    dx = pathXY(:,1) - x(1); dy = pathXY(:,2) - x(2);
    [~, idx] = min(dx.^2 + dy.^2);

    % ambil Hp titik ke depan sebagai referensi horizon NMPC
    yref = zeros(Hp, 2);
    for h = 1:Hp
        idx_h = min(idx + h, M);
        yref(h,:) = pathXY(idx_h, :);
    end

    [u, ~, ~] = nlmpcmove(nlobj, x, u, yref);

    % update state pakai model IDEAL (tanpa slip -> ini baseline)
    dx1 = u(1)*cos(x(3)); dy1 = u(1)*sin(x(3)); dth1 = (u(1)/L)*tan(u(2));
    x = x + Ts*[dx1; dy1; dth1];
    x(3) = atan2(sin(x(3)), cos(x(3)));

    X_log(k) = x(1); Y_log(k) = x(2);
    V_log(k) = u(1); Delta_log(k) = u(2);

    if norm([pathXY(end,1)-x(1), pathXY(end,2)-x(2)]) < 0.5
        X_log = X_log(1:k); Y_log = Y_log(1:k);
        V_log = V_log(1:k); Delta_log = Delta_log(1:k);
        break;
    end
end

figure('Name','NMPC Baseline');
subplot(2,1,1);
plot(pathXY(:,1), pathXY(:,2), 'k--'); hold on;
plot(X_log, Y_log, 'b-', 'LineWidth', 2);
axis equal; grid on; title('NMPC Baseline - Trajectory');
xlabel('Posisi X'); ylabel('Posisi Y');

subplot(2,1,2);
plot(V_log, 'g', 'LineWidth', 1.5); hold on;
plot(Delta_log, 'm', 'LineWidth', 1.5);
legend('v','delta'); grid on; title('NMPC Control Signal');
xlabel('Step'); ylabel('Aksi');