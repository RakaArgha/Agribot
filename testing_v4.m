% =========================================================================
% PENGUJIAN SAC v4 - pengaturan stage diambil dari get_curriculum_config.m
% yang SAMA PERSIS dengan training_v4.m. Sekaligus hitung metrik skripsi.
% =========================================================================
clear; clc; close all;

STAGE = 4;   % <<< SAMAKAN dengan stage yang mau dites

cfg = get_curriculum_config(STAGE);

fprintf('=== TESTING STAGE %d: %s ===\n', STAGE, cfg.deskripsi);

assignin('base','pathXY_raw',    cfg.pathXY_raw);
assignin('base','pathXY_padded', cfg.pathXY_padded);
assignin('base','x0',    cfg.x0);
assignin('base','y0',    cfg.y0);
assignin('base','theta0',cfg.theta0);
assignin('base','dist0', cfg.dist0);
assignin('base','idx0',  cfg.idx0);
assignin('base','Ts',    cfg.Ts);

p = fullfile(pwd,'SavedAgents',cfg.agent_filename);
if isfile(p)
    fprintf('Agent: %s\n', cfg.agent_filename);
    load(p);
else
    fprintf('Agent default tidak ketemu, pilih manual.\n');
    [f,d] = uigetfile('*.mat','Pilih File Agen SAC');
    if isequal(f,0), disp('Dibatalkan.'); return; end
    load(fullfile(d,f));
end
if ~exist('agentObj','var'), error('agentObj tidak ada di file .mat ini.'); end
agentObj.UseExplorationPolicy = false;   % matikan noise
assignin('base','agentObj',agentObj);

mdl = "Ackermann_Env";
if ~bdIsLoaded(mdl), open_system(mdl); end
out = sim(mdl, 'StopTime', '60');
assignin('base','out',out);
disp('Simulasi selesai.');

pathXY_raw = cfg.pathXY_raw;
X = out.x.Data(:); Y = out.y.Data(:);
V = out.v.Data(:); D = out.delta.Data(:);
t = out.tout;

d_goal = sqrt((pathXY_raw(end,1)-X).^2 + (pathXY_raw(end,2)-Y).^2);
k = find(d_goal < 0.3, 1, 'first');
if ~isempty(k)
    fprintf('SUKSES: sampai tujuan di t = %.2f s (step %d)\n', t(k), k);
    X=X(1:k); Y=Y(1:k); V=V(1:k); D=D(1:k); t=t(1:k);
else
    fprintf(2,'GAGAL: tidak pernah masuk radius 0.3 m. Jarak terdekat = %.2f m\n', min(d_goal));
end

% --- metrik untuk bab perbandingan skripsi ---
cte = zeros(numel(X),1);
for i = 1:numel(X)
    cte(i) = min(sqrt((pathXY_raw(:,1)-X(i)).^2 + (pathXY_raw(:,2)-Y(i)).^2));
end
d_delta = diff(D);
fprintf('--------- METRIK STAGE %d ---------\n', STAGE);
fprintf('CTE rata-rata   : %.3f m\n', mean(cte));
fprintf('CTE maksimum    : %.3f m\n', max(cte));
fprintf('RMSE CTE        : %.3f m\n', sqrt(mean(cte.^2)));
fprintf('Waktu tempuh    : %.2f s\n', t(end));
fprintf('Kehalusan setir : %.4f rad (RMS delta-delta)\n', sqrt(mean(d_delta.^2)));
fprintf('-----------------------------------\n');

figure('Name',sprintf('AGRIBOT Stage %d',STAGE),'Position',[100 100 900 700]);
subplot(3,1,1);
plot(pathXY_raw(:,1),pathXY_raw(:,2),'k--','LineWidth',1); hold on;
plot(X,Y,'b-','LineWidth',2);
plot(pathXY_raw(end,1),pathXY_raw(end,2),'ro','MarkerFaceColor','r');
plot(X(1),Y(1),'go','MarkerFaceColor','g');
grid on; axis equal;
title(sprintf('Trajectory Stage %d - %s', STAGE, cfg.deskripsi));
xlabel('X (m)'); ylabel('Y (m)');
legend('Path Target','Jalur Robot','Tujuan','Start','Location','best');

subplot(3,1,2);
plot(t,V,'g-','LineWidth',1.5); hold on; plot(t,D,'m-','LineWidth',1.5);
grid on; title('Sinyal Kendali'); xlabel('t (s)'); ylabel('aksi');
legend('v (m/s)','\delta (rad)','Location','best');

subplot(3,1,3);
plot(t,cte,'r-','LineWidth',1.5); grid on;
title('Cross-Track Error'); xlabel('t (s)'); ylabel('CTE (m)');