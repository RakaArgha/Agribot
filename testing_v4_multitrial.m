% =========================================================================
% TESTING MULTI-TRIAL v5 - jalankan N skenario acak dalam SATU eksekusi,
% semua trajektori ditumpuk di satu grafik + ringkasan statistik.
%
% INI validasi generalisasi yang sesungguhnya (analog "validation set"
% di supervised learning): kalau agent cuma menghafal SATU trajektori,
% dia akan gagal begitu start position/heading sedikit berbeda. Kalau
% dia benar-benar belajar mengikuti path, N trial dengan gangguan kecil
% harusnya semua berhasil dengan CTE yang mirip.
% =========================================================================
clear; clc; close all;

STAGE   = 1;   % <<< SAMAKAN dengan stage yang mau diuji
N_TRIAL = 10;  % <<< jumlah skenario acak

cfg = get_curriculum_config(STAGE);
fprintf('=== MULTI-TRIAL TESTING STAGE %d: %s ===\n', STAGE, cfg.deskripsi);
fprintf('Jumlah trial: %d\n\n', N_TRIAL);

p = fullfile(pwd,'SavedAgents',cfg.agent_filename);
if isfile(p)
    load(p);
else
    [f,d] = uigetfile('*.mat','Pilih File Agen SAC');
    if isequal(f,0), return; end
    load(fullfile(d,f));
end
if ~exist('agentObj','var'), error('agentObj tidak ada di file .mat ini.'); end
agentObj.UseExplorationPolicy = false;
assignin('base','agentObj',agentObj);

mdl = "Ackermann_Env";
if ~bdIsLoaded(mdl), open_system(mdl); end

% WAJIB: matikan Fast Restart. rlSimulinkEnv() otomatis MENYALAKAN Fast
% Restart saat training (untuk mempercepat), dan itu MENEMPEL di model
% walau training sudah selesai. Fast Restart melewati inisialisasi ulang
% antar sim() berturut-turut -- akibatnya x0/y0/theta0 yang di-assignin
% tiap iterasi loop TIDAK PERNAH terbaca ulang, dan semua trial diam-diam
% menjalankan kondisi awal yang sama persis (makanya trajektori menumpuk
% jadi satu garis dan CTE tiap trial identik).
set_param(mdl, 'FastRestart', 'off');

pathXY_raw = cfg.pathXY_raw;
assignin('base','pathXY_raw',    pathXY_raw);
assignin('base','pathXY_padded', cfg.pathXY_padded);
assignin('base','dist0', cfg.dist0);
assignin('base','idx0',  cfg.idx0);
assignin('base','Ts',    cfg.Ts);

% --- kumpulan hasil ---
hasil = struct('sukses',{},'cte_maks',{},'cte_rms',{},'waktu',{}, ...
               'X',{},'Y',{},'x0',{},'y0',{},'theta0',{},'slip',{});

figure('Name',sprintf('Multi-Trial Stage %d (N=%d)',STAGE,N_TRIAL), ...
       'Position',[80 80 950 750]);
subplot(2,1,1); hold on; grid on; axis equal;
plot(pathXY_raw(:,1), pathXY_raw(:,2), 'k--', 'LineWidth', 1.5, 'DisplayName','Path Target');
plot(pathXY_raw(end,1), pathXY_raw(end,2), 'ro', 'MarkerFaceColor','r', 'DisplayName','Tujuan');
title(sprintf('Trajectory %d Trial - Stage %d', N_TRIAL, STAGE));
xlabel('X (m)'); ylabel('Y (m)');
warna = lines(N_TRIAL);

for k = 1:N_TRIAL
    % --- kondisi acak, SAMA distribusinya dengan reset_lingkungan_v4 ---
    x0     = 0.05*(2*rand()-1);
    y0     = 0.05*(2*rand()-1);
    theta0 = 0.05*(2*rand()-1);
    if cfg.slip_enabled
        slip_factor = 0.7 + 0.3*rand();
    else
        slip_factor = 1.0;
    end

    assignin('base','x0',x0); assignin('base','y0',y0); assignin('base','theta0',theta0);
    assignin('base','slip_factor',slip_factor);

    out = sim(mdl, 'StopTime', '60');

    X = out.x.Data(:); Y = out.y.Data(:);
    t = out.tout;
    d_goal = sqrt((pathXY_raw(end,1)-X).^2 + (pathXY_raw(end,2)-Y).^2);
    kk = find(d_goal < 0.3, 1, 'first');
    sukses = ~isempty(kk);
    if sukses
        X = X(1:kk); Y = Y(1:kk); t = t(1:kk);
    end

    cte = zeros(numel(X),1);
    for i = 1:numel(X)
        cte(i) = min(sqrt((pathXY_raw(:,1)-X(i)).^2 + (pathXY_raw(:,2)-Y(i)).^2));
    end

    hasil(k).sukses   = sukses;
    hasil(k).cte_maks = max(cte);
    hasil(k).cte_rms  = sqrt(mean(cte.^2));
    hasil(k).waktu    = t(end);
    hasil(k).X = X; hasil(k).Y = Y;
    hasil(k).x0 = x0; hasil(k).y0 = y0; hasil(k).theta0 = theta0;
    hasil(k).slip = slip_factor;

    if sukses
        label_trial  = sprintf('Trial %d', k);
        label_status = 'SUKSES';
    else
        label_trial  = sprintf('Trial %d (GAGAL)', k);
        label_status = 'GAGAL';
    end

    subplot(2,1,1);
    warna_garis = warna(k,:);
    if ~sukses, warna_garis = [1 0 0]; end   % gagal selalu merah
    plot(X, Y, '-', 'Color', warna_garis, 'LineWidth', 1.2, ...
         'DisplayName', label_trial);

    fprintf('Trial %2d: %-6s | CTE maks=%.3f m | CTE rms=%.3f m | t=%.1fs | x0=%+.3f y0=%+.3f th0=%+.3f | slip=%.2f\n', ...
        k, label_status, hasil(k).cte_maks, hasil(k).cte_rms, hasil(k).waktu, ...
        x0, y0, theta0, slip_factor);
end

subplot(2,1,1); legend('Location','bestoutside'); hold off;

% --- ringkasan statistik ---
sukses_arr = [hasil.sukses];
cte_maks_arr = [hasil.cte_maks];
cte_rms_arr  = [hasil.cte_rms];
waktu_arr    = [hasil.waktu];

fprintf('\n================ RINGKASAN %d TRIAL ================\n', N_TRIAL);
fprintf('Success rate     : %d/%d (%.0f%%)\n', sum(sukses_arr), N_TRIAL, 100*mean(sukses_arr));
fprintf('CTE maks  : rata-rata=%.3f m | std=%.3f m | terburuk=%.3f m\n', ...
    mean(cte_maks_arr), std(cte_maks_arr), max(cte_maks_arr));
fprintf('CTE RMS   : rata-rata=%.3f m | std=%.3f m\n', mean(cte_rms_arr), std(cte_rms_arr));
fprintf('Waktu     : rata-rata=%.2f s | std=%.2f s\n', mean(waktu_arr), std(waktu_arr));
fprintf('=====================================================\n');
if mean(sukses_arr) < 1.0
    fprintf(2, 'PERHATIAN: ada trial GAGAL. Agent belum general, jangan lanjut stage berikutnya.\n');
elseif std(cte_maks_arr) > 0.3*mean(cte_maks_arr)
    fprintf(2, 'PERHATIAN: variasi CTE antar-trial besar (std > 30%% mean). Kurang stabil.\n');
else
    fprintf('Konsisten di semua trial. Aman lanjut ke stage berikutnya.\n');
end

subplot(2,1,2);
bar(categorical(1:N_TRIAL), cte_maks_arr);
yline(mean(cte_maks_arr), 'r--', 'rata-rata');
grid on; title('CTE Maksimum per Trial'); xlabel('Trial ke-'); ylabel('CTE maks (m)');