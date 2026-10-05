% =========================================================================
% EKSPOR_OBS_REF  Kunci jawaban untuk porting get_observation_v6 ke Python.
%
% Robot "virtual" berjalan menyusuri path dengan gangguan acak, dan di tiap
% langkah get_observation_v6 dipanggil. Python nanti harus menghasilkan
% obs dan idx_now yang SAMA untuk pose yang sama.
%
% OUTPUT (deploy/):
%   path_padded.csv  -> path yang dipakai agent (termasuk titik hantu & padding)
%   obs_ref.csv      -> tiap baris: x, y, theta, idx_prev, obs(1..23), idx_now
% =========================================================================
clear; clc;

STAGE = 6;   % <<< sama dengan agent yang dipakai

cfg = get_curriculum_config(STAGE);
P   = cfg.pathXY_padded;
Nr  = size(cfg.pathXY_raw, 1);          % jumlah titik path asli
if ~exist('deploy','dir'), mkdir('deploy'); end
writematrix(P, fullfile('deploy','path_padded.csv'));

rng(7);
N = Nr + 30;                             % sedikit lewat ujung path asli
R = zeros(N, 4 + 23 + 1);
idx_prev = cfg.idx0;

for k = 1:N
    s  = min(k, Nr);                                     % maju 1 titik/langkah
    s2 = min(s+1, size(P,1));
    arah = atan2(P(s2,2)-P(s,2), P(s2,1)-P(s,1));
    lat  = 0.3*(2*rand()-1);                             % geser samping +-0.3 m
    x  = P(s,1) - sin(arah)*lat;
    y  = P(s,2) + cos(arah)*lat;
    th = arah + 0.2*(2*rand()-1);                        % heading +-0.2 rad

    [obs, idx_now] = get_observation_v6(x, y, th, P, idx_prev);
    R(k,:) = [x, y, th, idx_prev, obs(:)', idx_now];
    idx_prev = idx_now;
end

writematrix(R, fullfile('deploy','obs_ref.csv'));
fprintf('>> path_padded.csv (%d titik) & obs_ref.csv (%d langkah) tersimpan\n', size(P,1), N);
fprintf('   titik path asli = %d, idx_now terakhir = %d\n', Nr, idx_now);