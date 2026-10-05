function [obs, idx_now] = get_observation_v6(x, y, theta, pathXY, idx_prev)
%#codegen
% GET_OBSERVATION_V6
%
% PERBEDAAN UTAMA DARI V5:
%   1. Index TIDAK BOLEH merayap melewati ujung path ASLI (N), bukan ujung
%      array padding (300). Ini yang dulu bikin dist_to_end jadi 0 permanen.
%   2. Observasi dinormalisasi supaya skalanya seragam (CTE ~1, waypoint ~5m
%      dibagi 5). SAC jauh lebih stabil dengan input yang sebanding.
%
% Ukuran port TIDAK BERUBAH: 5 input, 2 output, obs tetap 23x1.
% Jadi TIDAK PERLU mengubah wiring Simulink sama sekali.

    N_LOOKAHEAD = 10;
    STRIDE      = 5;
    WINDOW_BACK = 2;
    WINDOW_FWD  = 5;
    SKALA_WP    = 5.0;    % waypoint lookahead dibagi ini

    N = panjang_path_asli(pathXY);

    idx_prev = round(idx_prev);
    if idx_prev < 1, idx_prev = 1; end
    if idx_prev > N, idx_prev = N; end

    lo = max(1, idx_prev - WINDOW_BACK);
    hi = min(N, idx_prev + WINDOW_FWD);

    best_d = inf;
    best_i = idx_prev;
    for i = lo:hi
        d = (pathXY(i,1)-x)^2 + (pathXY(i,2)-y)^2;
        if d < best_d
            best_d = d;
            best_i = i;
        end
    end
    idx_now = best_i;
    dist_nearest = sqrt(best_d);

    % arah path di titik idx_now (pakai segmen mundur kalau sudah di ujung,
    % supaya tidak pernah atan2(0,0) di daerah padding)
    if idx_now < N
        seg_dx = pathXY(idx_now+1,1) - pathXY(idx_now,1);
        seg_dy = pathXY(idx_now+1,2) - pathXY(idx_now,2);
    elseif N > 1
        seg_dx = pathXY(N,1) - pathXY(N-1,1);
        seg_dy = pathXY(N,2) - pathXY(N-1,2);
    else
        seg_dx = 1; seg_dy = 0;
    end
    theta_path = atan2(seg_dy, seg_dx);

    heading_err = theta - theta_path;
    heading_err = atan2(sin(heading_err), cos(heading_err));

    to_robot_x = x - pathXY(idx_now,1);
    to_robot_y = y - pathXY(idx_now,2);
    CTE = -sin(theta_path)*to_robot_x + cos(theta_path)*to_robot_y;

    rel_wp = zeros(2*N_LOOKAHEAD, 1);
    ct = cos(-theta); st = sin(-theta);
    for k = 1:N_LOOKAHEAD
        idx_k = min(idx_now + k*STRIDE, N);       % dibatasi N, bukan M
        gx = pathXY(idx_k,1) - x;
        gy = pathXY(idx_k,2) - y;
        lx = ct*gx - st*gy;
        ly = st*gx + ct*gy;
        rel_wp(2*k-1) = lx / SKALA_WP;
        rel_wp(2*k)   = ly / SKALA_WP;
    end

    % CATATAN untuk get_reward_v11:
    %   obs(1) = CTE dalam METER (tidak dinormalisasi)
    %   obs(2) = heading_err dibagi pi  -> rentang [-1, 1]
    %   obs(3) = jarak ke titik path terdekat dalam METER
    obs = [CTE; heading_err/pi; dist_nearest; rel_wp];
end

% -------------------------------------------------------------------------
function N = panjang_path_asli(pathXY)
%#codegen
% Mengembalikan index titik terakhir path ASLI. Padding dibuat dengan cara
% mengulang titik terakhir (lihat pad_path.m), jadi titik-titik padding
% jaraknya nol dari titik sebelumnya.
    M = size(pathXY,1);
    N = 1;
    for i = M:-1:2
        dx = pathXY(i,1) - pathXY(i-1,1);
        dy = pathXY(i,2) - pathXY(i-1,2);
        if (dx*dx + dy*dy) > 1e-12
            N = i;
            return;
        end
    end
end