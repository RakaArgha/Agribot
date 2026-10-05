function [obs, idx_now] = get_observation_v4(x, y, theta, pathXY, idx_prev)
% GET_OBSERVATION_V4  SAMA seperti v3, TAPI pencarian titik terdekat
% sekarang pakai JENDELA KECIL di sekitar idx_prev (bukan scan seluruh
% path dari nol tiap step) -- meniru FindNearestPoint milik supervisor.
%
% Ini MENUTUP celah "lompat index" yang bikin robot bisa dapat reward
% gratis dengan berputar-putar di dekat sudut tikungan (index bisa
% lompat antar dua segmen path yang berdekatan secara geometris tapi
% jauh secara urutan/arc-length).
%
% idx_prev : index titik terdekat pada STEP SEBELUMNYA (dari Unit Delay,
%            initial condition = 1, di-reset ke 1 tiap episode baru)
% idx_now  : index titik terdekat SEKARANG -> WAJIB di-loop-back jadi
%            idx_prev untuk step berikutnya

    N_LOOKAHEAD = 10;
    STRIDE = 5;
    WINDOW = 30;   % cuma cari +-30 titik dari posisi terakhir (~3m di tiap arah)
    M = size(pathXY, 1);

    idx_prev = round(idx_prev);
    if idx_prev < 1,  idx_prev = 1;  end
    if idx_prev > M,  idx_prev = M;  end

    lo = max(1, idx_prev - WINDOW);
    hi = min(M, idx_prev + WINDOW);

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

    idx_next = min(idx_now + 1, M);
    seg_dx = pathXY(idx_next,1) - pathXY(idx_now,1);
    seg_dy = pathXY(idx_next,2) - pathXY(idx_now,2);
    theta_path = atan2(seg_dy, seg_dx);

    heading_err = theta - theta_path;
    heading_err = atan2(sin(heading_err), cos(heading_err));

    to_robot_x = x - pathXY(idx_now,1);
    to_robot_y = y - pathXY(idx_now,2);
    CTE = -sin(theta_path)*to_robot_x + cos(theta_path)*to_robot_y;

    rel_wp = zeros(2*N_LOOKAHEAD, 1);
    ct = cos(-theta); st = sin(-theta);
    for k = 1:N_LOOKAHEAD
        idx_k = min(idx_now + k*STRIDE, M);
        gx = pathXY(idx_k,1) - x;
        gy = pathXY(idx_k,2) - y;
        lx = ct*gx - st*gy;
        ly = st*gx + ct*gy;
        rel_wp(2*k-1) = lx;
        rel_wp(2*k)   = ly;
    end

    obs = [CTE; heading_err; dist_nearest; rel_wp];
end