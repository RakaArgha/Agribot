function [obs, idx_now] = get_observation_v5(x, y, theta, pathXY, idx_prev)
% GET_OBSERVATION_V5  Perbaikan dari v4: jendela pencarian DIPERKETAT
% drastis dan HANYA BOLEH MAJU (nyaris tidak boleh mundur).
%
% v4 pakai jendela +-30 -> MASIH bisa "menembus" lipatan sudut 90 derajat,
% karena titik di segmen SETELAH belok bisa masuk jendela +-30 dari titik
% SEBELUM belok (secara index dekat, walau secara jarak-tempuh jauh beda).
%
% v5: robot maksimal maju 0.1m/step (v_max x dt), jarak antar titik path
% 0.1m -> index REALISTISNYA cuma maju 1-2 per step. Jendela WINDOW_FWD=5
% (longgar dikit) dan WINDOW_BACK=2 (jaga-jaga overshoot kecil) menutup
% celah lompat index secara matematis -- tidak mungkin lagi "menembus"
% ke segmen lain yang jauh secara arc-length walau dekat secara Euclidean.

    N_LOOKAHEAD = 10;
    STRIDE = 5;
    WINDOW_BACK = 2;
    WINDOW_FWD  = 5;
    M = size(pathXY, 1);

    idx_prev = round(idx_prev);
    if idx_prev < 1,  idx_prev = 1;  end
    if idx_prev > M,  idx_prev = M;  end

    lo = max(1, idx_prev - WINDOW_BACK);
    hi = min(M, idx_prev + WINDOW_FWD);

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