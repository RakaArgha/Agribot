function [reward, dist_to_end] = get_reward_v2(x, y, obs, v, delta, prev_delta, dist_to_end_prev, pathXY)
% GET_REWARD_V2  Reward untuk path-following dengan progress shaping.
%
% obs               : [CTE; heading_err; dist_nearest; rel_wp...] dari get_observation_v2
% dist_to_end_prev  : jarak ke titik akhir path pada STEP SEBELUMNYA
%                     -> di Simulink, ini didapat dari Unit Delay (1/z) yang
%                        me-loop-back output kedua fungsi ini (dist_to_end)
%                     -> inisialisasi Unit Delay = jarak awal robot ke ujung path
% pathXY            : matriks waypoint path (titik terakhir = target akhir)
%
% Output kedua (dist_to_end) WAJIB di-loop-back ke input dist_to_end_prev
% pada step berikutnya (lihat instruksi wiring).

    CTE         = obs(1);
    heading_err = obs(2);

    end_pt = pathXY(end, :);
    dist_to_end = sqrt((end_pt(1)-x)^2 + (end_pt(2)-y)^2);

    r_cte      = -2.0 * abs(CTE);
    r_heading  = -0.5 * abs(heading_err);
    r_rate     = -3.0 * (delta - prev_delta)^2;   % dinaikkan sedikit utk kurangi jitter
    r_gerak    =  0.5 * v;

    % Progress reward: reward positif kalau makin dekat ke ujung path dibanding step lalu
    r_progress = 5.0 * (dist_to_end_prev - dist_to_end);

    r_goal = 0;
    if dist_to_end < 0.5
        r_goal = 200;
    end

    reward = r_cte + r_heading + r_rate + r_gerak + r_progress + r_goal;
end