function [reward, dist_to_end, debug] = get_reward_v9(x, y, obs, heading_err_prev, dist_to_end_prev, pathXY, idx_now)
% GET_REWARD_V9  Sama seperti v8, TAPI TIDAK mencari titik terdekat
% sendiri lagi -- pakai idx_now yang SAMA dari get_observation_v4.
% Ini memastikan reward dan observasi selalu setuju posisi "sekarang di
% titik path yang mana", menutup celah lompat-index yang sama.

    CTE         = obs(1);
    heading_err = obs(2);

    M = size(pathXY, 1);
    idxn = round(idx_now);
    if idxn < 1, idxn = 1; end
    if idxn > M, idxn = M; end

    seg_dx = diff(pathXY(:,1));
    seg_dy = diff(pathXY(:,2));
    seg_lengths = sqrt(seg_dx.^2 + seg_dy.^2);
    if idxn < M
        dist_to_end = sum(seg_lengths(idxn:end));
    else
        dist_to_end = 0;
    end

    raw_progress = dist_to_end_prev - dist_to_end;
    raw_progress = max(-1.0, min(1.0, raw_progress));
    r_progress = 20.0 * raw_progress;

    yaw_improve = abs(heading_err_prev) - abs(heading_err);
    r_yaw = 10.0 * yaw_improve;

    step_penalty = -0.01;
    r_cte_light = -0.2 * abs(CTE);

    shaped = r_progress + r_yaw + step_penalty + r_cte_light;
    shaped = max(-20, min(20, shaped));

    r_goal = 0;
    if dist_to_end < 0.2
        r_goal = 200;
    end

    reward = shaped + r_goal;
    debug = [r_progress; r_yaw; step_penalty; r_cte_light; r_goal; reward];
end