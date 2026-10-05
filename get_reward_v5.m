function [reward, dist_to_end] = get_reward_v5(x, y, obs, v, delta, prev_delta, dist_to_end_prev, pathXY)
% GET_REWARD_V5  SATU perubahan dari v4: dist_to_end sekarang dihitung
% sebagai SISA PANJANG JALUR (arc-length dari titik terdekat sampai akhir),
% BUKAN garis lurus ke titik akhir. Ini menghilangkan insentif "memotong
% sudut" yang terdeteksi di v4 (robot dapat reward progress walau menyimpang
% dari jalur, asal posisinya makin dekat garis-lurus ke target akhir).
%
% Sengaja HANYA satu perubahan ini dulu (tidak ditumpuk dengan heuristik
% lain) supaya efeknya bisa dievaluasi dengan jelas sebelum nambah lagi.

    CTE         = obs(1);
    heading_err = obs(2);

    M = size(pathXY, 1);
    dx = pathXY(:,1) - x; dy = pathXY(:,2) - y;
    [~, idx_nearest] = min(dx.^2 + dy.^2);

    % --- INI PERUBAHANNYA: arc-length tersisa, bukan garis lurus ---
    seg_dx = diff(pathXY(:,1));
    seg_dy = diff(pathXY(:,2));
    seg_lengths = sqrt(seg_dx.^2 + seg_dy.^2);
    if idx_nearest < M
        dist_to_end = sum(seg_lengths(idx_nearest:end));
    else
        dist_to_end = 0;
    end
    % -----------------------------------------------------------

    r_cte      = -3.0 * abs(CTE);
    r_heading  = -0.7 * abs(heading_err);
    r_rate     = -3.0 * (delta - prev_delta)^2;
    r_gerak    =  0.5 * v;

    raw_progress = dist_to_end_prev - dist_to_end;
    raw_progress = max(-1.0, min(1.0, raw_progress));
    r_progress = 5.0 * raw_progress;

    r_goal = 0;
    if dist_to_end < 0.2
        r_goal = 100;
    end

    reward = r_cte + r_heading + r_rate + r_gerak + r_progress + r_goal;
    reward = max(-50, min(50, reward));
end