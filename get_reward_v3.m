function [reward, dist_to_end] = get_reward_v3(x, y, obs, v, delta, prev_delta, dist_to_end_prev, pathXY)
% GET_REWARD_V3  Perbaikan dari v2:
%  - Penalti CTE & heading dinaikkan (agar robot benar2 balik ke jalur)
%  - "Gating": reward maju (r_gerak, r_progress) HANYA penuh kalau CTE kecil.
%    Kalau melenceng jauh, reward maju otomatis berkurang -> tidak ada lagi
%    insentif "yaudah lurus sejajar path aja".
%  - Radius selesai diperketat (0.5 -> 0.2) supaya benar2 mepet ke titik akhir.

    CTE         = obs(1);
    heading_err = obs(2);

    end_pt = pathXY(end, :);
    dist_to_end = sqrt((end_pt(1)-x)^2 + (end_pt(2)-y)^2);

    r_cte      = -5.0 * abs(CTE);         % naik dari -2.0
    r_heading  = -1.0 * abs(heading_err); % naik dari -0.5
    r_rate     = -3.0 * (delta - prev_delta)^2;

    % Gating: align_factor = 1 kalau tepat di jalur, turun ke 0 kalau CTE >= 1 meter
    align_factor = max(0, 1 - abs(CTE)/1.0);

    r_gerak    = 0.5 * v * align_factor;
    r_progress = 5.0 * (dist_to_end_prev - dist_to_end) * align_factor;

    r_goal = 0;
    if dist_to_end < 0.2          % diperketat dari 0.5
        r_goal = 300;             % dinaikkan sepadan, supaya presisi tetap dihargai besar
    end

    reward = r_cte + r_heading + r_rate + r_gerak + r_progress + r_goal;
end