function [reward, dist_to_end] = get_reward_v4(x, y, obs, v, delta, prev_delta, dist_to_end_prev, pathXY)
% GET_REWARD_V4  Perbaikan dari v3: MENCABUT mekanisme "gating" (align_factor)
% yang menyebabkan reward hacking & robot "membeku" saat kesasar.
% Kembali ke struktur v2 yang terbukti stabil, cuma menaikkan bobot CTE
% secara MODERAT (tidak seagresif v3), plus safety-clamp supaya tidak ada
% lagi lonjakan reward tak wajar.

    CTE         = obs(1);
    heading_err = obs(2);

    end_pt = pathXY(end, :);
    dist_to_end = sqrt((end_pt(1)-x)^2 + (end_pt(2)-y)^2);

    r_cte      = -3.0 * abs(CTE);          % v1=-2.0, v3=-5.0(terlalu agresif) -> v4=-3.0
    r_heading  = -0.7 * abs(heading_err);  % v1=-0.5, v3=-1.0 -> v4=-0.7
    r_rate     = -3.0 * (delta - prev_delta)^2;
    r_gerak    =  0.5 * v;                 % TIDAK di-gating lagi

    % Progress reward, DI-CLAMP supaya tidak bisa melonjak ekstrem walau
    % ada anomali sesaat (misal reset state yang kurang mulus)
    raw_progress = dist_to_end_prev - dist_to_end;
    raw_progress = max(-1.0, min(1.0, raw_progress));  % clamp ke [-1,1] meter/step
    r_progress = 5.0 * raw_progress;

    r_goal = 0;
    if dist_to_end < 0.2
        r_goal = 100;   % dikembalikan moderat (v1=100, v3=300 terlalu besar & memicu hacking)
    end

    reward = r_cte + r_heading + r_rate + r_gerak + r_progress + r_goal;

    % Safety clamp akhir: cegah reward tunggal meledak walau ada bug lain
    reward = max(-50, min(50, reward));
end