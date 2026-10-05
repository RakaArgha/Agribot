function [reward, dist_to_end] = get_reward_v6(x, y, obs, v, delta, prev_delta, dist_to_end_prev, pathXY)
% GET_REWARD_V6  Rebalancing berdasarkan DATA NYATA dari Pure Pursuit
% (yang terbukti bisa melewati tikungan: CTE maks 0.29 m, delta mentok 0.52).
%
% Masalah v5: berhenti tepat sebelum tikungan memberi reward 0 per step,
% sedangkan menikung memberi sekitar -0.34 per step. Jadi DIAM lebih
% menguntungkan daripada MENIKUNG -> agent memilih berhenti.
%
% Perubahan v6:
%  1. r_time     : penalti waktu tetap -> DIAM sekarang RUGI, tidak lagi gratis
%  2. r_cte      : -3.0 -> -1.0 (CTE 0.3 m saat menikung itu WAJAR & tak
%                  terhindarkan; PP saja kena 0.29 m, jangan dihukum berat)
%  3. r_heading  : -0.7 -> -0.3 (alasan sama: saat menikung heading memang beda)
%  4. r_progress :  5.0 -> 20.0 (progress jadi sinyal dominan)
%  5. clamp      : diperbaiki, bonus goal tidak lagi terpotong separuh

    CTE         = obs(1);
    heading_err = obs(2);

    M = size(pathXY, 1);
    dx = pathXY(:,1) - x; dy = pathXY(:,2) - y;
    [~, idx_nearest] = min(dx.^2 + dy.^2);

    % Sisa panjang jalur (arc-length), bukan garis lurus -> tidak ada insentif memotong sudut
    seg_dx = diff(pathXY(:,1));
    seg_dy = diff(pathXY(:,2));
    seg_lengths = sqrt(seg_dx.^2 + seg_dy.^2);
    if idx_nearest < M
        dist_to_end = sum(seg_lengths(idx_nearest:end));
    else
        dist_to_end = 0;
    end

    r_cte      = -1.0 * abs(CTE);
    r_heading  = -0.3 * abs(heading_err);
    r_rate     = -1.0 * (delta - prev_delta)^2;   % diturunkan dari -3.0: menikung butuh
                                                   % perubahan setir besar, jangan dihukum berat
    r_gerak    =  0.5 * v;
    r_time     = -0.2;                             % <<< KUNCI: diam sekarang RUGI

    raw_progress = dist_to_end_prev - dist_to_end;
    raw_progress = max(-1.0, min(1.0, raw_progress));
    r_progress = 20.0 * raw_progress;              % dinaikkan dari 5.0

    % Bagian shaped di-clamp supaya tidak meledak...
    shaped = r_cte + r_heading + r_rate + r_gerak + r_time + r_progress;
    shaped = max(-20, min(20, shaped));

    % ...tapi bonus goal DITAMBAHKAN SETELAH clamp, jadi tidak terpotong lagi
    r_goal = 0;
    if dist_to_end < 0.2
        r_goal = 200;
    end

    reward = shaped + r_goal;
end