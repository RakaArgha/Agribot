function [reward, dist_to_end, debug] = get_reward_v8(x, y, obs, heading_err_prev, dist_to_end_prev, pathXY)
% GET_REWARD_V8  Redesign berdasarkan reward Function ASLI milik supervisor.
%
% PERUBAHAN UTAMA dari v7:
%  1. r_heading (penalti ABSOLUT) DIHAPUS, diganti r_yaw (reward IMPROVEMENT).
%     Robot sekarang diberi reward kalau heading error MENGECIL dibanding
%     step lalu -- walau nilai absolutnya masih besar (lagi menikung tajam).
%     Ini pola PERSIS dari reward Function supervisor (yaw_improve).
%  2. r_rate DIHAPUS TOTAL dari reward. Kehalusan setir sekarang jadi
%     tanggung jawab Rate Limiter block di Simulink (structural), BUKAN
%     reward -- persis seperti desain supervisor (mereka juga tidak punya
%     suku rate-penalty di reward, tapi punya blok Rate Limiter di diagram).
%  3. step_penalty dikecilkan drastis: -0.2 -> -0.01 (sama seperti supervisor).
%  4. r_gerak (bonus kecepatan langsung) DIHAPUS -- progress reward saja
%     sudah cukup memberi insentif bergerak (gerak maju = progress positif).
%  5. r_cte dikecilkan jadi cuma "penjaga ringan", bukan komponen utama.
%
% Input BARU: heading_err_prev (dari Unit Delay baru di Simulink, men-delay
% obs(2) satu step -- lihat instruksi wiring).

    CTE         = obs(1);
    heading_err = obs(2);

    M = size(pathXY, 1);
    dx = pathXY(:,1) - x; dy = pathXY(:,2) - y;
    [~, idx_nearest] = min(dx.^2 + dy.^2);

    seg_dx = diff(pathXY(:,1));
    seg_dy = diff(pathXY(:,2));
    seg_lengths = sqrt(seg_dx.^2 + seg_dy.^2);
    if idx_nearest < M
        dist_to_end = sum(seg_lengths(idx_nearest:end));
    else
        dist_to_end = 0;
    end

    % --- Progress reward (arc-length improvement) - TETAP dari v6/v7 ---
    raw_progress = dist_to_end_prev - dist_to_end;
    raw_progress = max(-1.0, min(1.0, raw_progress));
    r_progress = 20.0 * raw_progress;

    % --- BARU: Yaw IMPROVEMENT (pola dari reward Function supervisor) ---
    yaw_improve = abs(heading_err_prev) - abs(heading_err);
    r_yaw = 10.0 * yaw_improve;

    % --- Step penalty kecil (samakan skala dengan supervisor: -0.01) ---
    step_penalty = -0.01;

    % --- CTE cuma penjaga ringan, bukan komponen dominan lagi ---
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