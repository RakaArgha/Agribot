function [reward, dist_to_end, debug] = get_reward_v11(x, y, obs, heading_err_prev, dist_to_end_prev, pathXY, idx_now)
%#codegen
% GET_REWARD_V11
%
% PERUBAHAN PALING PENTING vs v9/v10:
%   r_goal TIDAK LAGI memakai dist_to_end (panjang busur dari index).
%   r_goal memakai JARAK EUCLIDEAN ASLI dari (x,y) ke titik akhir path asli
%   -- rumus yang PERSIS SAMA dengan check_isdone_v4. Jadi secara matematis
%   mustahil r_goal menyala tanpa episode ikut berhenti.
%
%   Konsekuensinya: batas atas reward satu episode sekarang terhitung.
%   400 step x 10 (clamp) + 300 (goal) = 4300. Kalau kamu masih melihat
%   angka 10.000+, berarti ada masalah LAIN dan itu langsung ketahuan.
%
% Ukuran port TIDAK BERUBAH: 7 input, 3 output. debug melebar dari 6 ke 8
% elemen (Simulink menyesuaikan sendiri, tidak perlu rewiring).

    % --- baca observasi (skala harus cocok dengan get_observation_v6) ---
    CTE          = obs(1);          % meter
    heading_err  = obs(2);          % sudah dibagi pi, rentang [-1,1]
    dist_nearest = obs(3);          % meter

    N = panjang_path_asli(pathXY);

    idxn = round(idx_now);
    if idxn < 1, idxn = 1; end
    if idxn > N, idxn = N; end

    % --- panjang busur sisa (HANYA untuk progress shaping, BUKAN untuk goal) ---
    dist_to_end = 0;
    for i = idxn : N-1
        dx = pathXY(i+1,1) - pathXY(i,1);
        dy = pathXY(i+1,2) - pathXY(i,2);
        dist_to_end = dist_to_end + sqrt(dx*dx + dy*dy);
    end

    % --- progress: robot maks 0.1 m/step, jadi clamp 0.5 sudah sangat longgar ---
    raw_progress = dist_to_end_prev - dist_to_end;
    raw_progress = max(-0.5, min(0.5, raw_progress));
    r_progress = 30.0 * raw_progress;

    % --- yaw improvement (dua-duanya dalam satuan ternormalisasi) ---
    yaw_improve = abs(heading_err_prev) - abs(heading_err);
    yaw_improve = max(-0.2, min(0.2, yaw_improve));
    r_yaw = 15.0 * yaw_improve;

    % --- jaga tetap dekat path ---
    r_cte = -6.0 * abs(CTE);

    step_penalty = -0.05;

    shaped = r_progress + r_yaw + r_cte + step_penalty;
    shaped = max(-25, min(25, shaped));

    % --- GOAL: pakai jarak asli, identik dengan check_isdone_v4 ---
    p  = robot_params();
    Ng = max(1, N - p.n_ghost);        % tujuan ASLI, bukan ujung hantu
    dx_g = x - pathXY(Ng,1);
    dy_g = y - pathXY(Ng,2);
    d_goal = sqrt(dx_g*dx_g + dy_g*dy_g);

    r_goal = 0;
    if d_goal < 0.3
        r_goal = 300;
    end

    % --- KELUAR JALUR: identik dengan check_isdone_v4 ---
    r_off = 0;
    if dist_nearest > 0.5
        r_off = -100;
    end

    reward = shaped + r_goal + r_off;

    debug = [r_progress; r_yaw; r_cte; r_goal; r_off; reward; double(idxn); d_goal];
end

% -------------------------------------------------------------------------
function N = panjang_path_asli(pathXY)
%#codegen
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