function cfg = get_curriculum_config(stage)
% GET_CURRICULUM_CONFIG  Satu-satunya sumber kebenaran pengaturan stage.
%
% PERUBAHAN BESAR vs versi lama:
%   - Tidak lagi mengembalikan seg1_len/seg2_len (path siku 90 derajat).
%     Sudut siku punya radius 0 m, sedangkan radius belok minimum robot
%     1.14 m -> path itu memang MUSTAHIL diikuti. Sekarang kurikulumnya
%     berbasis RADIUS TIKUNGAN yang mengecil bertahap.
%   - Mengembalikan satu struct, bukan 6 nilai terpisah. Menambah parameter
%     baru nanti tidak akan merusak urutan argumen di training/testing.
%   - Path-nya langsung dibangun di sini, jadi training dan testing
%     MUSTAHIL memakai path yang berbeda.
%
% Cara pakai:
%   cfg = get_curriculum_config(2);
%   cfg.pathXY_raw, cfg.agent_filename, ...

% Radius kurikulum sekarang diturunkan dari R_min_dual robot (dual
% Ackermann simetris, delta_max=31deg -> R_min = 0.585 m), BUKAN dari
% angka R_min_single (1.14 m) yang lama. Dengan dua roda kemudi, robot
% mampu menikung jauh lebih tajam.
%
% SLIP DIHAPUS TOTAL dari simulasi ini. Semua stage berjalan dengan
% traksi ideal. Pemodelan slip dipindah ke simulasi 3D nanti.

p_robot = robot_params();
Rmin    = p_robot.R_min_dual;   % ~2.245 m

switch stage
    case 1
        % Lurus 15 m. Agent belajar "maju itu bagus" + jaga heading.
        cfg.deskripsi           = 'Lurus 15 m';
        cfg.pathXY_raw          = [(0:0.1:15)', zeros(151,1)];
        cfg.agent_filename      = 'Agen_SAC_v5_stage1.mat';
        cfg.prev_agent_filename = '';

    case 2
        % Tikungan landai, ~3x Rmin -> jauh dari batas kemampuan.
        R = 3 * Rmin;   % ~6.74 m
        cfg.deskripsi           = sprintf('L halus, R = %.2f m (landai)', R);
        cfg.pathXY_raw          = generate_L_path_smooth(10, 10, R, 0.1);
        cfg.agent_filename      = 'Agen_SAC_v5_stage2.mat';
        cfg.prev_agent_filename = 'Agen_SAC_v5_stage1.mat';

    case 3
        % Tikungan sedang, ~2x Rmin.
        R = 2 * Rmin;   % ~4.49 m
        cfg.deskripsi           = sprintf('L halus, R = %.2f m', R);
        cfg.pathXY_raw          = generate_L_path_smooth(10, 10, R, 0.1);
        cfg.agent_filename      = 'Agen_SAC_v5_stage3.mat';
        cfg.prev_agent_filename = 'Agen_SAC_v5_stage2.mat';

    case 4
        % Target akhir: ~1.5x Rmin. Ini yang harus
        % mulus dulu sebelum lanjut ke Stage 5.
        R = 1.5 * Rmin;   % ~3.37 m
        cfg.deskripsi           = sprintf('L halus, R = %.2f m (target bersih)', R);
        cfg.pathXY_raw          = generate_L_path_smooth(10, 10, R, 0.1);
        cfg.agent_filename      = 'Agen_SAC_v5_stage4.mat';
        cfg.prev_agent_filename = 'Agen_SAC_v5_stage3.mat';
    case 5
        % Uji batas: ~1.2x Rmin. Roda dalam ~24.7 deg (batas mekanik 31.2),
        % sudut virtual ~17.7 deg (85% dari batas 20.96 deg).
        R = 1.2 * Rmin;   % ~2.69 m
        cfg.deskripsi           = sprintf('L halus, R = %.2f m (uji batas)', R);
        cfg.pathXY_raw          = generate_L_path_smooth(10, 10, R, 0.1);
        cfg.agent_filename      = 'Agen_SAC_v5_stage5.mat';
        cfg.prev_agent_filename = 'Agen_SAC_v5_stage4.mat';
    case 6
        % Mendekati siku: radius path DI BAWAH R_min robot.
        % CTE ~0.23 m di tikungan TIDAK BISA DIHINDARI (batas geometris),
        % tetap di bawah ambang terminasi 0.5 m.
        R = 0.75 * Rmin;   % ~1.68 m
        cfg.deskripsi           = sprintf('L mendekati siku, R = %.2f m (< Rmin)', R);
        cfg.pathXY_raw          = generate_L_path_smooth(10, 10, R, 0.1);
        cfg.agent_filename      = 'Agen_SAC_v5_stage6.mat';
        cfg.prev_agent_filename = 'Agen_SAC_v5_stage5.mat';
    otherwise
        error('Stage %d belum didefinisikan.', stage);
end

% --- parameter yang sama untuk semua stage: SEMUA dari robot_params.m ---
% (satu sumber kebenaran -- jangan tulis literal di sini lagi)
p = robot_params();
cfg.stage         = stage;
cfg.Ts            = 0.1;
cfg.MAX_PATH_SIZE = 300;
cfg.V_MIN         = p.v_min;
cfg.V_MAX         = p.v_max;
cfg.DELTA_MAX     = p.delta_max;   % sekarang 31 deg, bukan 0.52 rad

% --- turunan, dihitung sekali di sini supaya training & testing identik ---
d = cfg.pathXY_raw(end,:) - cfg.pathXY_raw(end-1,:);  d = d/norm(d);
ghost = cfg.pathXY_raw(end,:) + ((1:p_robot.n_ghost)'*p_robot.path_spacing).*d;
path_ext = [cfg.pathXY_raw; ghost];
cfg.pathXY_padded = pad_path(path_ext, cfg.MAX_PATH_SIZE);
cfg.dist0     = sum(sqrt(sum(diff(path_ext,1,1).^2, 2)));        % ke ujung hantu
cfg.dist_real = sum(sqrt(sum(diff(cfg.pathXY_raw,1,1).^2, 2)));  % ke tujuan asli
cfg.idx0  = 1;
cfg.x0 = 0; cfg.y0 = 0; cfg.theta0 = 0;
end