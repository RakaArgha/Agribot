function p = robot_params()
%#codegen
% ROBOT_PARAMS  SATU-SATUNYA SUMBER KEBENARAN parameter fisik robot.
% Nilai geometri diambil dari kode kontrol supervisor (get_ackermann_logic).
%
% CATATAN CODEGEN: hitung semua pakai variabel lokal dulu, rakit struct
% SEKALI di akhir (Coder melarang tambah field setelah struct dibaca).
    
    % --- Geometri dari kode supervisor ---
    L_half = 0.86;          % setengah wheelbase (m)
    L      = 2*L_half;      % wheelbase = 1.72 m
    W      = 1.65;          % track width (m)
    n_ghost = 50;

    % --- Batas MEKANIK per roda (bukan sudut virtual) ---
    delta_wheel_max = 31.2 * pi/180;

    % --- Sudut VIRTUAL maksimum (yang dikeluarkan agent) ---
    % Pada mode simetris, roda DALAM selalu berbelok paling tajam.
    % Batas tercapai saat roda dalam = 31.2 deg:
    %   R_min     = W/2 + L_half/tan(delta_wheel_max)   = 2.245 m
    %   delta_max = atan(L_half / R_min)                = 20.96 deg
    % Dengan batas ini, tidak ada roda yang pernah melewati 31.2 deg.
    R_min_dual = W/2 + L_half/tan(delta_wheel_max);
    delta_max  = atan(L_half / R_min_dual);

    v_min = 0.2;
    v_max = 1.0;

    % PLACEHOLDER - belum ada spek motor kemudi. Ganti dengan datasheet.
    steer_rate_max = 1.0;   % rad/s

    path_spacing = 0.1;

    % --- rakit struct sekali ---
    p.L_half          = L_half;
    p.L               = L;
    p.W               = W;
    p.delta_wheel_max = delta_wheel_max;
    p.delta_max       = delta_max;
    p.R_min_dual      = R_min_dual;
    p.v_min           = v_min;
    p.v_max           = v_max;
    p.steer_rate_max  = steer_rate_max;
    p.path_spacing    = path_spacing;
    p.n_ghost = n_ghost;
end