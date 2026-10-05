function isdone = check_isdone_v4(x, y, pathXY)
%#codegen
% CHECK_ISDONE_V4
%
% Tiga syarat berhenti:
%   1. SAMPAI   : jarak asli ke titik akhir < 0.3 m   (cocok dengan r_goal)
%   2. NYASAR   : jarak ke titik path terdekat > 1.5 m (cocok dengan r_off)
%   3. KABUR    : posisi di luar kotak +-50 m (pengaman numerik)
%
% Syarat ke-2 adalah yang membunuh exploit "robot muter-muter": dulu robot
% bisa berputar 400 step dan mengisi replay buffer dengan sampah. Sekarang
% episodenya dipotong dalam beberapa step.
%
% Sengaja TIDAK menambah port input: jarak ke path dihitung sendiri di sini
% dengan scan penuh (bukan windowed), jadi tidak bisa tertipu index yang
% merayap. Ukuran port tetap 3 input / 1 output -> tanpa rewiring.

    N = panjang_path_asli(pathXY);

    % 1. sampai tujuan
    dx_g = x - pathXY(N,1);
    dy_g = y - pathXY(N,2);
    d_goal = sqrt(dx_g*dx_g + dy_g*dy_g);

    % 2. jarak ke path terdekat (scan seluruh path asli)
    best = inf;
    for i = 1:N
        dx = pathXY(i,1) - x;
        dy = pathXY(i,2) - y;
        d = dx*dx + dy*dy;
        if d < best
            best = d;
        end
    end
    d_path = sqrt(best);

    if (d_goal < 0.3) || (d_path > 1.5) || (abs(x) > 50) || (abs(y) > 50)
        isdone = 1;
    else
        isdone = 0;
    end
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