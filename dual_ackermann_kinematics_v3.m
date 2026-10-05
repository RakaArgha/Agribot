function [x_next, y_next, theta_next] = dual_ackermann_kinematics_v3(v, delta_f, delta_r, x, y, theta, slip_factor)
%#codegen
% DUAL_ACKERMANN_KINEMATICS_V3
%
% Kinematika bodi untuk robot 4 roda dengan kemudi DEPAN dan BELAKANG.
% Diturunkan dari ICR (Instantaneous Center of Rotation), jadi berlaku
% untuk semua mode sekaligus tanpa perlu if-else mode:
%
%   delta_r = -delta_f  -> dual Ackermann simetris (radius belok terkecil)
%   delta_r = 0         -> Ackermann biasa (setir depan saja)
%   delta_r = +delta_f  -> crab steering (translasi menyamping, tanpa putar)
%
% TURUNAN ICR (body frame: x depan, y kiri, titik asal = tengah robot):
%   Poros depan  di x = +L/2, poros belakang di x = -L/2.
%   ICR = (a,b) harus tegak lurus arah tiap roda:
%       (a - L/2)cos(df) + b sin(df) = 0
%       (a + L/2)cos(dr) + b sin(dr) = 0
%   Diselesaikan:
%       b = L / (tan(df) - tan(dr))
%       a = L/2 - b*tan(df)
%   Kecepatan titik tengah = omega * (b, -a), besarnya = v
%       -> omega = v / sqrt(a^2 + b^2)
%
% CEK SILANG (wajib disebut di skripsi):
%   delta_r = 0       -> b = L/tan(df), a = 0 ... bukan, a = L/2 - L/2 = ... 
%                        a = L/2 - (L/tan df)*tan df = L/2 - L = -L/2
%                        artinya ICR sejajar poros BELAKANG. Itu memang
%                        persis definisi bicycle model klasik. COCOK.
%   delta_r = -df     -> b = L/(2 tan df), a = 0 -> ICR sejajar TENGAH.
%                        R_min = L/(2 tan 0.52) = 0.57 m (vs 1.14 m).
%
% Ukuran port BERUBAH: 7 input (dulu 6, tambah delta_r), 3 output.

    p  = robot_params();
    L  = p.L;
    Ts = 0.1;
    EPS = 1e-6;

    tf = tan(delta_f);
    tr = tan(delta_r);
    denom = tf - tr;

    if abs(denom) < EPS
        % --- TIDAK ADA ROTASI: lurus (df=dr=0) atau crab (df=dr=c) ---
        % Semua roda menghadap arah sama, bodi bertranslasi tanpa berputar.
        omega  = 0;
        vx_b   = v * cos(delta_f);
        vy_b   = v * sin(delta_f);
    else
        b = L / denom;
        a = L/2 - b*tf;
        r_c = sqrt(a*a + b*b);

        omega = v / r_c;
        vx_b  = omega * b;      % kecepatan titik tengah, sumbu x bodi
        vy_b  = omega * (-a);   % komponen menyamping (nol saat simetris)
    end

    % slip mengurangi traksi -> mengurangi kecepatan yang benar-benar terjadi
    vx_b  = slip_factor * vx_b;
    vy_b  = slip_factor * vy_b;
    omega = slip_factor * omega;

    % body frame -> world frame
    ct = cos(theta); st = sin(theta);
    x_next     = x + (vx_b*ct - vy_b*st) * Ts;
    y_next     = y + (vx_b*st + vy_b*ct) * Ts;
    theta_next = theta + omega * Ts;

    % jaga theta di (-pi, pi]
    theta_next = atan2(sin(theta_next), cos(theta_next));
end