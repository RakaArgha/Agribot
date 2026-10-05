function [x_next, y_next, theta_next] = dual_ackermann_kinematics_v4(v, delta_f, delta_r, x, y, theta)
%#codegen
% DUAL_ACKERMANN_KINEMATICS_V4
% Sama seperti v3, TAPI slip_factor DIHAPUS TOTAL. Simulasi sekarang
% idealized murni (traksi sempurna). Slip akan dimodelkan nanti di
% simulasi 3D, bukan di sini.
%
% PORT BERUBAH: 6 input (dulu 7), 3 output.
% Di Simulink: HAPUS port input 'slip_factor' pada blok ini, dan HAPUS
% blok Constant 'slip_factor' di level Ackermann_Env.
%
% Kinematika bodi 4 roda, kemudi depan DAN belakang, diturunkan dari ICR:
%   delta_r = -delta_f  -> dual Ackermann simetris (R_min terkecil)
%   delta_r = 0         -> Ackermann biasa (setir depan saja)
%   delta_r = +delta_f  -> CRAB (translasi menyamping, tanpa berputar)
%
%   b = L / (tan(df) - tan(dr))      <- ICR koordinat y
%   a = L/2 - b*tan(df)              <- ICR koordinat x
%   kecepatan titik tengah = omega*(b, -a), besarnya = v

    p  = robot_params();
    L  = p.L;
    Ts = 0.1;
    EPS = 1e-6;

    tf = tan(delta_f);
    tr = tan(delta_r);
    denom = tf - tr;

    if abs(denom) < EPS
        % tidak ada rotasi: lurus (df=dr=0) atau crab (df=dr=c)
        omega = 0;
        vx_b  = v * cos(delta_f);
        vy_b  = v * sin(delta_f);
    else
        b = L / denom;
        a = L/2 - b*tf;
        r_c = sqrt(a*a + b*b);

        omega = sign(b) * v / r_c;
        vx_b  = omega * b;
        vy_b  = omega * (-a);   % NOL saat simetris; tidak nol = menyamping
    end

    ct = cos(theta); st = sin(theta);
    x_next     = x + (vx_b*ct - vy_b*st) * Ts;
    y_next     = y + (vx_b*st + vy_b*ct) * Ts;
    theta_next = theta + omega * Ts;
    theta_next = atan2(sin(theta_next), cos(theta_next));
end