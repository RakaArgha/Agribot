function [delta_wheels, v_wheels] = wheel_commands(v, delta_f, delta_r)
%#codegen
% WHEEL_COMMANDS  Menerjemahkan perintah tingkat-kendaraan (v, delta_f,
% delta_r) menjadi perintah 4 RODA INDEPENDEN.
%
% KENAPA INI DIBUTUHKAN
%   Robot tidak punya tie rod. Keempat roda dikemudikan sendiri-sendiri,
%   jadi TIDAK BOLEH keempatnya diberi sudut yang sama -- roda akan saling
%   melawan, ban tergerus, dan lintasan meleset dari prediksi kinematika.
%   Sudut yang benar diturunkan dari syarat geometris: garis tegak lurus
%   arah putar SETIAP roda harus bertemu di satu titik (ICR).
%
% URUTAN OUTPUT (4x1):
%   1 = FL (depan kiri)   2 = FR (depan kanan)
%   3 = RL (belakang kiri) 4 = RR (belakang kanan)
%
% CATATAN PENTING
%   Fungsi ini TIDAK ikut dalam loop RL. Agent tetap mengeluarkan perintah
%   tingkat kendaraan; blok ini hanya menerjemahkan. Jadi bisa dipasang
%   sebagai blok monitoring (output ke To Workspace) tanpa mengganggu
%   training sama sekali -- dan isinya nanti langsung dipakai di node ROS 2
%   yang mengirim perintah lewat CAN bus.
%
%   v_wheels adalah kecepatan LINEAR titik kontak roda (m/s). Untuk jadi
%   RPM motor: rpm = v_wheel / (2*pi*r_roda) * 60.

    p   = robot_params();
    L   = p.L;
    W   = p.W;
    EPS = 1e-6;

    % posisi keempat roda di body frame (x depan, y kiri)
    px = [ L/2;  L/2; -L/2; -L/2];
    py = [ W/2; -W/2;  W/2; -W/2];

    delta_wheels = zeros(4,1);
    v_wheels     = zeros(4,1);

    tf = tan(delta_f);
    tr = tan(delta_r);
    denom = tf - tr;

    if abs(denom) < EPS
        % --- lurus atau crab: semua roda sejajar, kecepatan sama ---
        for i = 1:4
            delta_wheels(i) = delta_f;
            v_wheels(i)     = v;
        end
        return;
    end

    b = L / denom;          % ICR: koordinat y
    a = L/2 - b*tf;         % ICR: koordinat x
    r_c = sqrt(a*a + b*b);
    omega = v / r_c;        % yaw rate

    for i = 1:4
        % kecepatan roda i = omega x (P_i - ICR), dalam 2D:
        vxi = omega * (b - py(i));
        vyi = omega * (px(i) - a);

        delta_wheels(i) = atan2(vyi, vxi);
        v_wheels(i)     = sqrt(vxi*vxi + vyi*vyi);

        % roda yang "menghadap ke belakang" (mundur relatif bodi):
        % balik tanda kecepatan dan putar sudutnya 180 deg supaya sudut
        % kemudi tetap di rentang wajar, bukan mendekati +-pi.
        if vxi < 0
            v_wheels(i)     = -v_wheels(i);
            delta_wheels(i) = atan2(-vyi, -vxi);
        end
    end
end