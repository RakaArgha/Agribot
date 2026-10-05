function pathXY_padded = pad_path(pathXY_raw, MAX_PATH_SIZE)
% PAD_PATH  Memperbesar path ke ukuran tetap MAX_PATH_SIZE x 2.
%
% PENTING: padding dilakukan dengan MENGULANG TITIK TERAKHIR.
% Semua fungsi baru (get_observation_v6, get_reward_v11, check_isdone_v4)
% mendeteksi panjang path asli dengan cara mencari titik terakhir yang
% BERBEDA dari titik sebelumnya. Kalau padding diisi nol/NaN, deteksi itu
% gagal dan bug index akan balik lagi.

N = size(pathXY_raw, 1);

if N > MAX_PATH_SIZE
    error(['Path punya %d titik, melebihi MAX_PATH_SIZE=%d. ' ...
           'Perbesar "res"/spacing atau perbesar MAX_PATH_SIZE ' ...
           '(dan ukuran port pathXY di Simulink).'], N, MAX_PATH_SIZE);
end

pathXY_padded = zeros(MAX_PATH_SIZE, 2);
pathXY_padded(1:N, :) = pathXY_raw;
for i = N+1 : MAX_PATH_SIZE
    pathXY_padded(i, :) = pathXY_raw(N, :);   % ulangi titik terakhir
end
end