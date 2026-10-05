function pathXY = generate_L_path(seg1_len, seg2_len, spacing)
% GENERATE_L_PATH  Membuat waypoint path berbentuk L.
%   pathXY = generate_L_path(8, 5, 0.1)
%   -> segmen 1: dari (0,0) ke (8,0)  [sepanjang sumbu X]
%   -> segmen 2: dari (8,0) ke (8,5)  [belok, sepanjang sumbu Y]
%   spacing = jarak antar titik waypoint (meter), makin kecil makin halus.
%
% Ubah logika di bawah untuk bentuk path lain (S, zigzag, kotak, dst.)

    if nargin < 1, seg1_len = 8; end
    if nargin < 2, seg2_len = 5; end
    if nargin < 3, spacing  = 0.1; end

    % Segmen 1: horizontal
    n1 = round(seg1_len / spacing);
    seg1_x = linspace(0, seg1_len, n1)';
    seg1_y = zeros(n1, 1);

    % Segmen 2: vertikal (mulai dari ujung segmen 1)
    n2 = round(seg2_len / spacing);
    seg2_x = ones(n2, 1) * seg1_len;
    seg2_y = linspace(0, seg2_len, n2)';

    pathXY = [seg1_x, seg1_y; seg2_x, seg2_y];

    % Buang titik duplikat di sambungan
    pathXY = unique(pathXY, 'rows', 'stable');
end