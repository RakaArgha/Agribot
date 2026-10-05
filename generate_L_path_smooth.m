function pathXY = generate_L_path_smooth(Lx, Ly, R, res)
% GENERATE_L_PATH_SMOOTH  Path L dengan tikungan berjari-jari R (fillet busur).
%
% KENAPA INI PENTING
%   Radius belok minimum robot = L/tan(max_steer) = 0.65/tan(0.52) = 1.14 m.
%   Sudut siku 90 derajat punya radius 0 m -> SECARA GEOMETRIS TIDAK BISA
%   DIIKUTI. Jadi reward menyuruh agent melakukan sesuatu yang mustahil, lalu
%   menghukumnya karena tidak bisa. Pure Pursuit "berhasil" justru karena dia
%   memotong sudut (CTE maks 0.29 m) -- bukan karena sudut siku itu feasible.
%
%   Dengan R >= 1.5 * R_min (~1.7 m), lintasan referensi jadi feasible dan
%   konflik "ikuti persis" vs "mungkin dilakukan" hilang.
%
% CURRICULUM YANG DISARANKAN (ganti R saja, reward tidak usah diutak-atik):
%   Stage 3a : R = 3.0   (tikungan landai)
%   Stage 3b : R = 2.0
%   Stage 3c : R = 1.7   (~1.5 x R_min, target akhir)
%   Stage 3d : R = 1.4   (mepet; opsional, untuk bab "batas kemampuan")
%
% Contoh:
%   pathXY = generate_L_path_smooth(5, 5, 1.7, 0.05);
%   pathXY = pad_path(pathXY, 300);

if nargin < 1 || isempty(Lx),  Lx  = 5.0;  end
if nargin < 2 || isempty(Ly),  Ly  = 5.0;  end
if nargin < 3 || isempty(R),   R   = 1.7;  end
if nargin < 4 || isempty(res), res = 0.05; end

R = min(R, min(Lx, Ly) * 0.9);     % jangan sampai busur melewati ujung

% Segmen 1: lurus sepanjang sumbu X, berhenti R sebelum sudut
x1 = (0 : res : (Lx - R))';
seg1 = [x1, zeros(numel(x1), 1)];

% Busur: pusat di (Lx - R, R), dari sudut -90 deg ke 0 deg
dtheta = res / R;
th = (-pi/2 : dtheta : 0)';
arc = [(Lx - R) + R*cos(th), R + R*sin(th)];

% Segmen 2: lurus naik sumbu Y dari y = R sampai Ly
y2 = ((R + res) : res : Ly)';
seg2 = [Lx * ones(numel(y2), 1), y2];

pathXY = [seg1; arc(2:end, :); seg2];

% buang titik kembar akibat penyambungan
d = [1; sqrt(sum(diff(pathXY,1,1).^2, 2))];
pathXY = pathXY(d > 1e-9, :);
end