% =========================================================================

% Skrip Pengujian AGRIBOT (L-Path dan S-Path)

% =========================================================================

clc; close all;

%% 1. MEMBUAT LINTASAN (WAYPOINTS)

disp('Memilih Lintasan Uji...');

pilihan_path = 'L'; % Ganti menjadi 'S' jika ingin menguji S-Path

if strcmp(pilihan_path, 'L')

% --- L-PATH ---

% Bagian 1: Maju lurus dari X=0 ke X=5 (Y tetap 0)

x_lurus = 0:0.1:5;

y_lurus = zeros(1, length(x_lurus));


% Bagian 2: Belok kiri patah, maju dari Y=0 ke Y=5 (X tetap 5)

y_belok = 0.1:0.1:5;

x_belok = 5 * ones(1, length(y_belok));


% Gabungkan array

Path_X = [x_lurus, x_belok];

Path_Y = [y_lurus, y_belok];

disp('L-Path (Sudut Siku) Berhasil Dibuat!');

elseif strcmp(pilihan_path, 'S')

% --- S-PATH ---

% Menggunakan fungsi sinus untuk membuat gelombang S yang mulus

% Robot maju sejauh 10 meter ke depan (X), berbelok-belok sejauh 3 meter (Y)

Path_X = 0:0.1:10;

Path_Y = 3 * sin(Path_X * (pi/5));

disp('S-Path (Gelombang Sinus) Berhasil Dibuat!');

end

%% 2. VISUALISASI LINTASAN UJI (Melihat Bentuk Arena)

figure('Name', 'Arena Ujian AGRIBOT');

plot(Path_X, Path_Y, 'b--', 'LineWidth', 2);

hold on;

plot(Path_X(1), Path_Y(1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g'); % Start

plot(Path_X(end), Path_Y(end), 'rx', 'MarkerSize', 10, 'LineWidth', 2); % Finish

grid on;

axis equal;

title(['Rencana Lintasan: ', pilihan_path, '-Path']);

xlabel('Posisi X (meter)');

ylabel('Posisi Y (meter)');

legend('Jalur Target', 'Titik Start', 'Garis Finis', 'Location', 'best');

%% 3. PERSIAPAN AGEN AI & SIMULASI

disp('Memuat Agen AI yang sudah pintar...');

% Memuat hasil training (Pastikan nama file sesuai dengan yang tersimpan nanti)

% Kita gunakan 'try-catch' agar tidak error jika namanya berbeda

try

load('Agen_SAC_2000_Episode.mat', 'agentObj');

disp('Agen berhasil dimuat dari Agen_SAC_2000_Episode.mat');

catch

disp('File tidak ditemukan. Pastikan nama file training sesuai.');

end

% Set opsi simulasi

simOpts = rlSimulationOptions('MaxSteps', 2000, 'StopOnError', 'on');

%% 4. LOGIKA PENGUJIAN (WAYPOINT FOLLOWER)

% CATATAN: Bagian ini adalah rancangan bagaimana kita akan mengeksekusi testing.

% Karena Simulink (Constant TX dan TY) hanya membaca 1 nilai saat awal dimulai,

% kita nanti harus sedikit memodifikasi Simulink agar bisa membaca array Path_X dan Path_Y,

% ATAU kita menggunakan skrip untuk menggeser target TX dan TY secara bertahap.

disp('--- Menyiapkan Logika Waypoint ---');

disp('Sistem siap: Robot akan mengejar Titik 1.');

disp('Jika Jarak Robot ke Titik 1 < 0.2 meter, target bergeser ke Titik 2, dst.');

disp('--------------------------------------------------');

disp('Silakan tunggu proses training 2000 episode selesai terlebih dahulu...');