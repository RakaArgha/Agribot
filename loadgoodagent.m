% =========================================================================
% Skrip untuk Memuat Agen AI yang Sudah Dilatih
% =========================================================================

clear; clc;

% Membuka jendela dialog untuk memilih file .mat
[nama_file, lokasi_folder] = uigetfile({'*.mat', 'File Agen MATLAB (*.mat)'}, ...
    'Pilih File Agen Hasil Training', pwd + "/SavedAgents");

% Mengecek apakah pengguna membatalkan pilihan
if isequal(nama_file, 0)
    disp('❌ Dibatalkan: Tidak ada file agen yang dipilih.');
else
    % Menggabungkan path dan memuat file
    jalur_lengkap = fullfile(lokasi_folder, nama_file);
    load(jalur_lengkap, 'agentObj');
    
    disp('======================================================')
    disp(['✅ BERHASIL: Agen dimuat dari -> ', nama_file]);
    disp('Otak AI (agentObj) sudah aktif di Workspace!');
    disp('Silakan buka Simulink dan tekan tombol RUN untuk menguji.');
    disp('======================================================')
end