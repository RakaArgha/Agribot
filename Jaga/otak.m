% 1. Definisi Observasi (2 Dimensi: Jarak dan Error Sudut)
obsInfo = rlNumericSpec([2 1]);
obsInfo.Name = "observations";

% 2. Definisi Aksi (2 Dimensi: Kecepatan v dan Kemudi delta)
% v     : 0.0 hingga 1.0 m/s
% delta : -0.52 hingga 0.52 rad (sekitar 30 derajat)
actInfo = rlNumericSpec([2 1], ...
    "LowerLimit", [0.0; -0.52], ...
    "UpperLimit", [1.0;  0.52]);
actInfo.Name = "actions";

% 3. Konfigurasi Dasar SAC
agentOpts = rlSACAgentOptions;
agentOpts.SampleTime = 0.1; % Harus sama dengan dt di blok kinematics

% 4. Pembuatan Agen (Menggunakan arsitektur jaringan default MATLAB)
agentObj = rlSACAgent(obsInfo, actInfo, agentOpts);

disp("===========================================")
disp("Otak agentObj berhasil dibuat di Workspace!")
disp("===========================================")