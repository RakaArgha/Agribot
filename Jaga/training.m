% =========================================================================
% Skrip Final Melatih Agen SAC (Path Following 1 Titik Acak)
% =========================================================================
clear all; clc;
disp('--- Memulai Inisialisasi Sistem ---');

% 1. Pancingan Simulink (Agar blok TX dan TY tidak error merah)
TX = 5; 
TY = 5;
assignin('base', 'TX', TX);
assignin('base', 'TY', TY);

% 2. Definisi Observasi (3 Dimensi: CTE, Heading Error, Jarak)
obsInfo = rlNumericSpec([3 1]);
obsInfo.Name = "observations";

% 3. Definisi Aksi (2 Dimensi: Kecepatan v dan Kemudi delta)
actInfo = rlNumericSpec([2 1], ...
    "LowerLimit", [0.0; -0.52], ...
    "UpperLimit", [1.0;  0.52]);
actInfo.Name = "actions";

% 4. Pembuatan Otak Agen SAC
agentOpts = rlSACAgentOptions;
agentOpts.SampleTime = 0.1; 
agentOpts.CriticOptimizerOptions(1).LearnRate = 1e-3;
agentOpts.CriticOptimizerOptions(2).LearnRate = 1e-3;
agentObj = rlSACAgent(obsInfo, actInfo, agentOpts);

% 5. Mendaftarkan Model Simulink
mdl = "Ackermann_Env";
agentBlk = mdl + "/RL Agent";
env = rlSimulinkEnv(mdl, agentBlk, obsInfo, actInfo);
env.ResetFcn = @(in) reset_lingkungan(in);

% 6. Konfigurasi Pelatihan (Dipaksa 2000 Episode)
trainOpts = rlTrainingOptions(...
    "MaxEpisodes", 1000, ...               
    "MaxStepsPerEpisode", 500, ...        
    "ScoreAveragingWindowLength", 5, ...  
    "StopTrainingCriteria", "EpisodeCount", ... % Berhenti berdasarkan jumlah episode
    "StopTrainingValue", 500, ...              % Harus sampai 2000
    "SaveAgentCriteria", "EpisodeCount", ...    % Simpan otomatis tiap 250 episode
    "SaveAgentValue", 250, ...                   
    "SaveAgentDirectory", pwd + "/SavedAgents", ... 
    "Plots", "training-progress");        

% 7. Memulai Training
disp('--- Memulai Proses Pelatihan... Jangan ditutup! ---');
trainingStats = train(agentObj, env, trainOpts);

% 8. Simpan Manual di Akhir
save('Agen_SAC_2000_Episode.mat', 'agentObj');
disp('--- Pelatihan 2000 Episode Selesai! ---');

% =========================================================================
% FUNGSI LOKAL PENGACAK TARGET
% =========================================================================
function in = reset_lingkungan(in)
    in = setVariable(in, 'x0', 0); 
    % X antara 3 s/d 8, Y antara -5 s/d 5
    target_x_baru = 3 + rand() * 5; 
    target_y_baru = -5 + rand() * 10;
    
    in = setVariable(in, 'TX', target_x_baru);
    in = setVariable(in, 'TY', target_y_baru);
end