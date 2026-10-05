% =========================================================================
% TRAINING SAC v2 - Path Following dengan Lookahead
% Curriculum Step 1: path LURUS pendek dulu
% =========================================================================
clear all; clc;
disp('--- Inisialisasi Sistem Path-Following v2 ---');

% 1. Generate path (curriculum step 1: lurus pendek, seg2_len=0)
seg1_len = 5; seg2_len = 0; spacing = 0.1;
pathXY_raw = generate_L_path(seg1_len, seg2_len, spacing);

MAX_PATH_SIZE = 300;   % ukuran tetap untuk semua path (lihat pad_path.m)
pathXY_padded = pad_path(pathXY_raw, MAX_PATH_SIZE);

x0 = 0; y0 = 0; theta0 = 0;
dist0 = sqrt((pathXY_padded(end,1)-x0)^2 + (pathXY_padded(end,2)-y0)^2);

assignin('base', 'pathXY_padded', pathXY_padded);
assignin('base', 'x0', x0);
assignin('base', 'y0', y0);
assignin('base', 'theta0', theta0);
assignin('base', 'dist0', dist0);
assignin('base', 'Ts', 0.1);

% 2. Observation (23 dim: CTE, heading_err, dist_nearest, 10x2 lookahead)
obsInfo = rlNumericSpec([23 1]);
obsInfo.Name = "observations";

% 3. Action (v, delta)
actInfo = rlNumericSpec([2 1], ...
    "LowerLimit", [0.0; -0.52], ...
    "UpperLimit", [1.0;  0.52]);
actInfo.Name = "actions";

% 4. Agent SAC
agentOpts = rlSACAgentOptions;
agentOpts.SampleTime = 0.1;
agentOpts.CriticOptimizerOptions(1).LearnRate = 1e-3;
agentOpts.CriticOptimizerOptions(2).LearnRate = 1e-3;
agentObj = rlSACAgent(obsInfo, actInfo, agentOpts);

% 5. Daftarkan model Simulink
mdl = "Ackermann_Env";
agentBlk = mdl + "/RL Agent";
env = rlSimulinkEnv(mdl, agentBlk, obsInfo, actInfo);
env.ResetFcn = @(in) reset_lingkungan_v2(in);

% 6. Opsi training
trainOpts = rlTrainingOptions(...
    "MaxEpisodes", 1000, ...
    "MaxStepsPerEpisode", 300, ...
    "ScoreAveragingWindowLength", 5, ...
    "StopTrainingCriteria", "EpisodeCount", ...
    "StopTrainingValue", 1000, ...
    "SaveAgentCriteria", "EpisodeCount", ...
    "SaveAgentValue", 250, ...
    "SaveAgentDirectory", pwd + "/SavedAgents", ...
    "Plots", "training-progress");

disp('--- Mulai training: CURRICULUM STEP 1 (path lurus) ---');
disp('Tunggu sampai grafik reward terlihat konvergen/stabil sebelum lanjut ke path L.');
trainingStats = train(agentObj, env, trainOpts);

save('Agen_SAC_v2_curriculum1.mat', 'agentObj');
% save('Agen_SAC_v2_curriculum2.mat', 'agentObj');
disp('--- Training curriculum step 1 selesai! ---');

% =========================================================================
function in = reset_lingkungan_v2(in)
    % Curriculum step 1: posisi start & path tetap sama tiap episode.
    % (Nanti di curriculum step 2, fungsi ini yang diubah untuk
    %  mengacak/memperbesar path secara bertahap.)
    in = setVariable(in, 'x0', 0);
    in = setVariable(in, 'y0', 0);
    in = setVariable(in, 'theta0', 0);
end