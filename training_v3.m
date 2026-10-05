% =========================================================================
% TRAINING SAC v3 - Curriculum-based (auto warm-start, fix disk penuh)
% =========================================================================
clear all; clc;

STAGE = 3;  % <<< GANTI CUMA ANGKA INI untuk pindah stage (1=lurus, 2=L)

if ~exist('SavedAgents','dir'), mkdir('SavedAgents'); end

% --- FIX DISK PENUH: matikan auto-archive Simulink Data Inspector ---
Simulink.sdi.setAutoArchiveMode(false);
Simulink.sdi.clear;

[seg1_len, seg2_len, spacing, slip_enabled, agent_filename, prev_agent_filename] = get_curriculum_config(STAGE);

fprintf('=== TRAINING STAGE %d ===\n', STAGE);
fprintf('seg1_len=%.2f  seg2_len=%.2f  slip_enabled=%d\n', seg1_len, seg2_len, slip_enabled);

pathXY_raw = generate_L_path(seg1_len, seg2_len, spacing);
MAX_PATH_SIZE = 300;
pathXY_padded = pad_path(pathXY_raw, MAX_PATH_SIZE);

fprintf('Titik akhir path: (%.2f, %.2f) | Jumlah titik asli: %d\n', ...
    pathXY_raw(end,1), pathXY_raw(end,2), size(pathXY_raw,1));

x0 = 0; y0 = 0; theta0 = 0;
dist0 = sqrt((pathXY_padded(end,1)-x0)^2 + (pathXY_padded(end,2)-y0)^2);
idx0 = 1;
if slip_enabled
    slip_factor = 0.7 + 0.3*rand();
else
    slip_factor = 1.0;   % STAGE ini belum pakai slip
end
assignin('base', 'slip_enabled', slip_enabled);

assignin('base', 'pathXY_padded', pathXY_padded);
assignin('base', 'x0', x0);
assignin('base', 'y0', y0);
assignin('base', 'theta0', theta0);
assignin('base', 'dist0', dist0);
assignin('base', 'idx0', idx0);
assignin('base', 'slip_factor', slip_factor);
assignin('base', 'Ts', 0.1);

if isempty(prev_agent_filename)
    fprintf('Mulai dari NOL (tidak ada warm start)\n');
    obsInfo = rlNumericSpec([23 1]); obsInfo.Name = "observations";
    actInfo = rlNumericSpec([2 1], "LowerLimit", [0.0; -0.52], "UpperLimit", [1.0; 0.52]);
    actInfo.Name = "actions";
    agentOpts = rlSACAgentOptions;
    agentOpts.SampleTime = 0.1;
    agentOpts.CriticOptimizerOptions(1).LearnRate = 1e-3;
    agentOpts.CriticOptimizerOptions(2).LearnRate = 1e-3;
    agentObj = rlSACAgent(obsInfo, actInfo, agentOpts);
else
    fprintf('WARM START dari: %s\n', prev_agent_filename);
    prev_path = fullfile(pwd, 'SavedAgents', prev_agent_filename);
    if ~isfile(prev_path)
        error('File agent sebelumnya tidak ditemukan: %s', prev_path);
    end
    load(prev_path);   % harus berisi variabel agentObj
    if ~exist('agentObj','var'), error('agentObj tidak ada di file %s', prev_agent_filename); end
    obsInfo = getObservationInfo(agentObj);
    actInfo = getActionInfo(agentObj);
end

mdl = "Ackermann_Env";
agentBlk = mdl + "/RL Agent";
env = rlSimulinkEnv(mdl, agentBlk, obsInfo, actInfo);
env.ResetFcn = @(in) reset_lingkungan_v3(in, slip_enabled);

trainOpts = rlTrainingOptions(...
    "MaxEpisodes", 1000, ...
    "MaxStepsPerEpisode", 400, ...
    "ScoreAveragingWindowLength", 5, ...
    "StopTrainingCriteria", "EpisodeCount", ...
    "StopTrainingValue", 1000, ...
    "SaveAgentCriteria", "EpisodeCount", ...
    "SaveAgentValue", 1000, ...   % simpan HANYA sekali di akhir -> hindari disk penuh
    "SaveAgentDirectory", pwd + "/SavedAgents", ...
    "Plots", "training-progress");

fprintf('--- Mulai training STAGE %d ---\n', STAGE);
trainingStats = train(agentObj, env, trainOpts);

save(fullfile(pwd, 'SavedAgents', agent_filename), 'agentObj');
fprintf('--- STAGE %d selesai, disimpan sebagai: %s ---\n', STAGE, agent_filename);

% =========================================================================
function in = reset_lingkungan_v3(in, slip_enabled)
    in = setVariable(in, 'x0', 0);
    in = setVariable(in, 'y0', 0);
    in = setVariable(in, 'theta0', 0);
    in = setVariable(in, 'idx0', 1);
    if slip_enabled
        slip_factor = 0.7 + 0.3*rand();   % slip acak BARU tiap episode
    else
        slip_factor = 1.0;
    end
    in = setVariable(in, 'slip_factor', slip_factor);
end