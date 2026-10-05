% =========================================================================
% TRAINING SAC v4 - semua pengaturan stage diambil dari
% get_curriculum_config.m (satu sumber kebenaran). TANPA SLIP.
% =========================================================================
clear all; clc;

STAGE = 6;   % <<< GANTI CUMA ANGKA INI (1..4)

cfg = get_curriculum_config(STAGE);

if ~exist('SavedAgents','dir'), mkdir('SavedAgents'); end
Simulink.sdi.setAutoArchiveMode(false);
Simulink.sdi.clear;

fprintf('=== TRAINING STAGE %d: %s ===\n', STAGE, cfg.deskripsi);
fprintf('Titik path asli: %d | panjang busur: %.2f m | ujung: (%.2f, %.2f)\n', ...
    size(cfg.pathXY_raw,1), cfg.dist0, cfg.pathXY_raw(end,1), cfg.pathXY_raw(end,2));
fprintf('v in [%.2f, %.2f] | delta_max virtual = %.3f rad (%.2f deg)\n', ...
    cfg.V_MIN, cfg.V_MAX, cfg.DELTA_MAX, rad2deg(cfg.DELTA_MAX));

assignin('base','pathXY_raw',    cfg.pathXY_raw);
assignin('base','pathXY_padded', cfg.pathXY_padded);
assignin('base','x0',    cfg.x0);
assignin('base','y0',    cfg.y0);
assignin('base','theta0',cfg.theta0);
assignin('base','dist0', cfg.dist0);
assignin('base','idx0',  cfg.idx0);
assignin('base','Ts',    cfg.Ts);

% --- agent ---
if isempty(cfg.prev_agent_filename)
    fprintf('Mulai dari NOL.\n');
    obsInfo = rlNumericSpec([23 1]); obsInfo.Name = "observations";
    actInfo = rlNumericSpec([2 1], ...
        "LowerLimit", [cfg.V_MIN; -cfg.DELTA_MAX], ...
        "UpperLimit", [cfg.V_MAX;  cfg.DELTA_MAX]);
    actInfo.Name = "actions";

    agentOpts = rlSACAgentOptions;
    agentOpts.SampleTime             = cfg.Ts;
    agentOpts.DiscountFactor         = 0.99;
    agentOpts.MiniBatchSize          = 128;
    agentOpts.ExperienceBufferLength = 1e6;
    agentOpts.TargetSmoothFactor     = 5e-3;
    agentOpts.NumWarmStartSteps      = 2000;
    agentOpts.CriticOptimizerOptions(1).LearnRate = 1e-3;
    agentOpts.CriticOptimizerOptions(2).LearnRate = 1e-3;
    agentOpts.ActorOptimizerOptions.LearnRate     = 1e-4;

    agentObj = rlSACAgent(obsInfo, actInfo, agentOpts);
else
    fprintf('WARM START dari: %s\n', cfg.prev_agent_filename);
    prev_path = fullfile(pwd,'SavedAgents',cfg.prev_agent_filename);
    if ~isfile(prev_path)
        error('Agent stage sebelumnya tidak ada: %s', prev_path);
    end
    load(prev_path);
    if ~exist('agentObj','var')
        error('agentObj tidak ada di %s', cfg.prev_agent_filename);
    end
    agentObj.UseExplorationPolicy = true;
    obsInfo = getObservationInfo(agentObj);
    actInfo = getActionInfo(agentObj);
end

mdl = "Ackermann_Env";
agentBlk = mdl + "/RL Agent";
env = rlSimulinkEnv(mdl, agentBlk, obsInfo, actInfo);
env.ResetFcn = @(in) reset_lingkungan_v4(in, cfg);

% --- Early stopping: ambang = 85% reward maksimum teoretis stage ini ---
% (progress penuh 30 x panjang path + bonus goal 300), bukan angka tebakan.
% Rata-rata diambil dari 50 episode supaya tidak berhenti karena kebetulan.
R_max_teori = 30*cfg.dist_real + 300;
stop_val    = 0.85 * R_max_teori;
fprintf('Reward maks teoretis = %.0f | ambang berhenti = %.0f\n', R_max_teori, stop_val);

trainOpts = rlTrainingOptions(...
    "MaxEpisodes", 1500, ...
    "MaxStepsPerEpisode", 400, ...
    "ScoreAveragingWindowLength", 50, ...
    "StopTrainingCriteria", "AverageReward", ...
    "StopTrainingValue", stop_val, ...
    "SaveAgentCriteria", "AverageReward", ...
    "SaveAgentValue", 0.80 * R_max_teori, ...
    "SaveAgentDirectory", pwd + "/SavedAgents", ...
    "Plots", "training-progress");

fprintf('--- Mulai training STAGE %d ---\n', STAGE);
trainingStats = train(agentObj, env, trainOpts);

save(fullfile(pwd,'SavedAgents',cfg.agent_filename), 'agentObj');
fprintf('--- STAGE %d selesai -> %s ---\n', STAGE, cfg.agent_filename);

% =========================================================================
function in = reset_lingkungan_v4(in, cfg)
    % Randomisasi kecil pose awal -- mencegah agent menghafal SATU trajektori.
    x0     = 0.05*(2*rand()-1);        % +-5 cm
    y0     = 0.05*(2*rand()-1);        % +-5 cm
    theta0 = 0.05*(2*rand()-1);        % +-0.05 rad (~2.9 deg)

    in = setVariable(in, 'x0',     x0);
    in = setVariable(in, 'y0',     y0);
    in = setVariable(in, 'theta0', theta0);
    in = setVariable(in, 'idx0',   cfg.idx0);
end