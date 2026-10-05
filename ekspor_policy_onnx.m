% =========================================================================
% EKSPOR_POLICY_ONNX  Hari 1: ekspor actor SAC ke ONNX + data rujukan.
%
% OUTPUT (folder ./deploy/):
%   sac_actor.onnx   -> jaringan actor untuk dijalankan di Python
%   ref_obs.csv      -> 2000 observasi uji (tiap baris = 23 elemen)
%   ref_act.csv      -> aksi MATLAB untuk tiap observasi (v, delta)
%   deploy_info.txt  -> batas aksi, nama input/output, parameter robot
%
% SYARAT: Deep Learning Toolbox Converter for ONNX Model Format
%   (Home -> Add-Ons -> cari "ONNX" -> install)
%
% ref_act adalah "kunci jawaban": besok Python harus menghasilkan aksi
% yang SAMA untuk ref_obs. Kalau beda, porting salah -- JANGAN ke robot.
% =========================================================================
clear; clc;

STAGE = 6;   % <<< agent yang mau dipakai di lapangan

cfg = get_curriculum_config(STAGE);
p   = robot_params();
load(fullfile(pwd,'SavedAgents',cfg.agent_filename));   % -> agentObj
agentObj.UseExplorationPolicy = false;                  % deterministik

if ~exist('deploy','dir'), mkdir('deploy'); end

% --- 1. ekspor jaringan actor ---
actor = getActor(agentObj);
net   = getModel(actor);
fprintf('Input  jaringan: %s\n', strjoin(string(net.InputNames), ', '));
fprintf('Output jaringan: %s\n', strjoin(string(net.OutputNames), ', '));
exportONNXNetwork(net, fullfile('deploy','sac_actor.onnx'));
fprintf('>> sac_actor.onnx tersimpan\n');

% --- 2. observasi uji dalam rentang realistis (skala sama dg get_observation_v6) ---
N = 2000;
rng(42);
obs = zeros(23, N);
obs(1,:) = 0.5*(2*rand(1,N)-1);              % CTE (m)
obs(2,:) = 0.15*(2*rand(1,N)-1);             % heading_err / pi
obs(3,:) = abs(obs(1,:)) + 0.02*rand(1,N);   % dist_nearest (m)
for k = 1:10
    obs(2+2*k,:) = (k*0.5 + 0.1*randn(1,N))/5;   % waypoint lx / 5
    obs(3+2*k,:) = (0.3*randn(1,N))/5;           % waypoint ly / 5
end

% --- 3. aksi MATLAB = kunci jawaban ---
act = zeros(N, 2);
for i = 1:N
    a = getAction(agentObj, {obs(:,i)});
    act(i,:) = double(a{1})';
end
writematrix(obs', fullfile('deploy','ref_obs.csv'));
writematrix(act,  fullfile('deploy','ref_act.csv'));
fprintf('>> ref_obs.csv & ref_act.csv tersimpan (%d sampel)\n', N);
fprintf('   rentang v     : [%.3f, %.3f]\n', min(act(:,1)), max(act(:,1)));
fprintf('   rentang delta : [%.3f, %.3f] rad\n', min(act(:,2)), max(act(:,2)));

% --- 4. info yang dibutuhkan sisi Python ---
fid = fopen(fullfile('deploy','deploy_info.txt'),'w');
fprintf(fid,'stage=%d\nagent=%s\n', STAGE, cfg.agent_filename);
fprintf(fid,'input_names=%s\n',  strjoin(string(net.InputNames), ','));
fprintf(fid,'output_names=%s\n', strjoin(string(net.OutputNames), ','));
fprintf(fid,'v_min=%.6f\nv_max=%.6f\n', cfg.V_MIN, cfg.V_MAX);
fprintf(fid,'delta_max_virtual=%.6f\n', cfg.DELTA_MAX);
fprintf(fid,'L_half=%.4f\nW=%.4f\n', p.L_half, p.W);
fprintf(fid,'delta_wheel_max=%.6f\nsteer_rate_max=%.4f\n', p.delta_wheel_max, p.steer_rate_max);
fprintf(fid,'Ts=%.3f\nn_ghost=%d\npath_spacing=%.3f\n', cfg.Ts, p.n_ghost, p.path_spacing);
fclose(fid);
fprintf('>> deploy_info.txt tersimpan\n\nKirim isi Command Window ini + deploy_info.txt.\n');