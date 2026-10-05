% =========================================================================
% EKSPOR_BOBOT  Ambil bobot actor SAC untuk dihitung ulang di Python (numpy).
% Pengganti ONNX (Softplus di cabang std tidak didukung ONNX).
%
% OUTPUT: deploy/actor_weights.mat  (dibaca Python dengan scipy.io.loadmat)
% Kirim SELURUH isi Command Window setelah menjalankan skrip ini.
% =========================================================================
clear; clc;

STAGE = 6;   % <<< sama dengan yang dipakai di ekspor_policy_onnx.m

cfg = get_curriculum_config(STAGE);
load(fullfile(pwd,'SavedAgents',cfg.agent_filename));   % -> agentObj
net = getModel(getActor(agentObj));

if ~exist('deploy','dir'), mkdir('deploy'); end

fprintf('===== LAPISAN =====\n');
S = struct();
for i = 1:numel(net.Layers)
    ly = net.Layers(i);
    nm = matlab.lang.makeValidName(ly.Name);
    ukuran = '';
    if isprop(ly,'Weights') && ~isempty(ly.Weights)
        S.(['W_' nm]) = double(ly.Weights);
        S.(['b_' nm]) = double(ly.Bias);
        ukuran = sprintf('W=%dx%d', size(ly.Weights,1), size(ly.Weights,2));
    end
    fprintf('%2d  %-22s %-40s %s\n', i, ly.Name, class(ly), ukuran);
end

fprintf('\n===== SAMBUNGAN =====\n');
C = net.Connections;
for i = 1:height(C)
    fprintf('%-22s -> %s\n', C.Source{i}, C.Destination{i});
end

save(fullfile('deploy','actor_weights.mat'), '-struct', 'S', '-v7');
fprintf('\n>> deploy/actor_weights.mat tersimpan\n');