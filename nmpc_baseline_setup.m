% =========================================================================
% NMPC BASELINE - Ackermann Path Following (model IDEAL, tanpa slip)
% Baseline pembanding untuk SAC. Butuh MPC Toolbox (sudah kamu punya).
% =========================================================================
clear; clc;

L  = 0.65;   % wheelbase, SAMAKAN dengan ackermann_kinematics
nx = 3;      % states:  [x; y; theta]
nu = 2;      % inputs:  [v; delta]
ny = 2;      % output yang di-track: [x; y]

nlobj = nlmpc(nx, ny, nu);

Ts = 0.1;
nlobj.Ts = Ts;
nlobj.PredictionHorizon = 15;   % preview ~1.5 detik ke depan
nlobj.ControlHorizon    = 5;

% Model dinamika (persis kinematika Ackermann, TANPA slip -> ini "ideal")
nlobj.Model.StateFcn = @(x,u) [u(1)*cos(x(3)); u(1)*sin(x(3)); (u(1)/L)*tan(u(2))];
nlobj.Model.OutputFcn = @(x,u) [x(1); x(2)];
nlobj.Model.IsContinuousTime = true;

% Batasan input (SAMA dengan batas aksi SAC, biar perbandingan adil)
nlobj.MV(1).Min = 0.0;    nlobj.MV(1).Max = 1.0;     % v
nlobj.MV(2).Min = -0.52;  nlobj.MV(2).Max = 0.52;    % delta
nlobj.MV(2).RateMin = -0.3; nlobj.MV(2).RateMax = 0.3;  % batasi laju steer -> halus

% Bobot cost: kejar posisi (x,y), redam perubahan steer kasar
nlobj.Weights.OutputVariables = [1 1];
nlobj.Weights.ManipulatedVariablesRate = [0.1 0.5];

% Validasi model (WAJIB sebelum dipakai)
validateFcns(nlobj, [0;0;0], [0.3;0]);

disp('NMPC object (nlobj) siap divalidasi. Simpan ke .mat untuk dipakai di skrip loop.');
save('nlobj_baseline.mat', 'nlobj', 'L');