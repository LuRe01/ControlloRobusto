%% PARAMETER_SCREENING.m

close all;
clc;

load('HINF_setup.mat');
load('HINF_controllers.mat');

I2 = eye(2);

%% Usiamo mixsyn come controllore nominale di riferimento

Kscreen = K_mix_scaled;

Lunc = G_uncertain_scaled*Kscreen;
Tunc = feedback(Lunc,I2);

opts = robOptions( ...
    'Sensitivity','on');

[stabMargin,wcu,infoRS] = ...
    robstab(Tunc,opts);

fprintf('\n============================================================\n');
fprintf('SCREENING DELLE INCERTEZZE\n');
fprintf('============================================================\n');

fprintf('Robust stability margin = [%.6f, %.6f]\n', ...
    stabMargin.LowerBound, ...
    stabMargin.UpperBound);

fprintf('\nSensibilita'' del margine alle incertezze:\n');

disp(infoRS.Sensitivity);

fprintf('\nWorst-case uncertainty:\n');
disp(wcu);