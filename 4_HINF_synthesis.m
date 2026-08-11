%% HINF_01_SYNTHESIS
%
% Progettazione di:
%
%   1. controllore mixed-sensitivity con mixsyn;
%   2. controllore equivalente con augw + hinfsyn;
%   3. PID diagonale ottimizzato con hinfstruct.
%
% Tutti i controllori vengono valutati sullo stesso obiettivo:
%
%      || WS*S  ||
%      || WU*KS ||_inf
%      || WT*T  ||

close all;
clc;

load('HINF_setup.mat');

nmeas = 2;
ncont = 2;

optsHinf = hinfsynOptions( ...
    'Display','on');

%% ========================================================================
%  1. MIXSYN
% ========================================================================

fprintf('\n============================================================\n');
fprintf('MIXED-SENSITIVITY CON MIXSYN\n');
fprintf('============================================================\n');

[K_mix,CL_mix,gamma_mix,info_mix] = ...
    mixsyn( ...
        G_nominal, ...
        WS, ...
        WU, ...
        WT, ...
        optsHinf);

K_mix = minreal( ...
    ss(K_mix), ...
    1e-7);

fprintf('Gamma mixsyn: %.8f\n',gamma_mix);
fprintf('Ordine K_mix: %d\n',order(K_mix));

%% ========================================================================
%  2. HINFSYN SUL PLANT AUGMENTATO
% ========================================================================

fprintf('\n============================================================\n');
fprintf('MIXED-SENSITIVITY CON AUGW + HINFSYN\n');
fprintf('============================================================\n');

[K_hinfsyn,CL_hinfsyn,gamma_hinfsyn,info_hinfsyn] = ...
    hinfsyn( ...
        P_mix, ...
        nmeas, ...
        ncont, ...
        optsHinf);

K_hinfsyn = minreal( ...
    ss(K_hinfsyn), ...
    1e-7);

fprintf('Gamma hinfsyn: %.8f\n',gamma_hinfsyn);
fprintf('Ordine K_hinfsyn: %d\n',order(K_hinfsyn));

fprintf('\nDifferenza relativa tra i gamma:\n');

fprintf('|gamma_mix-gamma_hinfsyn|/gamma_mix = %.6e\n', ...
    abs(gamma_mix-gamma_hinfsyn)/gamma_mix);

%% ========================================================================
%  3. HINFSTRUCT CON DUE PID DIAGONALI
%
%      [delta_F1_cmd]   [Kalpha  0    ] [e_alpha]
%      [delta_F2_cmd] = [0       Kbeta] [e_beta ]
% ========================================================================

fprintf('\n============================================================\n');
fprintf('HINFSTRUCT CON PID DIAGONALI\n');
fprintf('============================================================\n');

Kalpha = tunablePID( ...
    'Kalpha', ...
    'PID');

Kbeta = tunablePID( ...
    'Kbeta', ...
    'PID');

% Costanti di filtro derivative strettamente positive

Kalpha.Tf.Minimum = 1e-4;
Kbeta.Tf.Minimum  = 1e-4;

KpidTunable = blkdiag( ...
    Kalpha, ...
    Kbeta);

%% Funzioni di sensibilità generalizzate

I2 = eye(2);

LpidTunable = ...
    G_nominal*KpidTunable;

SpidTunable = ...
    feedback(I2,LpidTunable);

TpidTunable = ...
    feedback(LpidTunable,I2);

KSpidTunable = ...
    KpidTunable*SpidTunable;

%% Obiettivo pesato

CLpidTunable = [
    WS*SpidTunable
    WU*KSpidTunable
    WT*TpidTunable
];

optsHinfStruct = hinfstructOptions( ...
    'Display','final', ...
    'RandomStart',20);

[CLpidTuned,gamma_pid,info_pid] = ...
    hinfstruct( ...
        CLpidTunable, ...
        optsHinfStruct);

%% Estrazione dei PID ottimizzati

Kalpha_tuned = ...
    getBlockValue(CLpidTuned,'Kalpha');

Kbeta_tuned = ...
    getBlockValue(CLpidTuned,'Kbeta');

K_pid = blkdiag( ...
    ss(Kalpha_tuned), ...
    ss(Kbeta_tuned));

K_pid = minreal(K_pid,1e-7);

fprintf('Gamma PID hinfstruct: %.8f\n',gamma_pid);
fprintf('Ordine K_pid: %d\n',order(K_pid));

disp('PID pitch ottimizzato:');
disp(Kalpha_tuned);

disp('PID yaw ottimizzato:');
disp(Kbeta_tuned);

%% ========================================================================
%  4. VERIFICA NOMINALE IMMEDIATA
% ========================================================================

controllers = {
    K_mix
    K_hinfsyn
    K_pid
};

controllerNames = {
    'mixsyn'
    'hinfsyn'
    'hinfstruct PID'
};

for k = 1:numel(controllers)

    K = controllers{k};

    L = G_nominal*K;
    T = feedback(L,eye(2));

    maxRealPole = max(real(pole(T)));

    fprintf('\n%-20s: max Re(polo) = %.8e\n', ...
        controllerNames{k}, ...
        maxRealPole);

    if maxRealPole >= 0
        warning('%s non stabilizza nominalmente il plant.', ...
            controllerNames{k});
    end
end

%% ========================================================================
%  5. SALVATAGGIO
% ========================================================================

save('HINF_controllers.mat', ...
    'K_mix', ...
    'K_hinfsyn', ...
    'K_pid', ...
    'Kalpha_tuned', ...
    'Kbeta_tuned', ...
    'gamma_mix', ...
    'gamma_hinfsyn', ...
    'gamma_pid', ...
    'info_mix', ...
    'info_hinfsyn', ...
    'info_pid');

fprintf('\nHINF_controllers.mat creato correttamente.\n');