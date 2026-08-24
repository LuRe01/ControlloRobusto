%% NOTE TEORICHE - MU-SYNTHESIS E D-K ITERATION
% La mu-synthesis cerca un controllore robusto rispetto alla struttura
% esplicita delle incertezze. MUSYN implementa una D-K iteration alternando
% sintesi H-infinity e scalature D che approssimano la structured singular
% value mu. Il criterio e' direttamente collegato alla robust performance.
%

%% ========================================================================
% MU_SYNTHESIS
%
% D-K iteration mediante MUSYN
% ========================================================================
close all;
clc;
load('HINF_setup.mat');
load('HINF_controllers.mat');
load('ACTUATOR_LUMPED.mat');
I2 = eye(2);
nmeas = 2;
ncont = 2;

%% ========================================================================
% 1. PLANT INCERTO CON NOMI I/O
% ========================================================================
Gmu = G_uncertain_lumped_scaled;
Gmu.InputName = {
    'u1'
    'u2'
};
Gmu.OutputName = {
    'y1'
    'y2'
};

%% ========================================================================
% 2. PESI CON NOMI I/O
% ========================================================================
WS_mu = WS;
WS_mu.InputName = {
    'e1'
    'e2'
};
WS_mu.OutputName = {
    'zS1'
    'zS2'
};
WU_mu = WU;
WU_mu.InputName = {
    'u1'
    'u2'
};
WU_mu.OutputName = {
    'zU1'
    'zU2'
};
WT_mu = WT;
WT_mu.InputName = {
    'y1'
    'y2'
};
WT_mu.OutputName = {
    'zT1'
    'zT2'
};

%% ========================================================================
% 3. ERRORE
% ========================================================================
SumE1 = ...
    sumblk( ...
        'e1 = r1 - y1');
SumE2 = ...
    sumblk( ...
        'e2 = r2 - y2');

%% ========================================================================
% 4. GENERALIZED PLANT INCERTO
%
% ingressi  = [r1 r2 u1 u2]
% uscite    = [zS zU zT e]
%
% Gli ultimi 2 ingressi sono i controlli.
% Le ultime 2 uscite sono le misure al controllore.
% ========================================================================
P_mu = ...
    connect( ...
        Gmu, ...
        WS_mu, ...
        WU_mu, ...
        WT_mu, ...
        SumE1, ...
        SumE2, ...
        {'r1','r2','u1','u2'}, ...
        {'zS1','zS2', ...
         'zU1','zU2', ...
         'zT1','zT2', ...
         'e1','e2'});

%% ========================================================================
% 5. D-K ITERATION
% ========================================================================
optsMU = ...
    musynOptions( ...
    'Display','short', ...
    'MixedMU','on', ...
    'MaxIter',20, ...
    'TolPerf',1e-2);
fprintf('\n============================================================\n');
fprintf('MU-SYNTHESIS / D-K ITERATION\n');
fprintf('============================================================\n');
[K_mu_scaled,CLperf_mu,info_mu] = ...
    musyn( ...
        P_mu, ...
        nmeas, ...
        ncont, ...
        optsMU);
K_mu_scaled = ...
    minreal( ...
        ss(K_mu_scaled), ...
        1e-7);

%% ========================================================================
% 6. DESCALATURA
% ========================================================================
K_mu = ...
    minreal( ...
        Du * ...
        K_mu_scaled * ...
        Dy_inv, ...
        1e-7);

%% ========================================================================
% 7. VERIFICA NOMINALE
% ========================================================================
Lmu = G_scaled*K_mu_scaled;
Smu = feedback(I2,Lmu);
Tmu = feedback(Lmu,I2);
CLnom_mu = [
    WS*Smu
    WU*K_mu_scaled*Smu
    WT*Tmu
];
gammaNom_mu = ...
    hinfnorm( ...
        minreal(CLnom_mu,1e-7));
fprintf('\nPerformance robusta restituita da musyn = %.6f\n', ...
    CLperf_mu);
fprintf('Gamma nominale K_mu = %.6f\n', ...
    gammaNom_mu);
fprintf('Ordine K_mu = %d\n', ...
    order(K_mu_scaled));
fprintf('Max Re(polo nominale) = %.6e\n', ...
    max(real(pole(Tmu))));
if CLperf_mu > 1
    warning([ ...
        'La robust performance non e'' < 1. ', ...
        'Non cambiare subito il modello: ', ...
        'provare un rilassamento motivato dei pesi.']);
end

%% ========================================================================
% 8. SALVATAGGIO
% ========================================================================
save( ...
    'MU_controller.mat', ...
    'K_mu_scaled', ...
    'K_mu', ...
    'CLperf_mu', ...
    'gammaNom_mu', ...
    'info_mu', ...
    'P_mu');
fprintf('\nMU_controller.mat creato correttamente.\n');
