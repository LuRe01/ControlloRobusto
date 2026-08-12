%% ========================================================================
% MU_ANALYSIS_COMPARISON
%
% Confronto H-infinity vs mu-synthesis sullo STESSO modello incerto.
% ========================================================================

close all;
clc;

load('HINF_setup.mat');
load('HINF_controllers.mat');
load('ACTUATOR_LUMPED.mat');
load('MU_controller.mat');

I2 = eye(2);

controllers = {
    K_mix_scaled
    K_hinfsyn_scaled
    K_mu_scaled
};

controllerNames = {
    'mixsyn'
    'hinfsyn'
    'mu-synthesis'
};

Nc = numel(controllers);

NominalGamma = zeros(Nc,1);

RS_Lower = zeros(Nc,1);
RS_Upper = zeros(Nc,1);

RP_Lower = zeros(Nc,1);
RP_Upper = zeros(Nc,1);

MuRS_Lower = zeros(Nc,1);
MuRS_Upper = zeros(Nc,1);

MuRP_Lower = zeros(Nc,1);
MuRP_Upper = zeros(Nc,1);

NS = false(Nc,1);
NP = false(Nc,1);
RS = false(Nc,1);
RP = false(Nc,1);

for k = 1:Nc

    K = controllers{k};

    %% Nominale

    Lnom = G_scaled*K;

    Snom = feedback(I2,Lnom);
    Tnom = feedback(Lnom,I2);

    Wnom = [
        WS*Snom
        WU*K*Snom
        WT*Tnom
    ];

    NominalGamma(k) = ...
        hinfnorm(minreal(Wnom,1e-7));

    NS(k) = all(real(pole(Tnom))<0);
    NP(k) = NominalGamma(k)<1;

    %% Incerto

    Lunc = ...
        G_uncertain_lumped_scaled*K;

    Sunc = feedback(I2,Lunc);
    Tunc = feedback(Lunc,I2);

    Wunc = [
        WS*Sunc
        WU*K*Sunc
        WT*Tunc
    ];

    %% Robust stability

    stabMargin = ...
        robstab(Tunc);

    RS_Lower(k) = ...
        stabMargin.LowerBound;

    RS_Upper(k) = ...
        stabMargin.UpperBound;

    RS(k) = ...
        RS_Lower(k)>1;

    %% Robust performance

    wcGain = ...
        wcgain(Wunc);

    RP_Lower(k) = ...
        wcGain.LowerBound;

    RP_Upper(k) = ...
        wcGain.UpperBound;

    RP(k) = ...
        RP_Upper(k)<1;

    %% MU esplicita

    [Mdelta,Delta,BlockStructure] = ...
        lftdata(Wunc);

    nDelta = size(Delta,1);

    M11 = ...
        Mdelta( ...
            1:nDelta, ...
            1:nDelta);

    muRS = ...
        mussv( ...
            frd(M11,omegaHinf), ...
            BlockStructure);

    muRSup = ...
        squeeze(muRS.ResponseData(1,1,:));

    muRSlo = ...
        squeeze(muRS.ResponseData(1,2,:));

    MuRS_Upper(k) = max(muRSup);
    MuRS_Lower(k) = max(muRSlo);

    %% RP = Delta strutturale + Delta prestazionale

    nExogenous  = size(Wunc,2);
    nPerformance = size(Wunc,1);

    BlockStructureRP = [
        BlockStructure
        nExogenous,nPerformance
    ];

    muRP = ...
        mussv( ...
            frd(Mdelta,omegaHinf), ...
            BlockStructureRP);

    muRPup = ...
        squeeze(muRP.ResponseData(1,1,:));

    muRPlo = ...
        squeeze(muRP.ResponseData(1,2,:));

    MuRP_Upper(k) = max(muRPup);
    MuRP_Lower(k) = max(muRPlo);

end

resultsMU = table( ...
    controllerNames, ...
    NominalGamma, ...
    RS_Lower, ...
    RS_Upper, ...
    RP_Lower, ...
    RP_Upper, ...
    MuRS_Lower, ...
    MuRS_Upper, ...
    MuRP_Lower, ...
    MuRP_Upper, ...
    NS, ...
    NP, ...
    RS, ...
    RP, ...
    'VariableNames',{
        'Controller'
        'NominalGamma'
        'RS_Lower'
        'RS_Upper'
        'RP_Lower'
        'RP_Upper'
        'MuRS_Lower'
        'MuRS_Upper'
        'MuRP_Lower'
        'MuRP_Upper'
        'NS'
        'NP'
        'RS'
        'RP'
    });

fprintf('\n============================================================\n');
fprintf('CONFRONTO HINF vs MU-SYNTHESIS\n');
fprintf('============================================================\n');

disp(resultsMU);

writetable( ...
    resultsMU, ...
    'MU_vs_HINF_results.csv');