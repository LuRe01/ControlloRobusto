%% NOTE TEORICHE - CONFRONTO H-INFINITY / MU
% Confronta i controllori H-infinity e mu-synthesis sullo stesso modello
% incerto. robstab fornisce margini di stabilita' robusta, wcgain stima il
% worst-case gain e mussv calcola bounds della structured singular value.
% Il confronto e' significativo solo mantenendo identici plant, pesi e
% struttura di incertezza.
%

%% ========================================================================
% MU_ANALYSIS_COMPARISON
%
% Confronto H-infinity vs mu-synthesis sullo STESSO modello incerto.
%
% Strategia:
%
% - robstab: analisi automatica robust stability
%
% - wcgain:
%       1) prova prima il metodo standard, più veloce
%       2) se compare l'errore "Invalid MU upper bound",
%          ripete automaticamente usando MussvOptions = 'a'
%
% - mussv esplicita:
%       analisi sulla griglia COMPLETA omegaHinf
%
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

%% ========================================================================
% GRIGLIA COMPLETA PER L'ANALISI MU ESPLICITA
% ========================================================================
omegaAnalysis = omegaHinf;
fprintf('\n============================================================\n');
fprintf('MU ANALYSIS COMPARISON\n');
fprintf('Griglia completa: %d frequenze\n',numel(omegaAnalysis));
fprintf('Intervallo: %.3e - %.3e rad/s\n', ...
    min(omegaAnalysis),max(omegaAnalysis));
fprintf('============================================================\n');

%% ========================================================================
% OPZIONI WCGAIN
%
% FAST:
% utilizza le impostazioni standard di wcgain.
%
% ACCURATE:
% viene utilizzata SOLO se il metodo standard fallisce per il problema
% dell'upper bound di mu.
% ========================================================================
wcOptsFast = wcOptions;
wcOptsAccurate = wcOptions( ...
    'MussvOptions','a');

%% ========================================================================
% PREALLOCAZIONE
% ========================================================================
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

%% ========================================================================
% ANALISI DEI CONTROLLORI
% ========================================================================
for k = 1:Nc
    K = controllers{k};
    fprintf('\n============================================================\n');
    fprintf('Controller %d/%d: %s\n', ...
        k,Nc,controllerNames{k});
    fprintf('============================================================\n');

    %% ====================================================================
    % NOMINALE
    % =====================================================================
    fprintf('Nominal analysis...\n');
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
    NS(k) = ...
        all(real(pole(Tnom))<0);
    NP(k) = ...
        NominalGamma(k)<1;
    fprintf('Nominal analysis completed.\n');

    %% ====================================================================
    % SISTEMA INCERTO
    % =====================================================================
    Lunc = ...
        G_uncertain_lumped_scaled*K;
    Sunc = feedback(I2,Lunc);
    Tunc = feedback(Lunc,I2);
    Wunc = [
        WS*Sunc
        WU*K*Sunc
        WT*Tunc
    ];

    %% ====================================================================
    % ROBUST STABILITY
    % =====================================================================
    fprintf('Running robstab...\n');
    tic;
    stabMargin = ...
        robstab(Tunc);
    elapsedRobstab = toc;
    RS_Lower(k) = ...
        stabMargin.LowerBound;
    RS_Upper(k) = ...
        stabMargin.UpperBound;
    RS(k) = ...
        RS_Lower(k)>1;
    fprintf('robstab completed in %.2f s.\n', ...
        elapsedRobstab);

    %% ====================================================================
    % ROBUST PERFORMANCE - WCGAIN
    %
    % Prima viene provato wcgain standard.
    %
    % Se il metodo standard genera:
    %
    % "Invalid MU upper bound"
    %
    % viene effettuato automaticamente un secondo tentativo con
    % MussvOptions = 'a'.
    %
    % IMPORTANTE:
    % wcgain viene lasciato lavorare direttamente sul modello USS,
    % senza forzarlo sulla griglia omegaHinf.
    % =====================================================================
    fprintf('Running wcgain (standard method)...\n');
    tic;
    try

        %% Tentativo veloce
        wcGain = ...
            wcgain( ...
                Wunc, ...
                wcOptsFast);
        fprintf('wcgain standard method successful.\n');
    catch ME

        %% ================================================================
        % FALLBACK
        % ================================================================
        if contains( ...
                ME.message, ...
                'Invalid MU upper bound', ...
                'IgnoreCase',true)
            fprintf('\n');
            fprintf(['Standard wcgain failed because of an invalid ' ...
                     'MU upper bound.\n']);
            fprintf(['Retrying wcgain with ' ...
                     'MussvOptions = ''a''...\n']);
            wcGain = ...
                wcgain( ...
                    Wunc, ...
                    wcOptsAccurate);
            fprintf('wcgain accurate method successful.\n');
        else
            % Se l'errore NON è quello dell'upper bound di mu,
            % non viene nascosto.
            rethrow(ME);
        end
    end
    elapsedWcgain = toc;
    fprintf('wcgain completed in %.2f s.\n', ...
        elapsedWcgain);
    RP_Lower(k) = ...
        wcGain.LowerBound;
    RP_Upper(k) = ...
        wcGain.UpperBound;
    RP(k) = ...
        RP_Upper(k)<1;

    %% ====================================================================
    % DECOMPOSIZIONE LFT
    % =====================================================================
    fprintf('Extracting LFT data...\n');
    [Mdelta,Delta,BlockStructure] = ...
        lftdata(Wunc);

    %% ====================================================================
    % ROBUST STABILITY VIA MU ESPLICITA
    %
    % Costruzione del blocco M11 associato all'incertezza fisica.
    %
    % Se:
    %
    % Delta : nDeltaOut x nDeltaIn
    %
    % allora:
    %
    % M11 : nDeltaIn x nDeltaOut
    % =====================================================================
    szDelta = size(Delta);
    M11 = ...
        Mdelta( ...
            1:szDelta(2), ...
            1:szDelta(1));
    fprintf( ...
        'Running explicit mu RS on %d frequencies...\n', ...
        numel(omegaAnalysis));
    tic;
    muRS = ...
        mussv( ...
            frd(M11,omegaAnalysis), ...
            BlockStructure, ...
            's');
    elapsedMuRS = toc;
    fprintf('mu RS completed in %.2f s.\n', ...
        elapsedMuRS);
    muRSup = ...
        squeeze( ...
            muRS.ResponseData(1,1,:));
    muRSlo = ...
        squeeze( ...
            muRS.ResponseData(1,2,:));
    MuRS_Upper(k) = ...
        max(muRSup);
    MuRS_Lower(k) = ...
        max(muRSlo);

    %% ====================================================================
    % ROBUST PERFORMANCE VIA MU ESPLICITA
    %
    % Delta_aug =
    %
    %       [ Delta       0      ]
    %       [   0     DeltaPerf  ]
    %
    %
    % DeltaPerf rappresenta il blocco prestazionale fittizio.
    % =====================================================================
    nExogenous = ...
        size(Wunc,2);
    nPerformance = ...
        size(Wunc,1);

    %% Copia della struttura fisica
    BlockStructureRP = ...
        BlockStructure;

    %% Aggiunta blocco prestazionale
    perfBlock = ...
        BlockStructure(1);
    perfBlock.Name = ...
        'DeltaPerf';
    perfBlock.Size = ...
        [nExogenous nPerformance];
    perfBlock.Type = ...
        'ultidyn';
    perfBlock.Occurrences = ...
        1;
    perfBlock.Simplify = ...
        0;
    BlockStructureRP(end+1,1) = ...
        perfBlock;

    %% ====================================================================
    % MU ROBUST PERFORMANCE
    %
    % Analisi sulla griglia COMPLETA omegaHinf.
    % =====================================================================
    fprintf( ...
        'Running explicit mu RP on %d frequencies...\n', ...
        numel(omegaAnalysis));
    tic;
    muRP = ...
        mussv( ...
            frd(Mdelta,omegaAnalysis), ...
            BlockStructureRP, ...
            's');
    elapsedMuRP = toc;
    fprintf('mu RP completed in %.2f s.\n', ...
        elapsedMuRP);
    muRPup = ...
        squeeze( ...
            muRP.ResponseData(1,1,:));
    muRPlo = ...
        squeeze( ...
            muRP.ResponseData(1,2,:));
    MuRP_Upper(k) = ...
        max(muRPup);
    MuRP_Lower(k) = ...
        max(muRPlo);

    %% ====================================================================
    % FINE CONTROLLER
    % =====================================================================
    [gWS_nom,wWS_nom] = hinfnorm(minreal(WS*Snom,1e-7));
    [gWU_nom,wWU_nom] = hinfnorm(minreal(WU*K*Snom,1e-7));
    [gWT_nom,wWT_nom] = hinfnorm(minreal(WT*Tnom,1e-7));
    fprintf('\nNominal weighted channels:\n');
    fprintf('WS*S  = %.6f at %.6f rad/s\n',gWS_nom,wWS_nom);
    fprintf('WU*KS = %.6f at %.6f rad/s\n',gWU_nom,wWU_nom);
    fprintf('WT*T  = %.6f at %.6f rad/s\n',gWT_nom,wWT_nom);
    fprintf('\nController %s completed.\n',controllerNames{k});
    fprintf( ...
        'Nominal gamma : %.6f\n', ...
        NominalGamma(k));
    fprintf( ...
        'RS margin     : [%.6f, %.6f]\n', ...
        RS_Lower(k), ...
        RS_Upper(k));
    fprintf( ...
        'WC gain       : [%.6f, %.6f]\n', ...
        RP_Lower(k), ...
        RP_Upper(k));
    fprintf( ...
        'mu RS         : [%.6f, %.6f]\n', ...
        MuRS_Lower(k), ...
        MuRS_Upper(k));
    fprintf( ...
        'mu RP         : [%.6f, %.6f]\n', ...
        MuRP_Lower(k), ...
        MuRP_Upper(k));
end

%% ========================================================================
% TABELLA RISULTATI
% ========================================================================
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

%% ========================================================================
% STAMPA RISULTATI
% ========================================================================
fprintf('\n============================================================\n');
fprintf('CONFRONTO HINF vs MU-SYNTHESIS\n');
fprintf('============================================================\n');
disp(resultsMU);

%% ========================================================================
% SALVATAGGIO
% ========================================================================
writetable( ...
    resultsMU, ...
    'MU_vs_HINF_results.csv');
