%% HINF_02_ANALYSIS
%
% Analisi completa dei controllori H-infinity:
%
%   NS = Nominal Stability
%   NP = Nominal Performance
%   RS = Robust Stability
%   RP = Robust Performance
%
% Strumenti:
%
%   hinfnorm
%   mussv
%   robstab
%   robgain
%   wcgain
%   usample
%   usubs

close all;
clc;

load('HINF_setup.mat');
load('HINF_controllers.mat');

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

nControllers = numel(controllers);

I2 = eye(2);

%% ========================================================================
%  1. PREALLOCAZIONE RISULTATI
% ========================================================================

controllerOrder = zeros(nControllers,1);

maxRealPoleNominal = zeros(nControllers,1);

gammaNominal = zeros(nControllers,1);

muRSpeakUpper = zeros(nControllers,1);
muRSpeakLower = zeros(nControllers,1);

muRPpeakUpper = zeros(nControllers,1);
muRPpeakLower = zeros(nControllers,1);

robustStabilityLower = zeros(nControllers,1);
robustStabilityUpper = zeros(nControllers,1);

worstCaseGainLower = zeros(nControllers,1);
worstCaseGainUpper = zeros(nControllers,1);

NS_pass = false(nControllers,1);
NP_pass = false(nControllers,1);
RS_pass = false(nControllers,1);
RP_pass = false(nControllers,1);

%% Dati per i grafici

S_all = cell(nControllers,1);
T_all = cell(nControllers,1);
KS_all = cell(nControllers,1);
GS_all = cell(nControllers,1);

muNP_all = cell(nControllers,1);

muRSupper_all = cell(nControllers,1);
muRSlower_all = cell(nControllers,1);

muRPupper_all = cell(nControllers,1);
muRPlower_all = cell(nControllers,1);

wcuRS_all = cell(nControllers,1);
wcuRP_all = cell(nControllers,1);

%% ========================================================================
%  2. CICLO SUI CONTROLLORI
% ========================================================================

for k = 1:nControllers

    K = controllers{k};

    fprintf('\n\n');
    fprintf('############################################################\n');
    fprintf('CONTROLLORE: %s\n',upper(controllerNames{k}));
    fprintf('############################################################\n');

    controllerOrder(k) = order(K);

    %% ====================================================================
    %  2.1 FUNZIONI DI SENSIBILITA' NOMINALI
    % =====================================================================

    L = G_nominal*K;

    S = feedback(I2,L);
    T = feedback(L,I2);

    KS = K*S;
    GS = S*G_nominal;

    S_all{k} = minreal(S,1e-7);
    T_all{k} = minreal(T,1e-7);
    KS_all{k} = minreal(KS,1e-7);
    GS_all{k} = minreal(GS,1e-7);

    WeightedNominalCL = [
        WS*S
        WU*KS
        WT*T
    ];

    %% ====================================================================
    %  2.2 NOMINAL STABILITY
    % =====================================================================

    nominalPoles = pole(T);

    maxRealPoleNominal(k) = ...
        max(real(nominalPoles));

    NS_pass(k) = ...
        all(real(nominalPoles)<0);

    fprintf('\n=== NOMINAL STABILITY ===\n');

    fprintf('Massima parte reale dei poli: %.8e\n', ...
        maxRealPoleNominal(k));

    fprintf('NS: %s\n', ...
        passText(NS_pass(k)));

    %% ====================================================================
    %  2.3 NOMINAL PERFORMANCE
    % =====================================================================

    [gammaNominal(k),wPeakNominal] = ...
        hinfnorm(minreal(WeightedNominalCL,1e-7));

    NP_pass(k) = ...
        gammaNominal(k)<1;

    fprintf('\n=== NOMINAL PERFORMANCE ===\n');

    fprintf('||[WS*S; WU*KS; WT*T]||inf = %.6f\n', ...
        gammaNominal(k));

    fprintf('Frequenza critica: %.6f rad/s\n', ...
        wPeakNominal);

    fprintf('NP: %s\n', ...
        passText(NP_pass(k)));

    %% Profilo della performance nominale

    WeightedNominalFRD = ...
        frd(WeightedNominalCL,omegaHinf);

    svNominal = sigma( ...
        WeightedNominalFRD, ...
        omegaHinf);

    muNPprofile = squeeze(max(svNominal,[],1));

    muNP_all{k} = muNPprofile;

    %% ====================================================================
    %  2.4 CLOSED LOOP INCERTO
    % =====================================================================

    Luncertain = G_uncertain*K;

    Suncertain = feedback(I2,Luncertain);
    Tuncertain = feedback(Luncertain,I2);

    KSuncertain = K*Suncertain;

    WeightedUncertainCL = [
        WS*Suncertain
        WU*KSuncertain
        WT*Tuncertain
    ];

    %% ====================================================================
    %  2.5 ROBUST STABILITY CON ROBSTAB
    % =====================================================================

    robOpts = robOptions( ...
        'Sensitivity','on');

    [stabMargin,wcuRS,infoRS] = ...
        robstab( ...
            Tuncertain, ...
            robOpts);

    robustStabilityLower(k) = ...
        stabMargin.LowerBound;

    robustStabilityUpper(k) = ...
        stabMargin.UpperBound;

    RS_pass(k) = ...
        stabMargin.LowerBound > 1;

    wcuRS_all{k} = wcuRS;

    fprintf('\n=== ROBUST STABILITY: ROBSTAB ===\n');

    fprintf('Margine RS: [%.6f, %.6f]\n', ...
        stabMargin.LowerBound, ...
        stabMargin.UpperBound);

    fprintf('RS: %s\n', ...
        passText(RS_pass(k)));

    fprintf('Sensibilità del margine alle incertezze:\n');
    disp(infoRS.Sensitivity);

    %% ====================================================================
    %  2.6 ROBUST PERFORMANCE CON WCGAIN
    % =====================================================================

    wcOptions = wcgainOptions( ...
        'MaxOverFrequency','on', ...
        'RelTol',0.05);

    [worstCaseGain,wcuRP,infoWCGain] = ...
        wcgain( ...
            WeightedUncertainCL, ...
            wcOptions);

    worstCaseGainLower(k) = ...
        worstCaseGain.LowerBound;

    worstCaseGainUpper(k) = ...
        worstCaseGain.UpperBound;

    RP_pass(k) = ...
        worstCaseGain.UpperBound < 1;

    wcuRP_all{k} = wcuRP;

    fprintf('\n=== ROBUST PERFORMANCE: WCGAIN ===\n');

    fprintf('Worst-case gain: [%.6f, %.6f]\n', ...
        worstCaseGain.LowerBound, ...
        worstCaseGain.UpperBound);

    fprintf('RP: %s\n', ...
        passText(RP_pass(k)));

    %% Controllo equivalente mediante robgain con gamma = 1

    [performanceMargin,~,~] = ...
        robgain( ...
            WeightedUncertainCL, ...
            1);

    fprintf('Margine robgain gamma=1: [%.6f, %.6f]\n', ...
        performanceMargin.LowerBound, ...
        performanceMargin.UpperBound);

    %% ====================================================================
    %  2.7 MU-ANALYSIS ESPLICITA
    %
    %  lftdata fornisce:
    %
    %     M, Delta, BlockStructure
    %
    %  tali che:
    %
    %     WeightedUncertainCL = Fu(M,Delta)
    % =====================================================================

    [Mdelta,Delta,BlockStructure] = ...
        lftdata(WeightedUncertainCL);

    nDelta = size(Delta,1);

    %% Blocco per la robusta stabilità

    M11 = Mdelta( ...
        1:nDelta, ...
        1:nDelta);

    M11frd = frd( ...
        M11, ...
        omegaHinf);

    muRSbounds = mussv( ...
        M11frd, ...
        BlockStructure);

    muRSupper = squeeze( ...
        muRSbounds.ResponseData(1,1,:));

    muRSlower = squeeze( ...
        muRSbounds.ResponseData(1,2,:));

    muRSupper_all{k} = muRSupper;
    muRSlower_all{k} = muRSlower;

    muRSpeakUpper(k) = max(muRSupper);
    muRSpeakLower(k) = max(muRSlower);

    %% Blocco fittizio per la robusta performance
    %
    % WeightedUncertainCL ha:
    %
    %   2 ingressi esogeni;
    %   6 uscite prestazionali.
    %
    % Il blocco fittizio Delta_p ha quindi dimensione 2x6.

    nExogenous = size(WeightedUncertainCL,2);
    nPerformance = size(WeightedUncertainCL,1);

    BlockStructureRP = [
        BlockStructure
        nExogenous, nPerformance
    ];

    MdeltaFRD = frd( ...
        Mdelta, ...
        omegaHinf);

    muRPbounds = mussv( ...
        MdeltaFRD, ...
        BlockStructureRP);

    muRPupper = squeeze( ...
        muRPbounds.ResponseData(1,1,:));

    muRPlower = squeeze( ...
        muRPbounds.ResponseData(1,2,:));

    muRPupper_all{k} = muRPupper;
    muRPlower_all{k} = muRPlower;

    muRPpeakUpper(k) = max(muRPupper);
    muRPpeakLower(k) = max(muRPlower);

    fprintf('\n=== MU-ANALYSIS ===\n');

    fprintf('Peak mu_RS lower/upper: %.6f / %.6f\n', ...
        muRSpeakLower(k), ...
        muRSpeakUpper(k));

    fprintf('Peak mu_RP lower/upper: %.6f / %.6f\n', ...
        muRPpeakLower(k), ...
        muRPpeakUpper(k));

    %% ====================================================================
    %  2.8 WORST CASE
    % =====================================================================

    G_worst_RS = usubs( ...
        G_uncertain, ...
        wcuRS);

    T_worst_RS = feedback( ...
        G_worst_RS*K, ...
        I2);

    fprintf('\nPoli del worst case RS:\n');

    disp(pole(T_worst_RS));

    G_worst_RP = usubs( ...
        G_uncertain, ...
        wcuRP);

    S_worst_RP = feedback( ...
        I2, ...
        G_worst_RP*K);

    T_worst_RP = feedback( ...
        G_worst_RP*K, ...
        I2);

    KS_worst_RP = K*S_worst_RP;

    CL_worst_RP = [
        WS*S_worst_RP
        WU*KS_worst_RP
        WT*T_worst_RP
    ];

    fprintf('Norma pesata nel worst case RP: %.6f\n', ...
        norm(CL_worst_RP,inf));

    %% ====================================================================
    %  2.9 GRAFICO NOMINALE VS WORST CASE
    % =====================================================================

    tComparison = 0:0.01:10;

    figure( ...
        'Name', ...
        ['Nominale vs worst case - ',controllerNames{k}]);

    subplot(2,1,1);

    step( ...
        T(1,1)*reference.alphaStep, ...
        T_worst_RP(1,1)*reference.alphaStep, ...
        tComparison);

    grid on;
    ylabel('\delta\alpha [rad]');
    title([controllerNames{k},': tracking pitch']);
    legend('Nominale','Worst case','Location','best');

    subplot(2,1,2);

    step( ...
        T(2,2)*reference.betaStep, ...
        T_worst_RP(2,2)*reference.betaStep, ...
        tComparison);

    grid on;
    ylabel('\delta\beta [rad]');
    xlabel('Tempo [s]');
    title([controllerNames{k},': tracking yaw']);
    legend('Nominale','Worst case','Location','best');
end

%% ========================================================================
%  3. GRAFICI COMPARATIVI DI S, KS E T
% ========================================================================

figure('Name','Sensitivity S');

for k = 1:nControllers
    sigma(S_all{k},omegaHinf);
    hold on;
end

sigma(inv(WS),omegaHinf);

grid on;
title('Sensitivity S e limite W_S^{-1}');

legend( ...
    controllerNames{:}, ...
    'W_S^{-1}', ...
    'Location','best');

figure('Name','Control sensitivity KS');

for k = 1:nControllers
    sigma(KS_all{k},omegaHinf);
    hold on;
end

sigma(inv(WU),omegaHinf);

grid on;
title('Control sensitivity KS e limite W_U^{-1}');

legend( ...
    controllerNames{:}, ...
    'W_U^{-1}', ...
    'Location','best');

figure('Name','Complementary sensitivity T');

for k = 1:nControllers
    sigma(T_all{k},omegaHinf);
    hold on;
end

sigma(inv(WT),omegaHinf);

grid on;
title('Complementary sensitivity T e limite W_T^{-1}');

legend( ...
    controllerNames{:}, ...
    'W_T^{-1}', ...
    'Location','best');

%% ========================================================================
%  4. GRAFICI MU
% ========================================================================

for k = 1:nControllers

    figure( ...
        'Name', ...
        ['Mu analysis - ',controllerNames{k}]);

    semilogx( ...
        omegaHinf, ...
        muNP_all{k}, ...
        'LineWidth',1.5);

    hold on;

    semilogx( ...
        omegaHinf, ...
        muRSupper_all{k}, ...
        'LineWidth',1.5);

    semilogx( ...
        omegaHinf, ...
        muRSlower_all{k}, ...
        '--', ...
        'LineWidth',1.2);

    semilogx( ...
        omegaHinf, ...
        muRPupper_all{k}, ...
        'LineWidth',1.5);

    semilogx( ...
        omegaHinf, ...
        muRPlower_all{k}, ...
        '--', ...
        'LineWidth',1.2);

    yline(1,'k:','LineWidth',1.2);

    grid on;

    xlabel('\omega [rad/s]');
    ylabel('Valore');

    title([ ...
        controllerNames{k}, ...
        ': NP, RS e RP']);

    legend( ...
        '\sigma_{max} nominale', ...
        '\mu_{RS} upper', ...
        '\mu_{RS} lower', ...
        '\mu_{RP} upper', ...
        '\mu_{RP} lower', ...
        'Limite 1', ...
        'Location','best');
end

%% ========================================================================
%  5. TRACKING NOMINALE
% ========================================================================

tStep = 0:0.01:10;

figure('Name','Tracking pitch');

for k = 1:nControllers

    [y,t] = step( ...
        T_all{k}(:,1)*reference.alphaStep, ...
        tStep);

    alphaResponse = squeeze(y(:,1,1));

    plot( ...
        t, ...
        rad2deg(alphaResponse), ...
        'LineWidth',1.4);

    hold on;
end

yline(rad2deg(reference.alphaStep),'k:');

grid on;
xlabel('Tempo [s]');
ylabel('\delta\alpha [deg]');
title('Tracking di un gradino pitch di 2°');

legend( ...
    controllerNames{:}, ...
    'Riferimento', ...
    'Location','best');

figure('Name','Tracking yaw');

for k = 1:nControllers

    [y,t] = step( ...
        T_all{k}(:,2)*reference.betaStep, ...
        tStep);

    betaResponse = squeeze(y(:,2,1));

    plot( ...
        t, ...
        rad2deg(betaResponse), ...
        'LineWidth',1.4);

    hold on;
end

yline(rad2deg(reference.betaStep),'k:');

grid on;
xlabel('Tempo [s]');
ylabel('\delta\beta [deg]');
title('Tracking di un gradino yaw di 3°');

legend( ...
    controllerNames{:}, ...
    'Riferimento', ...
    'Location','best');

%% ========================================================================
%  6. SFORZO DI CONTROLLO
% ========================================================================

figure('Name','Control effort - riferimento pitch');

for k = 1:nControllers

    [u,t] = step( ...
        KS_all{k}(:,1)*reference.alphaStep, ...
        tStep);

    uData = squeeze(u);

    plot(t,uData,'LineWidth',1.2);
    hold on;
end

grid on;
xlabel('Tempo [s]');
ylabel('\delta F_{cmd} [N]');
title('Sforzo di controllo per riferimento pitch');

%% ========================================================================
%  7. REIEZIONE DEI DISTURBI
% ========================================================================

figure('Name','Disturbo sul primo attuatore');

for k = 1:nControllers

    [y,t] = step( ...
        GS_all{k}(:,1)*disturbance.force1, ...
        tStep);

    alphaDisturbance = squeeze(y(:,1,1));

    plot( ...
        t, ...
        rad2deg(alphaDisturbance), ...
        'LineWidth',1.4);

    hold on;
end

grid on;
xlabel('Tempo [s]');
ylabel('\delta\alpha [deg]');
title('Reiezione di un disturbo di forza di 0.05 N');

legend(controllerNames{:},'Location','best');

%% ========================================================================
%  8. MONTE CARLO SUL MODELLO LINEARIZZATO INCERTO
% ========================================================================

rng(10);

nSamples = 20;

G_samples = usample( ...
    G_uncertain, ...
    nSamples);

for k = 1:nControllers

    K = controllers{k};

    figure( ...
        'Name', ...
        ['Monte Carlo - ',controllerNames{k}]);

    hold on;

    for j = 1:nSamples

        Gsample = G_samples(:,:,j);

        Tsample = feedback( ...
            Gsample*K, ...
            I2);

        [y,t] = step( ...
            Tsample(1,1)*reference.alphaStep, ...
            tStep);

        plot( ...
            t, ...
            rad2deg(squeeze(y)), ...
            'LineWidth',0.7);
    end

    yline(rad2deg(reference.alphaStep),'k--');

    grid on;
    xlabel('Tempo [s]');
    ylabel('\delta\alpha [deg]');

    title([ ...
        controllerNames{k}, ...
        ': campioni incerti']);
end

%% ========================================================================
%  9. TABELLA RIASSUNTIVA
% ========================================================================

results = table( ...
    controllerNames, ...
    controllerOrder, ...
    maxRealPoleNominal, ...
    gammaNominal, ...
    muRSpeakLower, ...
    muRSpeakUpper, ...
    muRPpeakLower, ...
    muRPpeakUpper, ...
    robustStabilityLower, ...
    robustStabilityUpper, ...
    worstCaseGainLower, ...
    worstCaseGainUpper, ...
    NS_pass, ...
    NP_pass, ...
    RS_pass, ...
    RP_pass, ...
    'VariableNames',{
        'Controller'
        'Order'
        'MaxRealNominalPole'
        'NominalGamma'
        'MuRSLower'
        'MuRSUpper'
        'MuRPLower'
        'MuRPUpper'
        'RobustStabilityLower'
        'RobustStabilityUpper'
        'WorstCaseGainLower'
        'WorstCaseGainUpper'
        'NS'
        'NP'
        'RS'
        'RP'
    });

disp(results);

writetable( ...
    results, ...
    'HINF_analysis_results.csv');

save('HINF_analysis_results.mat', ...
    'results', ...
    'S_all', ...
    'T_all', ...
    'KS_all', ...
    'GS_all', ...
    'muNP_all', ...
    'muRSupper_all', ...
    'muRSlower_all', ...
    'muRPupper_all', ...
    'muRPlower_all', ...
    'wcuRS_all', ...
    'wcuRP_all');

%% ========================================================================
%  FUNZIONE LOCALE
% ========================================================================

function text = passText(condition)

    if condition
        text = 'PASS';
    else
        text = 'FAIL';
    end

end