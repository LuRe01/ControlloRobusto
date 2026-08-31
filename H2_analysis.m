%% NOTE TEORICHE - ANALISI DEL CONTROLLORE H2
% Valuta il controllore H2 sul plant nominale e lo esprime anche nelle
% metriche utilizzate per i controllori H-infinity. Questo permette di
% distinguere l'obiettivo di progetto (norma H2, prestazione media/RMS) dalla
% robustezza worst-case, che viene verificata separatamente negli script
% HINF_analysis.m e MU_analysis_comparison.m.

close all;
clc;

load('HINF_setup.mat');
load('H2_controller.mat');

I2 = eye(2);

%% ========================================================================
% 1. CLOSED LOOP NOMINALE
% =========================================================================
Ls = G_scaled*K_h2_scaled;
Ss = feedback(I2,Ls);
Ts = feedback(Ls,I2);
KSs = K_h2_scaled*Ss;

Lp = G_nominal*K_h2;
Sp = feedback(I2,Lp);
Tp = feedback(Lp,I2);
KSp = K_h2*Sp;

maxRealPole = max(real(pole(Ts)));
nominalStable = maxRealPole < 0;

%% ========================================================================
% 2. METRICHE H2 E H-INFINITY
% =========================================================================
h2Objective = norm(CL_h2_check,2);
[gWS,wWS] = hinfnorm(minreal(WS*Ss,1e-7));
[gWU,wWU] = hinfnorm(minreal(WU*KSs,1e-7));
[gWT,wWT] = hinfnorm(minreal(WT*Ts,1e-7));
[gHinf,wHinf] = hinfnorm(minreal([WS*Ss;WU*KSs;WT*Ts],1e-7));

%% ========================================================================
% 3. PRESTAZIONI TEMPORALI
% =========================================================================
t = 0:0.002:12;

pitch = squeeze(step(Tp(:,1)*reference.alphaStep,t));
yaw   = squeeze(step(Tp(:,2)*reference.betaStep,t));

pitchFinal = dcgain(Tp(1,1))*reference.alphaStep;
yawFinal   = dcgain(Tp(2,2))*reference.betaStep;

infoPitch = stepinfo(pitch(:,1),t,pitchFinal, ...
    'SettlingTimeThreshold',0.02);
infoYaw = stepinfo(yaw(:,2),t,yawFinal, ...
    'SettlingTimeThreshold',0.02);

pitchSSerror = 100*abs(reference.alphaStep-pitchFinal)/abs(reference.alphaStep);
yawSSerror   = 100*abs(reference.betaStep-yawFinal)/abs(reference.betaStep);

%% ========================================================================
% 4. FIGURE
% =========================================================================
figure('Name','H2 - Sensitivity functions');

% Per ciascuna funzione MIMO vengono mostrati entrambi i valori singolari.
% Il colore identifica la funzione; lo stile di linea distingue
% sigma_max (continua) da sigma_min (tratteggiata).
functionColors = lines(3);
svS  = squeeze(sigma(Ss,omegaHinf));
svT  = squeeze(sigma(Ts,omegaHinf));
svKS = squeeze(sigma(KSs,omegaHinf));

% sigma restituisce [nSingularValues x nFrequency] per sistemi MIMO.
% Il controllo seguente rende il codice robusto anche a vettori trasposti.
if size(svS,2) ~= numel(omegaHinf),  svS  = svS.';  end
if size(svT,2) ~= numel(omegaHinf),  svT  = svT.';  end
if size(svKS,2) ~= numel(omegaHinf), svKS = svKS.'; end

semilogx(omegaHinf,20*log10(svS(1,:)),'-', ...
    'Color',functionColors(1,:),'LineWidth',1.4); hold on;
semilogx(omegaHinf,20*log10(svS(end,:)),'--', ...
    'Color',functionColors(1,:),'LineWidth',1.4);
semilogx(omegaHinf,20*log10(svT(1,:)),'-', ...
    'Color',functionColors(2,:),'LineWidth',1.4);
semilogx(omegaHinf,20*log10(svT(end,:)),'--', ...
    'Color',functionColors(2,:),'LineWidth',1.4);
semilogx(omegaHinf,20*log10(svKS(1,:)),'-', ...
    'Color',functionColors(3,:),'LineWidth',1.4);
semilogx(omegaHinf,20*log10(svKS(end,:)),'--', ...
    'Color',functionColors(3,:),'LineWidth',1.4);

grid on;
xlabel('$\omega$ [rad/s]','Interpreter','latex');
ylabel('Singular values [dB]');
title('Controllore H$_2$: $S(j\omega)$, $T(j\omega)$ e $K(j\omega)S(j\omega)$', ...
    'Interpreter','latex');
legend({'$\bar{\sigma}(S)$','$\underline{\sigma}(S)$', ...
        '$\bar{\sigma}(T)$','$\underline{\sigma}(T)$', ...
        '$\bar{\sigma}(KS)$','$\underline{\sigma}(KS)$'}, ...
    'Interpreter','latex','Location','best');

figure('Name','H2 - Nominal tracking');
tiledlayout(2,1,'TileSpacing','compact','Padding','compact');
nexttile;
plot(t,rad2deg(pitch(:,1)),'LineWidth',1.5); hold on;
yline(rad2deg(reference.alphaStep),'--'); grid on;
ylabel('$\Delta\alpha$ [deg]','Interpreter','latex');
title('Inseguimento nominale del riferimento di pitch - H$_2$', ...
    'Interpreter','latex');
nexttile;
plot(t,rad2deg(yaw(:,2)),'LineWidth',1.5); hold on;
yline(rad2deg(reference.betaStep),'--'); grid on;
xlabel('$t$ [s]','Interpreter','latex');
ylabel('$\Delta\beta$ [deg]','Interpreter','latex');
title('Inseguimento nominale del riferimento di yaw - H$_2$', ...
    'Interpreter','latex');

%% ========================================================================
% 5. TABELLA
% =========================================================================
H2AnalysisResults = table( ...
    string('H2'),order(K_h2_scaled),maxRealPole,h2Objective, ...
    gHinf,wHinf,gWS,wWS,gWU,wWU,gWT,wWT, ...
    infoPitch.RiseTime,infoPitch.SettlingTime,infoPitch.Overshoot, ...
    infoYaw.RiseTime,infoYaw.SettlingTime,infoYaw.Overshoot, ...
    pitchSSerror,yawSSerror,nominalStable, ...
    'VariableNames',{ ...
    'Controller','Order','MaxRealPole','H2Objective', ...
    'WeightedHinfNorm','WeightedHinfPeakFrequency', ...
    'WS_S_Norm','WS_S_PeakFrequency', ...
    'WU_KS_Norm','WU_KS_PeakFrequency', ...
    'WT_T_Norm','WT_T_PeakFrequency', ...
    'PitchRiseTime','PitchSettlingTime','PitchOvershootPercent', ...
    'YawRiseTime','YawSettlingTime','YawOvershootPercent', ...
    'PitchSteadyStateErrorPercent','YawSteadyStateErrorPercent', ...
    'NominalStable'});

disp(H2AnalysisResults);
writetable(H2AnalysisResults,'H2_analysis_results.csv');
save('H2_analysis_results.mat','H2AnalysisResults','Ss','Ts','KSs','Sp','Tp','KSp');

