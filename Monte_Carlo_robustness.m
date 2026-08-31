%% NOTE TEORICHE - VALIDAZIONE MONTE CARLO
% Campiona plant appartenenti alla famiglia incerta e valuta stabilita',
% tracking, banda, accoppiamento e reiezione dei disturbi. Il Monte Carlo
% non sostituisce una certificazione mu/worst-case, ma fornisce una verifica
% numerica complementare e intuitiva delle prestazioni robuste.
%

%% ========================================================================
% MONTE_CARLO_ROBUSTNESS
%
% Punto 5 della traccia.
% 10 plant parametrici incerti:
%   - stabilita'
%   - settling time
%   - bandwidth
%   - reiezione disturbi aerodinamici
% ========================================================================
close all;
clc;
load('HINF_setup.mat');
load('HINF_controllers.mat');
load('MU_controller.mat');
load('H2_controller.mat');
assert(exist('Gd_uncertain','var') == 1, ...
    'Gd_uncertain deve essere disponibile dal modello lineare esteso.');
I2 = eye(2);
controllers = {
    K_mix
    K_hinfsyn
    K_pidcomp
    K_mu
    K_h2
};
controllerNames = {
    'mixsyn'
    'hinfsyn'
    'PID+comp'
    'mu-synthesis'
    'H2'
};
Nc = numel(controllers);
nSamples = 10;
rng(20);

%% Plant complessivo per campionare in maniera coerente
% primi 2 ingressi  = comando
% ultimi 2 ingressi = disturbi aerodinamici
Gall_uncertain = [
    G_uncertain_full, ...
    Gd_uncertain
];
Gall_samples = ...
    usample( ...
        Gall_uncertain, ...
        nSamples);
t = 0:0.005:30; % orizzonte esteso per evitare falsi NaN sul settling
resultController = strings(Nc*nSamples,1);
resultSample = zeros(Nc*nSamples,1);
Stable = false(Nc*nSamples,1);
PitchSettling = nan(Nc*nSamples,1);
YawSettling   = nan(Nc*nSamples,1);
PitchSettledWithinHorizon = false(Nc*nSamples,1);
YawSettledWithinHorizon   = false(Nc*nSamples,1);
PitchBW = nan(Nc*nSamples,1);
YawBW   = nan(Nc*nSamples,1);
PitchDistPeak = nan(Nc*nSamples,1);
YawDistPeak   = nan(Nc*nSamples,1);

%% Metriche aggiuntive di tracking e accoppiamento
PitchSSerror = nan(Nc*nSamples,1);
YawSSerror   = nan(Nc*nSamples,1);
YawFromPitchPeak = nan(Nc*nSamples,1);
PitchFromYawPeak = nan(Nc*nSamples,1);
row = 0;
for ic = 1:Nc
    K = controllers{ic};
    for is = 1:nSamples
        row = row + 1;
        resultController(row) = ...
            string(controllerNames{ic});
        resultSample(row) = is;
        Gsample = ...
            Gall_samples(:,:,is);
        Gu = ...
            Gsample(:,1:2);
        Gd = ...
            Gsample(:,3:4);
        L = Gu*K;
        S = feedback(I2,L);
        T = feedback(L,I2);
        stableNow = ...
            all(real(pole(T))<0);
        Stable(row) = stableNow;
        if ~stableNow
            continue;
        end

        %% ---------------------------------------------------------------
        % TRACKING PITCH
        % ---------------------------------------------------------------
        yPitch = ...
            step( ...
            T(:,1)*reference.alphaStep, ...
            t);
        yPitch = squeeze(yPitch);
        yssPitch = ...
            dcgain(T(1,1)) * ...
            reference.alphaStep;
        infoPitch = ...
            stepinfo( ...
            yPitch(:,1), ...
            t, ...
            yssPitch, ...
            'SettlingTimeThreshold',0.02);
        PitchSettling(row) = infoPitch.SettlingTime;
        if ~isfinite(PitchSettling(row))
            [PitchSettling(row),PitchSettledWithinHorizon(row)] = ...
                settlingTimeFromTrace(t,yPitch(:,1),yssPitch,0.02);
        else
            PitchSettledWithinHorizon(row) = true;
        end
        PitchSSerror(row) = ...
            100 * ...
            abs(reference.alphaStep-yssPitch) / ...
            abs(reference.alphaStep);
        YawFromPitchPeak(row) = ...
            max(abs(yPitch(:,2)));

        %% ---------------------------------------------------------------
        % TRACKING YAW
        % ---------------------------------------------------------------
        yYaw = ...
            step( ...
            T(:,2)*reference.betaStep, ...
            t);
        yYaw = squeeze(yYaw);
        yssYaw = ...
            dcgain(T(2,2)) * ...
            reference.betaStep;
        infoYaw = ...
            stepinfo( ...
            yYaw(:,2), ...
            t, ...
            yssYaw, ...
            'SettlingTimeThreshold',0.02);
        YawSettling(row) = infoYaw.SettlingTime;
        if ~isfinite(YawSettling(row))
            [YawSettling(row),YawSettledWithinHorizon(row)] = ...
                settlingTimeFromTrace(t,yYaw(:,2),yssYaw,0.02);
        else
            YawSettledWithinHorizon(row) = true;
        end
        YawSSerror(row) = ...
            100 * ...
            abs(reference.betaStep-yssYaw) / ...
            abs(reference.betaStep);
        PitchFromYawPeak(row) = ...
            max(abs(yYaw(:,1)));

        %% ---------------------------------------------------------------
        % BANDWIDTH
        % ---------------------------------------------------------------
        PitchBW(row) = ...
            bandwidth(T(1,1));
        YawBW(row) = ...
            bandwidth(T(2,2));

        %% ---------------------------------------------------------------
        % DISTURBI AERODINAMICI
        % ---------------------------------------------------------------
        Td = S*Gd;
        yDalpha = ...
            step( ...
                Td(:,1)*aero.alpha.amplitude, ...
                t);
        yDalpha = squeeze(yDalpha);
        PitchDistPeak(row) = ...
            max(abs(yDalpha(:,1)));
        yDbeta = ...
            step( ...
                Td(:,2)*aero.beta.amplitude, ...
                t);
        yDbeta = squeeze(yDbeta);
        YawDistPeak(row) = ...
            max(abs(yDbeta(:,2)));
    end
end
MonteCarloResults = table( ...
    resultController, ...
    resultSample, ...
    Stable, ...
    PitchSettling, ...
    YawSettling, ...
    PitchSettledWithinHorizon, ...
    YawSettledWithinHorizon, ...
    PitchBW, ...
    YawBW, ...
    PitchSSerror, ...
    YawSSerror, ...
    YawFromPitchPeak, ...
    PitchFromYawPeak, ...
    PitchDistPeak, ...
    YawDistPeak, ...
    'VariableNames',{
    'Controller'
    'Sample'
    'Stable'
    'PitchSettling'
    'YawSettling'
    'PitchSettledWithinHorizon'
    'YawSettledWithinHorizon'
    'PitchBandwidth'
    'YawBandwidth'
    'PitchSteadyStateErrorPercent'
    'YawSteadyStateErrorPercent'
    'YawFromPitchPeak'
    'PitchFromYawPeak'
    'PitchDisturbancePeak'
    'YawDisturbancePeak'
    });
disp(MonteCarloResults);
writetable( ...
    MonteCarloResults, ...
    'MonteCarloRobustness.csv');


%% ========================================================================
% FUNZIONE LOCALE - SETTLING TIME ROBUSTO
% ========================================================================
function [ts,settled] = settlingTimeFromTrace(t,y,yss,threshold)
%SETTLINGTIMEFROMTRACE Determina il primo istante dopo l'ultima uscita banda.
%
% La banda e' +/- threshold*|yss|. Per riferimenti quasi nulli viene usata
% una piccola scala assoluta per evitare una banda numericamente nulla.

    t = t(:);
    y = y(:);

    scale = max(abs(yss),1e-9);
    band = threshold*scale;
    outside = abs(y-yss) > band;

    lastOutside = find(outside,1,'last');

    if isempty(lastOutside)
        ts = t(1);
        settled = true;
    elseif lastOutside < numel(t)
        ts = t(lastOutside+1);
        settled = true;
    else
        ts = NaN;
        settled = false;
    end
end
