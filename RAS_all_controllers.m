%% ========================================================================
% RAS_ALL_CONTROLLERS
%
% Sezione numerica della regione di attrazione:
%
% delta_alpha_dot(0) = 0
% delta_beta_dot(0)  = 0
%
% Modello NON LINEARE nominale.
% ========================================================================

close all;
clc;

modelName = 'helicopter_2DOF';

%% ========================================================================
% 1. CONFIGURAZIONE DETERMINISTICA
% ========================================================================

actRAS = act;
actRAS.noiseEnable = 0;

sensorRAS = sensor;
sensorRAS.noiseEnable = 0;

aeroRAS = aero;
aeroRAS.enable = 0;
aeroRAS.alpha.amplitude = 0;
aeroRAS.beta.amplitude  = 0;

deltaPlantRAS = zeros(7,1);

refRAS = ref;

refRAS.alpha.initial = alpha0;
refRAS.alpha.final   = alpha0;
refRAS.alpha.time    = 0;

refRAS.beta.initial = beta0;
refRAS.beta.final   = beta0;
refRAS.beta.time    = 0;

%% ========================================================================
% 2. CONTROLLORI
% ========================================================================

controllerData = {
    'LQG',          'LQG',  1, 'xNL_LQG';
    'LQGI',         'LQG',  2, 'xNL_LQG';
    'mixsyn',       'HINF', 1, 'xNL_HINF';
    'hinfsyn',      'HINF', 2, 'xNL_HINF';
    'PID+comp',     'HINF', 3, 'xNL_HINF';
    'mu-synthesis', 'HINF', 4, 'xNL_HINF'
};

%% ========================================================================
% 3. GRIGLIA
% ========================================================================

deltaAlphaGrid = ...
    deg2rad( ...
        linspace(-25,25,31));

deltaBetaGrid = ...
    deg2rad( ...
        linspace(-45,45,41));

angleTolerance = ...
    deg2rad(0.25);

rateTolerance = ...
    deg2rad(0.5);

maxAlpha = deg2rad(85);
maxBeta  = deg2rad(170);

stopTime = 15;

xEquilibrium = [
    alpha0
    0
    beta0
    0
];

RASmaps = ...
    cell(size(controllerData,1),1);

%% ========================================================================
% 4. CICLO
% ========================================================================

for ic = 1:size(controllerData,1)

    controllerName = ...
        controllerData{ic,1};

    family = ...
        controllerData{ic,2};

    controllerID = ...
        controllerData{ic,3};

    loggedVariable = ...
        controllerData{ic,4};

    stableMap = ...
        false( ...
            numel(deltaBetaGrid), ...
            numel(deltaAlphaGrid));

    fprintf('\nRAS: %s\n',controllerName);

    for ia = 1:numel(deltaAlphaGrid)

        for ib = 1:numel(deltaBetaGrid)

            q0_test = [
                alpha0 + deltaAlphaGrid(ia)
                beta0  + deltaBetaGrid(ib)
            ];

            qdot0_test = [
                0
                0
            ];

            simIn = ...
                Simulink.SimulationInput(modelName);

            simIn = ...
                simIn.setVariable( ...
                    'q0', ...
                    q0_test);

            simIn = ...
                simIn.setVariable( ...
                    'qdot0', ...
                    qdot0_test);

            simIn = ...
                simIn.setVariable( ...
                    'act', ...
                    actRAS);

            simIn = ...
                simIn.setVariable( ...
                    'sensor', ...
                    sensorRAS);

            simIn = ...
                simIn.setVariable( ...
                    'aero', ...
                    aeroRAS);

            simIn = ...
                simIn.setVariable( ...
                    'delta_plant', ...
                    deltaPlantRAS);

            simIn = ...
                simIn.setVariable( ...
                    'ref', ...
                    refRAS);

            if strcmp(family,'LQG')

                simIn = ...
                    simIn.setVariable( ...
                        'LQG_controller_id', ...
                        controllerID);

            else

                simIn = ...
                    simIn.setVariable( ...
                        'HINF_controller_id', ...
                        controllerID);

            end

            simIn = ...
                simIn.setModelParameter( ...
                    'StopTime', ...
                    num2str(stopTime));

            try

                simOut = sim(simIn);

                xTS = ...
                    simOut.get( ...
                        loggedVariable);

                xData = ...
                    squeeze(xTS.Data);

                if size(xData,2) ~= 4 && ...
                        size(xData,1) == 4

                    xData = xData.';
                end

                finiteTrajectory = ...
                    all(isfinite(xData(:)));

                withinLimits = ...
                    max(abs(xData(:,1))) < maxAlpha && ...
                    max(abs(xData(:,3))) < maxBeta;

                %% Convergenza sull'ultimo 10% della simulazione

                N = size(xData,1);

                firstTailSample = ...
                    max( ...
                        1, ...
                        floor(0.90*N));

                xTail = ...
                    xData(firstTailSample:end,:);

                errorTail = ...
                    xTail - ...
                    xEquilibrium;

                maxTailError = ...
                    max( ...
                        abs(errorTail), ...
                        [], ...
                        1);

                converged = ...
                    maxTailError(1) < angleTolerance && ...
                    maxTailError(2) < rateTolerance  && ...
                    maxTailError(3) < angleTolerance && ...
                    maxTailError(4) < rateTolerance;

                stableMap(ib,ia) = ...
                    finiteTrajectory && ...
                    withinLimits && ...
                    converged;

            catch ME

                fprintf( ...
                    'Simulazione fallita: %s\n', ...
                    ME.message);

                stableMap(ib,ia) = false;
            end
        end
    end

    RASmaps{ic} = ...
        stableMap;

    figure( ...
        'Name', ...
        ['RAS - ',controllerName]);

    imagesc( ...
        rad2deg(deltaAlphaGrid), ...
        rad2deg(deltaBetaGrid), ...
        stableMap);

    axis xy;
    grid on;

    xlabel('\delta\alpha(0) [deg]');
    ylabel('\delta\beta(0) [deg]');

    title( ...
        ['Sezione numerica RAS - ', ...
        controllerName]);

    colorbar;
end

save( ...
    'RAS_ALL_results.mat', ...
    'RASmaps', ...
    'deltaAlphaGrid', ...
    'deltaBetaGrid', ...
    'controllerData');