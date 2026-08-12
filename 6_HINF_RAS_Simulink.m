%% HINF_03_RAS_SIMULINK
%
% Stima numerica della sezione:
%
%   delta_alpha_dot(0) = 0
%   delta_beta_dot(0)  = 0
%
% della regione di attrazione del modello non lineare.
%
% Il rumore deve essere disabilitato perché la RAS riguarda
% una convergenza deterministica verso l'equilibrio.

close all;
clc;

load('HINF_setup.mat');
load('HINF_controllers.mat');

%% ========================================================================
%  1. CONFIGURAZIONE DEL MODELLO
% ========================================================================

modelName = 'helicopter_2DOF';

loggedStateName = 'x_nonlinear';

% Variabile del Variant Subsystem:
%
% 1 = mixsyn
% 2 = hinfsyn
% 3 = PID hinfstruct

controllerIDs = [1 2 3];

controllerNames = {
    'mixsyn'
    'hinfsyn'
    'hinfstruct PID'
};

stopTime = 15;

%% ========================================================================
%  2. CONFIGURAZIONE DETERMINISTICA
% ========================================================================

act.noiseEnable = 0;

if isfield(sensor,'noiseEnable')
    sensor.noiseEnable = 0;
end

delta_plant = zeros(7,1);

%% Riferimento all'equilibrio

delta_r_RAS = zeros(2,1);

%% ========================================================================
%  3. GRIGLIA DELLE CONDIZIONI INIZIALI
% ========================================================================

deltaAlphaGrid = deg2rad( ...
    linspace(-25,25,41));

deltaBetaGrid = deg2rad( ...
    linspace(-45,45,51));

%% Criteri di convergenza

angleTolerance = deg2rad(0.25);
rateTolerance  = deg2rad(0.5);

maxAlpha = deg2rad(85);
maxBeta  = deg2rad(170);

RASmaps = cell(numel(controllerIDs),1);

%% ========================================================================
%  4. CICLO SUI CONTROLLORI
% ========================================================================

for ic = 1:numel(controllerIDs)

    HINF_controller_id = controllerIDs(ic);

    stableMap = false( ...
        numel(deltaBetaGrid), ...
        numel(deltaAlphaGrid));

    fprintf('\nAnalisi RAS: %s\n', ...
        controllerNames{ic});

    for ia = 1:numel(deltaAlphaGrid)

        for ib = 1:numel(deltaBetaGrid)

            q0_test = [
                alpha0 + deltaAlphaGrid(ia);
                beta0  + deltaBetaGrid(ib)
            ];

            qdot0_test = zeros(2,1);

            simIn = Simulink.SimulationInput(modelName);

            simIn = simIn.setVariable( ...
                'HINF_controller_id', ...
                HINF_controller_id);

            simIn = simIn.setVariable( ...
                'q0', ...
                q0_test);

            simIn = simIn.setVariable( ...
                'qdot0', ...
                qdot0_test);

            simIn = simIn.setVariable( ...
                'delta_r_RAS', ...
                delta_r_RAS);

            simIn = simIn.setVariable( ...
                'act', ...
                act);

            simIn = simIn.setVariable( ...
                'sensor', ...
                sensor);

            simIn = simIn.setVariable( ...
                'delta_plant', ...
                delta_plant);

            simIn = simIn.setModelParameter( ...
                'StopTime', ...
                num2str(stopTime));

            try

                simOut = sim(simIn);

                xTS = simOut.logsout.get( ...
                    loggedStateName).Values;

                xData = squeeze(xTS.Data);

                if size(xData,2) ~= 4 && size(xData,1) == 4
                    xData = xData.';
                end

                xFinal = xData(end,:).';

                dxFinal = xFinal - x0;

                finiteTrajectory = ...
                    all(isfinite(xData(:)));

                withinLimits = ...
                    max(abs(xData(:,1))) < maxAlpha && ...
                    max(abs(xData(:,3))) < maxBeta;

                converged = ...
                    abs(dxFinal(1)) < angleTolerance && ...
                    abs(dxFinal(2)) < rateTolerance  && ...
                    abs(dxFinal(3)) < angleTolerance && ...
                    abs(dxFinal(4)) < rateTolerance;

                stableMap(ib,ia) = ...
                    finiteTrajectory && ...
                    withinLimits && ...
                    converged;

            catch simulationError

                warning( ...
                    'Simulazione fallita: %s', ...
                    simulationError.message);

                stableMap(ib,ia) = false;
            end
        end
    end

    RASmaps{ic} = stableMap;

    figure( ...
        'Name', ...
        ['RAS - ',controllerNames{ic}]);

    imagesc( ...
        rad2deg(deltaAlphaGrid), ...
        rad2deg(deltaBetaGrid), ...
        stableMap);

    axis xy;

    xlabel('\delta\alpha(0) [deg]');
    ylabel('\delta\beta(0) [deg]');

    title([ ...
        'Regione numerica di convergenza - ', ...
        controllerNames{ic}]);

    colorbar;
end

save('HINF_RAS_results.mat', ...
    'RASmaps', ...
    'deltaAlphaGrid', ...
    'deltaBetaGrid', ...
    'controllerNames');