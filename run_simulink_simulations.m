%% RUN_SIMULINK_CAMPAIGN
% Campagna automatica di simulazioni Simulink per la relazione finale
% Traccia n. 3 - Controllo Robusto, Elicottero 2DOF
%
% Compatibile con MATLAB / Simulink R2026a.
%
% PRESUPPOSTI DEL MODELLO
% ----------------------
% 1) Il modello si chiama helicopter_2DOF.slx (modificare cfg.model se diverso).
% 2) Sono stati introdotti i selettori:
%       LQG_plant_id      : 1 = lineare, 2 = non lineare
%       Robust_plant_id   : 1 = lineare, 2 = non lineare
%       LQG_controller_id : 1 = LQGI,   2 = LQG
%       HINF_controller_id: 1 = mixsyn, 2 = hinfsyn,
%                           3 = PID+comp, 4 = mu-synthesis, 5 = H2
% 3) E' attivo Signal Logging (logsout) per i segnali elencati in SIG.
% 4) I segnali di reporting sono SOLO derivazioni e non rientrano nel loop.
%
% OUTPUT
% ------
% Crea la cartella ./simulink_result contenente:
%   - una sottocartella per ogni simulazione;
%   - PNG a 300 dpi equivalenti agli Scope di reporting;
%   - figure comparative finali;
%   - campaign_summary.csv.
%
% NOTA
% ----
% I grafici vengono ricostruiti dai segnali logsout anziche' "fotografare"
% la finestra Scope. In questo modo sono riproducibili, vettorialmente puliti
% e pronti per essere inseriti nella relazione.

clear;
close all;
clc;

%% ========================================================================
% 0. INIZIALIZZAZIONE DEL PROGETTO
% =========================================================================
%
% Gli script di inizializzazione vengono eseguiti nel Base Workspace,
% cioe' nello stesso workspace usato da Simulink per risolvere i parametri
% dei blocchi. Non viene usata la funzione MATLAB run(), cosi' non esiste
% alcun conflitto con eventuali file run.m presenti nel progetto.

runFirstExistingScriptInBase({ ...
    'build_uncertain_linear_model.m', ...
    'build_uncertain_linear_model(1).m'});

runFirstExistingScriptInBase({ ...
    'init.m', ...
    'init(1).m'});

% Se per qualsiasi motivo init.m non ha creato ref/aero, li ricostruiamo
% con i valori di test gia' adottati nel progetto.
ensureSimulationProfilesInBase();

% I controllori e i dati di progetto devono essere visibili a Simulink.
loadFirstExistingToBase({'LQG_2DOF_Controllers.mat'});
loadFirstExistingToBase({'HINF_setup.mat'});
loadFirstExistingToBase({'HINF_controllers.mat'});
loadFirstExistingToBase({'MU_controller.mat'});
loadFirstExistingToBase({'H2_controller.mat'});

% Ricostruisce Aobs, Bobs, Cobs, Dobs, xhat0 per il blocco State-Space
% Kalman_observer, se non sono gia' presenti nei MAT caricati.
ensureKalmanObserverInBase();

% Copie locali delle variabili usate direttamente da questo script.
ref    = getBaseVariable('ref');
aero   = getBaseVariable('aero');
act    = getBaseVariable('act');
sensor = getBaseVariable('sensor');
alpha0 = getBaseVariable('alpha0');
beta0  = getBaseVariable('beta0');

% Modello linearizzato incerto e trim incerto.
% Servono SOLO per costruire confronti LIN/NL closed-loop coerenti:
% per ogni realizzazione parametrica delta, il ramo lineare viene valutato
% sulla stessa realizzazione del ramo non lineare.
P_uncertain_ext = getBaseVariable('P_uncertain_ext');
u0_uncertain    = getBaseVariable('u0_uncertain');
u0_nominal      = getBaseVariable('u0_nominal');

% Verifica preventiva: meglio fermarsi qui che produrre una serie di errori Simulink.
validateBaseWorkspaceForCampaign();

%% ========================================================================
% 1. CONFIGURAZIONE GENERALE
% =========================================================================

cfg.model = 'helicopter_2DOF';
if ~isfile([cfg.model '.slx']) && isfile('helicopter_2DOF(2).slx')
    cfg.model = 'helicopter_2DOF(2)';
end

cfg.stopTime = 15;          % [s] - simulazioni standard
cfg.rasValidationStopTime = 40; % [s] - sola validazione del bordo RAS
cfg.pngResolution = 300;    % [dpi]
cfg.outputDir = fullfile(pwd,'simulink_result');
cfg.saveSimulationMAT = false;
cfg.showFigures = false;

% Per figure da relazione si preferisce una simulazione deterministica.
% Le prove stocastiche del Kalman possono essere abilitate separatamente.
cfg.defaultSensorNoise = 0;
cfg.defaultActuatorNoise = 0;

if ~exist(cfg.outputDir,'dir')
    mkdir(cfg.outputDir);
end

% Cancella solo i PNG/CSV della precedente campagna, non risultati MAT/RAS.
cleanupPreviousCampaign(cfg.outputDir);

%% ========================================================================
% 2. NOMI DEI SEGNALI LOGGATI
% =========================================================================
% Modificare SOLO questa sezione se nel modello sono stati usati nomi diversi.
%
% Ogni elemento deve corrispondere al Signal name visibile in logsout.

SIG.alphaRef = 'alpha_ref';
SIG.betaRef  = 'beta_ref';

SIG.alphaLQG = 'alpha_LQG';
SIG.betaLQG  = 'beta_LQG';
SIG.alphaRob = 'alpha_Robust';
SIG.betaRob  = 'beta_Robust';

SIG.eAlphaLQG = 'e_alpha_LQG';
SIG.eBetaLQG  = 'e_beta_LQG';
SIG.eAlphaRob = 'e_alpha_Robust';
SIG.eBetaRob  = 'e_beta_Robust';

SIG.u1cmdLQG = 'deltaF1_cmd_LQG';
SIG.u2cmdLQG = 'deltaF2_cmd_LQG';
SIG.u1cmdRob = 'deltaF1_cmd_Robust';
SIG.u2cmdRob = 'deltaF2_cmd_Robust';

SIG.u1actLQG = 'deltaF1_LQG';
SIG.u2actLQG = 'deltaF2_LQG';
SIG.u1actRob = 'deltaF1_Robust';
SIG.u2actRob = 'deltaF2_Robust';

SIG.dAlpha = 'd_alpha';
SIG.dBeta  = 'd_beta';

SIG.alphaLinLQG = 'alpha_lin_LQG';
SIG.betaLinLQG  = 'beta_lin_LQG';
SIG.alphaNLLQG  = 'alpha_nl_LQG';
SIG.betaNLLQG   = 'beta_nl_LQG';

SIG.alphaLinRob = 'alpha_lin_Robust';
SIG.betaLinRob  = 'beta_lin_Robust';
SIG.alphaNLRob  = 'alpha_nl_Robust';
SIG.betaNLRob   = 'beta_nl_Robust';

%% ========================================================================
% 3. PROFILI DI RIFERIMENTO E DISTURBO
% =========================================================================

refPitch = ref;
refPitch.alpha.initial = alpha0;
refPitch.alpha.final   = alpha0 + deg2rad(3);
refPitch.alpha.time    = 2;
refPitch.beta.initial  = beta0;
refPitch.beta.final    = beta0;
refPitch.beta.time     = 2;

refCombined = ref;
refCombined.alpha.initial = alpha0;
refCombined.alpha.final   = alpha0 + deg2rad(3);
refCombined.alpha.time    = 2;
refCombined.beta.initial  = beta0;
refCombined.beta.final    = beta0 - deg2rad(5);
refCombined.beta.time     = 5;

refEquilibrium = ref;
refEquilibrium.alpha.initial = alpha0;
refEquilibrium.alpha.final   = alpha0;
refEquilibrium.alpha.time    = 2;
refEquilibrium.beta.initial  = beta0;
refEquilibrium.beta.final    = beta0;
refEquilibrium.beta.time     = 2;

aeroOff = aero;
aeroOff.enable = 0;
aeroOff.alpha.amplitude = 0;
aeroOff.beta.amplitude  = 0;

aeroStep = aero;
aeroStep.enable = 1;
aeroStep.alpha.time      = 6;
aeroStep.alpha.amplitude = 5e-3;   % [N m]
aeroStep.beta.time       = 10;
aeroStep.beta.amplitude  = 2e-3;   % [N m]

% Se nel modello e' stato implementato aero.mode, questo valore e' innocuo
% anche per le prove step. Se il campo non viene usato, Simulink lo ignora.
aeroStep.mode = "step";

% Campione parametrico volutamente severo ma interno all'iper-cubo [-1,1].
% Ordine definito in fcn.m:
% [J_alpha, J_y, J_z, m, l, epsilon_p, epsilon_y]
deltaNominal = zeros(7,1);
deltaRobustTest = [ ...
    +1.0; ... % J_alpha +10%
    -1.0; ... % J_y     -15%
    +1.0; ... % J_z     +10%
    +1.0; ... % m       +10%
    -1.0; ... % l        -5%
    +1.0; ... % epsilon_p +30%
    -1.0  ... % epsilon_y -30%
    ];

%% ========================================================================
% 4. DEFINIZIONE DELLA CAMPAGNA
% =========================================================================
% plant:      1 lineare, 2 non lineare
% LQG ctl:    1 LQGI, 2 LQG
% robust ctl: 1 mixsyn, 2 hinfsyn, 3 PID+comp, 4 mu-synthesis, 5 H2

EXP = repmat(emptyExperiment(),0,1);

% ---- LQG / LQGI nominali -------------------------------------------------
% Per il confronto LIN/NL corretto ogni plant deve chiudere il proprio loop.
EXP(end+1) = makeExp('01_LQG_nominal_LIN', ...
    'LQG nominale - plant linearizzato', ...
    'LQG',1,2,1,refCombined,aeroOff,deltaNominal);

EXP(end+1) = makeExp('01_LQG_nominal_NL', ...
    'LQG nominale - plant non lineare', ...
    'LQG',2,2,1,refCombined,aeroOff,deltaNominal);

EXP(end+1) = makeExp('02_LQGI_nominal_NL', ...
    'LQGI nominale - plant non lineare', ...
    'LQG',2,1,1,refCombined,aeroOff,deltaNominal);

% ---- Motivazione dell'integratore ---------------------------------------
EXP(end+1) = makeExp('03_LQG_disturbance_LIN', ...
    'LQG - reiezione disturbo costante - plant linearizzato', ...
    'LQG',1,2,1,refCombined,aeroStep,deltaNominal);

EXP(end+1) = makeExp('03_LQG_disturbance_NL', ...
    'LQG - reiezione disturbo costante', ...
    'LQG',2,2,1,refCombined,aeroStep,deltaNominal);

EXP(end+1) = makeExp('04_LQGI_disturbance_LIN', ...
    'LQGI - reiezione disturbo costante - plant linearizzato', ...
    'LQG',1,1,1,refCombined,aeroStep,deltaNominal);

EXP(end+1) = makeExp('04_LQGI_disturbance_NL', ...
    'LQGI - reiezione disturbo costante', ...
    'LQG',2,1,1,refCombined,aeroStep,deltaNominal);

% ---- Validazione linearizzazione ----------------------------------------
EXP(end+1) = makeExp('05_LQGI_linear', ...
    'LQGI - modello linearizzato', ...
    'LQG',1,1,1,refCombined,aeroOff,deltaNominal);

EXP(end+1) = makeExp('06_LQGI_nonlinear', ...
    'LQGI - modello non lineare', ...
    'LQG',2,1,1,refCombined,aeroOff,deltaNominal);

% ---- Confronto nominale dei controllori robusti / H2 ----------------------
EXP(end+1) = makeExp('07_mixsyn_nominal_LIN', ...
    'mixsyn - nominale linearizzato', ...
    'ROB',1,1,1,refCombined,aeroOff,deltaNominal);

EXP(end+1) = makeExp('07_mixsyn_nominal_NL', ...
    'mixsyn - nominale non lineare', ...
    'ROB',2,1,1,refCombined,aeroOff,deltaNominal);

EXP(end+1) = makeExp('08_hinfsyn_nominal_LIN', ...
    'hinfsyn - nominale linearizzato', ...
    'ROB',1,1,2,refCombined,aeroOff,deltaNominal);

EXP(end+1) = makeExp('08_hinfsyn_nominal_NL', ...
    'hinfsyn - nominale non lineare', ...
    'ROB',2,1,2,refCombined,aeroOff,deltaNominal);

EXP(end+1) = makeExp('09_PIDcomp_nominal_LIN', ...
    'hinfstruct PID + compensatore - nominale linearizzato', ...
    'ROB',1,1,3,refCombined,aeroOff,deltaNominal);

EXP(end+1) = makeExp('09_PIDcomp_nominal_NL', ...
    'hinfstruct PID + compensatore - nominale non lineare', ...
    'ROB',2,1,3,refCombined,aeroOff,deltaNominal);

EXP(end+1) = makeExp('10_H2_nominal_LIN', ...
    'H2 - nominale linearizzato', ...
    'ROB',1,1,5,refCombined,aeroOff,deltaNominal);

EXP(end+1) = makeExp('10_H2_nominal_NL', ...
    'H2 - nominale non lineare', ...
    'ROB',2,1,5,refCombined,aeroOff,deltaNominal);

% ---- Stress test comune su plant incerto + disturbo ----------------------
EXP(end+1) = makeExp('11_mixsyn_uncertain_NL', ...
    'mixsyn - plant incerto non lineare con disturbo', ...
    'ROB',2,1,1,refCombined,aeroStep,deltaRobustTest);

EXP(end+1) = makeExp('12_mu_uncertain_NL', ...
    'mu-synthesis - plant incerto non lineare con disturbo', ...
    'ROB',2,1,4,refCombined,aeroStep,deltaRobustTest);

EXP(end+1) = makeExp('13_H2_uncertain_NL', ...
    'H2 - plant incerto non lineare con disturbo', ...
    'ROB',2,1,5,refCombined,aeroStep,deltaRobustTest);

% ---- Validazione LIN/NL della STESSA realizzazione parametrica ------------
% Queste prove NON sostituiscono gli stress test 11-13, che mantengono il
% trim nominale e verificano la robustezza anche rispetto all'errore di trim.
% Qui, invece, si usa u0(delta) sia sul LIN sia sul NL per isolare il confronto
% tra il linearizzato della realizzazione incerta e il rispettivo non lineare.

e = makeExp('V1_mixsyn_uncertain_matched_LIN', ...
    'mixsyn - realizzazione incerta matched - plant linearizzato', ...
    'ROB',1,1,1,refCombined,aeroStep,deltaRobustTest);
e.matchedTrim = true;
EXP(end+1) = e;

e = makeExp('V1_mixsyn_uncertain_matched_NL', ...
    'mixsyn - realizzazione incerta matched - plant non lineare', ...
    'ROB',2,1,1,refCombined,aeroStep,deltaRobustTest);
e.matchedTrim = true;
EXP(end+1) = e;

e = makeExp('V2_mu_uncertain_matched_LIN', ...
    'mu-synthesis - realizzazione incerta matched - plant linearizzato', ...
    'ROB',1,1,4,refCombined,aeroStep,deltaRobustTest);
e.matchedTrim = true;
EXP(end+1) = e;

e = makeExp('V2_mu_uncertain_matched_NL', ...
    'mu-synthesis - realizzazione incerta matched - plant non lineare', ...
    'ROB',2,1,4,refCombined,aeroStep,deltaRobustTest);
e.matchedTrim = true;
EXP(end+1) = e;

e = makeExp('V3_H2_uncertain_matched_LIN', ...
    'H2 - realizzazione incerta matched - plant linearizzato', ...
    'ROB',1,1,5,refCombined,aeroStep,deltaRobustTest);
e.matchedTrim = true;
EXP(end+1) = e;

e = makeExp('V3_H2_uncertain_matched_NL', ...
    'H2 - realizzazione incerta matched - plant non lineare', ...
    'ROB',2,1,5,refCombined,aeroStep,deltaRobustTest);
e.matchedTrim = true;
EXP(end+1) = e;

%% ========================================================================
% 5. EVENTUALI PUNTI RAS AUTOMATICI
% =========================================================================
% Per mu-synthesis e H2 vengono cercate coppie adiacenti rispetto alla mappa
% numerica a orizzonte finito: un punto classificato interno e uno esterno.
% Le prove a 40 s verificano se il punto esterno e' realmente divergente o
% semplicemente convergente piu' lentamente del criterio RAS a 15 s.

rasInfoMu = findRASBoundaryPair('mu');
rasInfoH2 = findRASBoundaryPair('h2');

if rasInfoMu.available
    fprintf('\nRAS: trovata coppia automatica vicino al bordo per %s.\n', ...
        rasInfoMu.controllerName);

    eIn = makeExp('14_RAS_inside_mu', ...
        'mu-synthesis - punto interno alla mappa RAS numerica', ...
        'ROB',2,1,4,refEquilibrium,aeroOff,deltaNominal);
    eIn.q0Override = [alpha0 + deg2rad(rasInfoMu.inside(1)); ...
                      beta0  + deg2rad(rasInfoMu.inside(2))];
    eIn.stopTime = cfg.rasValidationStopTime;
    EXP(end+1) = eIn;

    eOut = makeExp('15_RAS_outside_mu', ...
        'mu-synthesis - punto appena esterno alla mappa RAS numerica', ...
        'ROB',2,1,4,refEquilibrium,aeroOff,deltaNominal);
    eOut.q0Override = [alpha0 + deg2rad(rasInfoMu.outside(1)); ...
                       beta0  + deg2rad(rasInfoMu.outside(2))];
    eOut.stopTime = cfg.rasValidationStopTime;
    EXP(end+1) = eOut;
else
    fprintf('\nRAS: coppia mu-synthesis non disponibile.\n');
end

if rasInfoH2.available
    fprintf('\nRAS: trovata coppia automatica vicino al bordo per %s.\n', ...
        rasInfoH2.controllerName);

    eIn = makeExp('16_RAS_inside_H2', ...
        'H2 - punto interno alla mappa RAS numerica', ...
        'ROB',2,1,5,refEquilibrium,aeroOff,deltaNominal);
    eIn.q0Override = [alpha0 + deg2rad(rasInfoH2.inside(1)); ...
                      beta0  + deg2rad(rasInfoH2.inside(2))];
    eIn.stopTime = cfg.rasValidationStopTime;
    EXP(end+1) = eIn;

    eOut = makeExp('17_RAS_outside_H2', ...
        'H2 - punto appena esterno alla mappa RAS numerica', ...
        'ROB',2,1,5,refEquilibrium,aeroOff,deltaNominal);
    eOut.q0Override = [alpha0 + deg2rad(rasInfoH2.outside(1)); ...
                       beta0  + deg2rad(rasInfoH2.outside(2))];
    eOut.stopTime = cfg.rasValidationStopTime;
    EXP(end+1) = eOut;
else
    fprintf('\nRAS: coppia H2 non disponibile.\n');
end

%% ========================================================================
% 6. PREPARAZIONE DEL MODELLO
% =========================================================================

load_system(cfg.model);

% I segnali marcati "Log Selected Signals" confluiscono in logsout.
set_param(cfg.model,'SignalLogging','on');
set_param(cfg.model,'SignalLoggingName','logsout');
set_param(cfg.model,'StopTime',num2str(cfg.stopTime));

% Evita l'apertura automatica degli Scope durante una campagna batch.
setScopeOpenState(cfg.model,'off');

%% ========================================================================
% 7. ESECUZIONE DELLE SIMULAZIONI
% =========================================================================

outputs = cell(numel(EXP),1);
summaryRows = cell(numel(EXP),1);

fprintf('\n============================================================\n');
fprintf('CAMPAGNA SIMULINK - %d SIMULAZIONI\n',numel(EXP));
fprintf('============================================================\n');

for k = 1:numel(EXP)
    exp = EXP(k);

    fprintf('\n[%02d/%02d] %s\n',k,numel(EXP),exp.description);

    expDir = fullfile(cfg.outputDir,exp.id);
    if ~exist(expDir,'dir')
        mkdir(expDir);
    end

    in = Simulink.SimulationInput(cfg.model);

    % Selettori di modello e controllore.
    in = in.setVariable('LQG_plant_id',exp.plantId);
    in = in.setVariable('Robust_plant_id',exp.plantId);
    in = in.setVariable('LQG_controller_id',exp.lqgControllerId);
    in = in.setVariable('HINF_controller_id',exp.robustControllerId);

    % Ingressi e incertezza non lineare.
    in = in.setVariable('ref',exp.ref);
    in = in.setVariable('aero',exp.aero);
    in = in.setVariable('delta_plant',exp.deltaPlant);

    % --------------------------------------------------------------------
    % Realizzazione deterministica del modello linearizzato.
    %
    % Se plantId = 1 il blocco linearizzato deve rappresentare la STESSA
    % realizzazione parametrica descritta da exp.deltaPlant. Nel caso
    % nominale delta = 0 e si recupera esattamente P_nominal_ext.
    %
    % Per le prove "matched" viene inoltre usato sul plant non lineare il
    % trim u0(delta) coerente con quella stessa realizzazione. Gli stress
    % test originali 11-13 mantengono invece il trim nominale.
    % --------------------------------------------------------------------
    if exp.plantId == 1 || exp.matchedTrim
        [PextRun,u0Matched] = evaluatePlantRealizationAtNormalizedDelta( ...
            P_uncertain_ext,u0_uncertain,exp.deltaPlant);

        if exp.plantId == 1
            % Il blocco State-Space linearizzato usa il nome P_nominal_ext.
            % SimulationInput lo sovrascrive SOLO per questa singola run.
            in = in.setVariable('P_nominal_ext',PextRun);
        end
    else
        u0Matched = u0_nominal;
    end

    if exp.matchedTrim
        in = in.setVariable('u0',u0Matched);
    else
        in = in.setVariable('u0',u0_nominal);
    end

    % Simulazioni deterministiche per figure da relazione.
    actRun = act;
    sensorRun = sensor;
    actRun.noiseEnable = cfg.defaultActuatorNoise;
    sensorRun.noiseEnable = cfg.defaultSensorNoise;

    in = in.setVariable('act',actRun);
    in = in.setVariable('sensor',sensorRun);

    % Punto iniziale RAS, quando richiesto.
    if ~isempty(exp.q0Override)
        in = in.setVariable('q0',exp.q0Override);
        in = in.setVariable('qdot0',[0;0]);
    else
        in = in.setVariable('q0',[alpha0;beta0]);
        in = in.setVariable('qdot0',[0;0]);
    end

    runStopTime = cfg.stopTime;
    if isfinite(exp.stopTime)
        runStopTime = exp.stopTime;
    end

    in = in.setModelParameter( ...
        'StopTime',num2str(runStopTime), ...
        'ReturnWorkspaceOutputs','on');

    tic;
    try
        % Durante sim() vengono soppressi temporaneamente i warning MATLAB/
        % Simulink per mantenere leggibile la console. Gli errori continuano
        % invece a propagarsi normalmente e vengono gestiti dal catch.
        out = simWithWarningsSuppressed(in);
        elapsed = toc;
        outputs{k} = out;

        logs = getLogsout(out);

        % Produce i PNG equivalenti agli Scope pertinenti alla famiglia.
        makeRunPlots(logs,exp,SIG,expDir,cfg);

        if cfg.saveSimulationMAT
            save(fullfile(expDir,'simulation_output.mat'),'out','exp','-v7.3');
        end

        summaryRows{k} = {string(exp.id),string(exp.description),true,elapsed,""};
        fprintf('  completata in %.2f s\n',elapsed);

    catch ME
        elapsed = toc;
        outputs{k} = [];
        summaryRows{k} = {string(exp.id),string(exp.description),false,elapsed,string(ME.message)};
        warning('Simulazione %s fallita:\n%s',exp.id,getReport(ME,'basic','hyperlinks','off'));
    end
end

%% ========================================================================
% 8. FIGURE COMPARATIVE PER LA RELAZIONE
% =========================================================================

comparisonDir = fullfile(cfg.outputDir,'comparisons');
if ~exist(comparisonDir,'dir')
    mkdir(comparisonDir);
end

% 8.1 LQG vs LQGI - nominale
compareRuns(outputs,EXP, ...
    {'01_LQG_nominal_NL','02_LQGI_nominal_NL'}, ...
    {'LQG','LQGI'},'LQG',SIG,comparisonDir,cfg, ...
    '01_LQG_vs_LQGI_nominal_tracking', ...
    'Confronto nominale LQG e LQGI');

% 8.2 LQG vs LQGI - disturbo costante (grafico errori)
compareErrors(outputs,EXP, ...
    {'03_LQG_disturbance_NL','04_LQGI_disturbance_NL'}, ...
    {'LQG','LQGI'},'LQG',SIG,comparisonDir,cfg, ...
    '02_LQG_vs_LQGI_disturbance_error', ...
    'Errore di tracking sotto disturbo aerodinamico costante');

% 8.3 Linearizzato vs non lineare LQGI
compareRuns(outputs,EXP, ...
    {'05_LQGI_linear','06_LQGI_nonlinear'}, ...
    {'Linearizzato','Non lineare'},'LQG',SIG,comparisonDir,cfg, ...
    '03_linear_vs_nonlinear_LQGI', ...
    'Validazione del modello linearizzato con LQGI');

% 8.4 confronto nominale: H-infinity / PID strutturato / H2
compareRuns(outputs,EXP, ...
    {'07_mixsyn_nominal_NL','08_hinfsyn_nominal_NL', ...
     '09_PIDcomp_nominal_NL','10_H2_nominal_NL'}, ...
    {'mixsyn','hinfsyn','PID + compensatore','H2'},'ROB',SIG,comparisonDir,cfg, ...
    '04_robust_controllers_and_H2_tracking', ...
    'Confronto nominale dei controllori robusti e H2 sul modello non lineare');

compareControls(outputs,EXP, ...
    {'07_mixsyn_nominal_NL','08_hinfsyn_nominal_NL', ...
     '09_PIDcomp_nominal_NL','10_H2_nominal_NL'}, ...
    {'mixsyn','hinfsyn','PID + compensatore','H2'},'ROB',SIG,comparisonDir,cfg, ...
    '05_robust_controllers_and_H2_control_effort', ...
    'Confronto dello sforzo di controllo nominale');

% 8.5 stress test comune: mixsyn vs mu-synthesis vs H2
compareRuns(outputs,EXP, ...
    {'11_mixsyn_uncertain_NL','12_mu_uncertain_NL','13_H2_uncertain_NL'}, ...
    {'mixsyn','mu-synthesis','H2'},'ROB',SIG,comparisonDir,cfg, ...
    '06_mixsyn_vs_mu_vs_H2_uncertain_tracking', ...
    'Confronto su plant non lineare incerto con disturbo');

compareControls(outputs,EXP, ...
    {'11_mixsyn_uncertain_NL','12_mu_uncertain_NL','13_H2_uncertain_NL'}, ...
    {'mixsyn','mu-synthesis','H2'},'ROB',SIG,comparisonDir,cfg, ...
    '07_mixsyn_vs_mu_vs_H2_uncertain_control_effort', ...
    'Sforzo di controllo sul plant incerto');

% 8.6 RAS mu-synthesis
if rasInfoMu.available
    compareRuns(outputs,EXP, ...
        {'14_RAS_inside_mu','15_RAS_outside_mu'}, ...
        {'Interno al criterio','Esterno al criterio'},'ROB',SIG,comparisonDir,cfg, ...
        '08_RAS_inside_vs_outside_mu', ...
        'Validazione a lungo termine del bordo numerico - mu-synthesis');
end

% 8.7 RAS H2
if rasInfoH2.available
    compareRuns(outputs,EXP, ...
        {'16_RAS_inside_H2','17_RAS_outside_H2'}, ...
        {'Interno al criterio','Esterno al criterio'},'ROB',SIG,comparisonDir,cfg, ...
        '09_RAS_inside_vs_outside_H2', ...
        'Validazione a lungo termine del bordo numerico - H2');
end


% 8.8 Confronti LIN/NL closed-loop corretti
% -------------------------------------------------------------------------
% Ogni coppia seguente usa DUE simulazioni indipendenti:
%   - nella run LIN il controllore chiude il loop sul plant linearizzato;
%   - nella run NL  il controllore chiude il loop sul plant non lineare.
%
% In questo modo tracking, tracking error e differenza degli errori hanno
% un significato closed-loop corretto. Le prove RAS sono volutamente escluse:
% lontano dal punto di equilibrio il linearizzato non e' un modello adatto
% per stimare la RAS del sistema non lineare.

linNlDir = fullfile(cfg.outputDir,'linear_vs_nonlinear_closed_loop');
if ~exist(linNlDir,'dir')
    mkdir(linNlDir);
end

makeLinNlComparisonSet(outputs,EXP, ...
    {'01_LQG_nominal_LIN','01_LQG_nominal_NL'}, ...
    'LQG',SIG,linNlDir,cfg, ...
    '01_LQG_nominal', ...
    'LQG nominale');

makeLinNlComparisonSet(outputs,EXP, ...
    {'05_LQGI_linear','06_LQGI_nonlinear'}, ...
    'LQG',SIG,linNlDir,cfg, ...
    '02_LQGI_nominal', ...
    'LQGI nominale');

makeLinNlComparisonSet(outputs,EXP, ...
    {'03_LQG_disturbance_LIN','03_LQG_disturbance_NL'}, ...
    'LQG',SIG,linNlDir,cfg, ...
    '03_LQG_disturbance', ...
    'LQG con disturbo aerodinamico costante');

makeLinNlComparisonSet(outputs,EXP, ...
    {'04_LQGI_disturbance_LIN','04_LQGI_disturbance_NL'}, ...
    'LQG',SIG,linNlDir,cfg, ...
    '04_LQGI_disturbance', ...
    'LQGI con disturbo aerodinamico costante');

makeLinNlComparisonSet(outputs,EXP, ...
    {'07_mixsyn_nominal_LIN','07_mixsyn_nominal_NL'}, ...
    'ROB',SIG,linNlDir,cfg, ...
    '05_mixsyn_nominal', ...
    'mixsyn nominale');

makeLinNlComparisonSet(outputs,EXP, ...
    {'08_hinfsyn_nominal_LIN','08_hinfsyn_nominal_NL'}, ...
    'ROB',SIG,linNlDir,cfg, ...
    '06_hinfsyn_nominal', ...
    'hinfsyn nominale');

makeLinNlComparisonSet(outputs,EXP, ...
    {'09_PIDcomp_nominal_LIN','09_PIDcomp_nominal_NL'}, ...
    'ROB',SIG,linNlDir,cfg, ...
    '07_PIDcomp_nominal', ...
    'PID + compensatore nominale');

makeLinNlComparisonSet(outputs,EXP, ...
    {'10_H2_nominal_LIN','10_H2_nominal_NL'}, ...
    'ROB',SIG,linNlDir,cfg, ...
    '08_H2_nominal', ...
    'H2 nominale');

makeLinNlComparisonSet(outputs,EXP, ...
    {'V1_mixsyn_uncertain_matched_LIN','V1_mixsyn_uncertain_matched_NL'}, ...
    'ROB',SIG,linNlDir,cfg, ...
    '09_mixsyn_uncertain_matched', ...
    'mixsyn - stessa realizzazione parametrica incerta e trim coerente');

makeLinNlComparisonSet(outputs,EXP, ...
    {'V2_mu_uncertain_matched_LIN','V2_mu_uncertain_matched_NL'}, ...
    'ROB',SIG,linNlDir,cfg, ...
    '10_mu_uncertain_matched', ...
    'mu-synthesis - stessa realizzazione parametrica incerta e trim coerente');

makeLinNlComparisonSet(outputs,EXP, ...
    {'V3_H2_uncertain_matched_LIN','V3_H2_uncertain_matched_NL'}, ...
    'ROB',SIG,linNlDir,cfg, ...
    '11_H2_uncertain_matched', ...
    'H2 - stessa realizzazione parametrica incerta e trim coerente');

%% ========================================================================
% 9. RIEPILOGO CSV
% =========================================================================

summaryRows = vertcat(summaryRows{:});
summaryTable = cell2table(summaryRows, ...
    'VariableNames',{'Experiment','Description','Success','Elapsed_s','ErrorMessage'});

writetable(summaryTable,fullfile(cfg.outputDir,'campaign_summary.csv'));

disp(' ');
disp('============================================================');
disp('CAMPAGNA SIMULINK COMPLETATA');
disp('============================================================');
disp(summaryTable(:,1:4));
fprintf('Risultati: %s\n',cfg.outputDir);

% Ripristina apertura manuale degli Scope se desiderato.
% setScopeOpenState(cfg.model,'on');

%% ========================================================================
% FUNZIONI LOCALI
% =========================================================================

function E = emptyExperiment()
    E = struct( ...
        'id','', ...
        'description','', ...
        'family','', ...
        'plantId',2, ...
        'lqgControllerId',1, ...
        'robustControllerId',1, ...
        'ref',struct, ...
        'aero',struct, ...
        'deltaPlant',zeros(7,1), ...
        'q0Override',[], ...
        'stopTime',NaN, ...
        'matchedTrim',false);
end

function E = makeExp(id,description,family,plantId,lqgId,robId,ref,aero,deltaPlant)
    E = emptyExperiment();
    E.id = id;
    E.description = description;
    E.family = family;
    E.plantId = plantId;
    E.lqgControllerId = lqgId;
    E.robustControllerId = robId;
    E.ref = ref;
    E.aero = aero;
    E.deltaPlant = deltaPlant(:);
end

function [Pdet,u0det] = evaluatePlantRealizationAtNormalizedDelta(PuncExt,u0unc,delta)
%EVALUATEPLANTREALIZATIONATNORMALIZEDDELTA
% Valuta P_uncertain_ext e u0_uncertain sulla stessa realizzazione fisica
% descritta dal vettore normalizzato delta in [-1,1].
%
% Ordine coerente con fcn.m e build_uncertain_linear_model.m:
%   [J_alpha, J_y, J_z, m, l, epsilon_p, epsilon_y]

    names = { ...
        'J_alpha', ...
        'J_y', ...
        'J_z', ...
        'm', ...
        'l', ...
        'epsilon_p', ...
        'epsilon_y'};

    delta = delta(:);

    if numel(delta) ~= numel(names)
        error('deltaPlant deve avere %d elementi.',numel(names));
    end

    U = PuncExt.Uncertainty;
    values = struct;

    for k = 1:numel(names)
        name = names{k};

        if ~isfield(U,name)
            error('Il blocco incerto "%s" non e'' presente in P_uncertain_ext.',name);
        end

        blk = U.(name);
        d = max(-1,min(1,delta(k)));

        nominal = blk.NominalValue;
        range = blk.Range;

        % Mappa lineare della variabile normalizzata d in [-1,1] sul range
        % effettivo del corrispondente ureal. Gestisce anche range asimmetrici.
        if d >= 0
            value = nominal + d*(range(2)-nominal);
        else
            value = nominal + (-d)*(range(1)-nominal);
        end

        values.(name) = value;
    end

    % ---- Modello linearizzato della realizzazione -----------------------
    Pwork = PuncExt;

    for k = 1:numel(names)
        name = names{k};
        Pwork = usubs(Pwork,name,values.(name));
    end

    try
        remainingP = fieldnames(Pwork.Uncertainty);
    catch
        remainingP = {};
    end

    if ~isempty(remainingP)
        error('Restano incertezze non sostituite in PextRun: %s', ...
            strjoin(remainingP,', '));
    end

    Pdet = ss(Pwork);

    % ---- Trim della stessa realizzazione --------------------------------
    uwork = u0unc;

    for k = 1:numel(names)
        name = names{k};

        try
            Uu = uwork.Uncertainty;
            present = isfield(Uu,name);
        catch
            present = false;
        end

        if present
            uwork = usubs(uwork,name,values.(name));
        end
    end

    try
        remainingU0 = fieldnames(uwork.Uncertainty);
    catch
        remainingU0 = {};
    end

    if ~isempty(remainingU0)
        error('Restano incertezze non sostituite in u0Run: %s', ...
            strjoin(remainingU0,', '));
    end

    u0det = double(uwork);
    u0det = u0det(:);
end

function logs = getLogsout(out)
    if isprop(out,'logsout') || isfield(out,'logsout')
        logs = out.logsout;
    else
        try
            logs = out.get('logsout');
        catch
            error(['logsout non disponibile. Abilitare Signal Logging nel modello ', ...
                   'e marcare i segnali di REPORTING con Log Selected Signals.']);
        end
    end

    if isempty(logs)
        error('logsout e'' vuoto: nessun segnale risulta loggato.');
    end
end

function makeRunPlots(logs,exp,SIG,outDir,cfg)
    if strcmpi(exp.family,'LQG')
        family = 'LQG';
    else
        family = 'ROB';
    end

    % Tracking
    try
        [aRef,bRef,a,b] = trackingSignals(logs,family,SIG);
        f = newFigure(cfg);
        tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

        nexttile;
        plot(aRef.Time,rad2deg(aRef.Data),'--','LineWidth',1.4); hold on;
        plot(a.Time,rad2deg(a.Data),'LineWidth',1.5);
        grid on;
        ylabel('$\alpha$ [deg]','Interpreter','latex');
        title('Pitch tracking','Interpreter','latex');
        legend({'$\alpha_{ref}$','$\alpha$'},'Interpreter','latex','Location','best');

        nexttile;
        plot(bRef.Time,rad2deg(bRef.Data),'--','LineWidth',1.4); hold on;
        plot(b.Time,rad2deg(b.Data),'LineWidth',1.5);
        grid on;
        xlabel('Time [s]','Interpreter','latex');
        ylabel('$\beta$ [deg]','Interpreter','latex');
        title('Yaw tracking','Interpreter','latex');
        legend({'$\beta_{ref}$','$\beta$'},'Interpreter','latex','Location','best');

        sgtitle(exp.description,'Interpreter','none');
        exportFigure(f,fullfile(outDir,'01_tracking.png'),cfg);
    catch ME
        warnMissing('tracking',exp.id,ME);
    end

    % Tracking error
    try
        [ea,eb] = errorSignals(logs,family,SIG);
        f = newFigure(cfg);
        tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

        nexttile;
        plot(ea.Time,rad2deg(ea.Data),'LineWidth',1.5); grid on; yline(0,'--');
        ylabel('$e_\alpha$ [deg]','Interpreter','latex');
        title('Pitch tracking error','Interpreter','latex');

        nexttile;
        plot(eb.Time,rad2deg(eb.Data),'LineWidth',1.5); grid on; yline(0,'--');
        xlabel('Time [s]','Interpreter','latex');
        ylabel('$e_\beta$ [deg]','Interpreter','latex');
        title('Yaw tracking error','Interpreter','latex');

        sgtitle(exp.description,'Interpreter','none');
        exportFigure(f,fullfile(outDir,'02_tracking_error.png'),cfg);
    catch ME
        warnMissing('tracking error',exp.id,ME);
    end

    % Control effort
    try
        [u1,u2] = controlSignals(logs,family,SIG);
        f = newFigure(cfg);
        tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

        nexttile;
        plot(u1.Time,u1.Data,'LineWidth',1.5); grid on;
        ylabel('$\Delta F_{1,cmd}$ [N]','Interpreter','latex');
        title('Main-rotor control command','Interpreter','latex');

        nexttile;
        plot(u2.Time,u2.Data,'LineWidth',1.5); grid on;
        xlabel('Time [s]','Interpreter','latex');
        ylabel('$\Delta F_{2,cmd}$ [N]','Interpreter','latex');
        title('Tail-rotor control command','Interpreter','latex');

        sgtitle(exp.description,'Interpreter','none');
        exportFigure(f,fullfile(outDir,'03_control_effort.png'),cfg);
    catch ME
        warnMissing('control effort',exp.id,ME);
    end

    % Commanded vs actual actuator force
    try
        [u1c,u2c] = controlSignals(logs,family,SIG);
        [u1a,u2a] = actuatorSignals(logs,family,SIG);
        f = newFigure(cfg);
        tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

        nexttile;
        plot(u1c.Time,u1c.Data,'--','LineWidth',1.3); hold on;
        plot(u1a.Time,u1a.Data,'LineWidth',1.5); grid on;
        ylabel('$\Delta F_1$ [N]','Interpreter','latex');
        title('Main-rotor actuator dynamics','Interpreter','latex');
        legend({'Commanded','Actual'},'Location','best');

        nexttile;
        plot(u2c.Time,u2c.Data,'--','LineWidth',1.3); hold on;
        plot(u2a.Time,u2a.Data,'LineWidth',1.5); grid on;
        xlabel('Time [s]','Interpreter','latex');
        ylabel('$\Delta F_2$ [N]','Interpreter','latex');
        title('Tail-rotor actuator dynamics','Interpreter','latex');
        legend({'Commanded','Actual'},'Location','best');

        sgtitle(exp.description,'Interpreter','none');
        exportFigure(f,fullfile(outDir,'04_actuator_response.png'),cfg);
    catch ME
        warnMissing('actuator response',exp.id,ME);
    end

    % Disturbances
    try
        da = tsFromLogs(logs,SIG.dAlpha);
        db = tsFromLogs(logs,SIG.dBeta);
        f = newFigure(cfg);
        tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

        nexttile;
        plot(da.Time,da.Data,'LineWidth',1.5); grid on;
        ylabel('$d_\alpha$ [N m]','Interpreter','latex');
        title('Pitch aerodynamic disturbance','Interpreter','latex');

        nexttile;
        plot(db.Time,db.Data,'LineWidth',1.5); grid on;
        xlabel('Time [s]','Interpreter','latex');
        ylabel('$d_\beta$ [N m]','Interpreter','latex');
        title('Yaw aerodynamic disturbance','Interpreter','latex');

        sgtitle(exp.description,'Interpreter','none');
        exportFigure(f,fullfile(outDir,'05_aerodynamic_disturbances.png'),cfg);
    catch ME
        warnMissing('disturbances',exp.id,ME);
    end

    % NOTA:
    % I segnali dei due plant simulati in parallelo nella stessa run non
    % vengono piu' usati per il confronto ufficiale LIN/NL. Solo il plant
    % selezionato chiude infatti il feedback. I confronti corretti vengono
    % costruiti nella Sezione 8.8 usando due run closed-loop indipendenti.

end

function makeLinNlComparisonSet(outputs,EXP,ids,family,SIG,rootDir,cfg,stem,figTitle)
%MAKELINNLCOMPARISONSET Genera confronti LIN/NL da due closed loop distinti.
%
% Produce:
%   01_tracking_LIN_vs_NL.png
%   02_tracking_error_LIN_vs_NL.png
%   03_tracking_error_difference.png

    pairDir = fullfile(rootDir,stem);
    if ~exist(pairDir,'dir')
        mkdir(pairDir);
    end

    labels = {'Linearizzato','Non lineare'};

    compareRuns(outputs,EXP,ids,labels,family,SIG,pairDir,cfg, ...
        '01_tracking_LIN_vs_NL', ...
        [figTitle ' - tracking LIN vs NL']);

    compareErrors(outputs,EXP,ids,labels,family,SIG,pairDir,cfg, ...
        '02_tracking_error_LIN_vs_NL', ...
        [figTitle ' - errore di tracking LIN vs NL']);

    compareClosedLoopErrorGap(outputs,EXP,ids,family,SIG,pairDir,cfg, ...
        '03_tracking_error_difference', ...
        [figTitle ' - differenza degli errori closed-loop']);
end

function compareClosedLoopErrorGap(outputs,EXP,ids,family,SIG,outDir,cfg,fileName,figTitle)
%COMPARECLOSEDLOOPERRORGAP Confronta e_NL - e_LIN.
%
% IMPORTANTE: ids{1} deve essere la run LIN e ids{2} la run NL.
% Entrambe sono simulazioni closed-loop indipendenti.

    idx = indicesForIds(EXP,ids);
    if any(cellfun(@isempty,outputs(idx)))
        warning('Confronto %s saltato: una simulazione non e'' disponibile.',fileName);
        return
    end

    logsLin = getLogsout(outputs{idx(1)});
    logsNL  = getLogsout(outputs{idx(2)});

    [eaLin,ebLin] = errorSignals(logsLin,family,SIG);
    [eaNL, ebNL ] = errorSignals(logsNL, family,SIG);

    % Interpolazione del LIN sulla griglia temporale del NL.
    % Serve solo per sottrarre due timeseries che possono avere griglie
    % diverse a causa del solver variable-step.
    eaLinI = interp1(eaLin.Time,eaLin.Data,eaNL.Time,'linear','extrap');
    ebLinI = interp1(ebLin.Time,ebLin.Data,ebNL.Time,'linear','extrap');

    deltaEa = eaNL.Data - eaLinI;
    deltaEb = ebNL.Data - ebLinI;

    f = newFigure(cfg);
    tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

    nexttile;
    plot(eaNL.Time,rad2deg(deltaEa),'LineWidth',1.5);
    grid on;
    yline(0,'k--');
    ylabel('$e_{\alpha,NL}-e_{\alpha,LIN}$ [deg]','Interpreter','latex');
    title('Pitch: closed-loop tracking-error difference','Interpreter','latex');

    nexttile;
    plot(ebNL.Time,rad2deg(deltaEb),'LineWidth',1.5);
    grid on;
    yline(0,'k--');
    xlabel('Time [s]','Interpreter','latex');
    ylabel('$e_{\beta,NL}-e_{\beta,LIN}$ [deg]','Interpreter','latex');
    title('Yaw: closed-loop tracking-error difference','Interpreter','latex');

    sgtitle(figTitle,'Interpreter','none');
    exportFigure(f,fullfile(outDir,[fileName '.png']),cfg);
end

function compareRuns(outputs,EXP,ids,labels,family,SIG,outDir,cfg,fileName,figTitle)
    idx = indicesForIds(EXP,ids);
    if any(cellfun(@isempty,outputs(idx)))
        warning('Confronto %s saltato: una simulazione non e'' disponibile.',fileName);
        return
    end

    f = newFigure(cfg);
    tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

    nexttile; hold on; grid on;
    for i = 1:numel(idx)
        logs = getLogsout(outputs{idx(i)});
        [~,~,a,~] = trackingSignals(logs,family,SIG);
        plotSemanticTrace(a.Time,rad2deg(a.Data),labels{i},1.5);
    end
    logs0 = getLogsout(outputs{idx(1)});
    [aRef,~,~,~] = trackingSignals(logs0,family,SIG);
    plot(aRef.Time,rad2deg(aRef.Data),'k--','LineWidth',1.3,'DisplayName','Reference');
    ylabel('$\alpha$ [deg]','Interpreter','latex');
    title('Pitch','Interpreter','latex');
    legend('Location','best');

    nexttile; hold on; grid on;
    for i = 1:numel(idx)
        logs = getLogsout(outputs{idx(i)});
        [~,~,~,b] = trackingSignals(logs,family,SIG);
        plotSemanticTrace(b.Time,rad2deg(b.Data),labels{i},1.5);
    end
    [~,bRef,~,~] = trackingSignals(logs0,family,SIG);
    plot(bRef.Time,rad2deg(bRef.Data),'k--','LineWidth',1.3,'DisplayName','Reference');
    xlabel('Time [s]','Interpreter','latex');
    ylabel('$\beta$ [deg]','Interpreter','latex');
    title('Yaw','Interpreter','latex');
    legend('Location','best');

    sgtitle(figTitle,'Interpreter','none');
    exportFigure(f,fullfile(outDir,[fileName '.png']),cfg);
end

function compareErrors(outputs,EXP,ids,labels,family,SIG,outDir,cfg,fileName,figTitle)
    idx = indicesForIds(EXP,ids);
    if any(cellfun(@isempty,outputs(idx)))
        warning('Confronto %s saltato: una simulazione non e'' disponibile.',fileName);
        return
    end

    f = newFigure(cfg);
    tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

    nexttile; hold on; grid on; yline(0,'k--','HandleVisibility','off');
    for i = 1:numel(idx)
        logs = getLogsout(outputs{idx(i)});
        [ea,~] = errorSignals(logs,family,SIG);
        plotSemanticTrace(ea.Time,rad2deg(ea.Data),labels{i},1.5);
    end
    ylabel('$e_\alpha$ [deg]','Interpreter','latex');
    title('Pitch tracking error','Interpreter','latex');
    legend('Location','best');

    nexttile; hold on; grid on; yline(0,'k--','HandleVisibility','off');
    for i = 1:numel(idx)
        logs = getLogsout(outputs{idx(i)});
        [~,eb] = errorSignals(logs,family,SIG);
        plotSemanticTrace(eb.Time,rad2deg(eb.Data),labels{i},1.5);
    end
    xlabel('Time [s]','Interpreter','latex');
    ylabel('$e_\beta$ [deg]','Interpreter','latex');
    title('Yaw tracking error','Interpreter','latex');
    legend('Location','best');

    sgtitle(figTitle,'Interpreter','none');
    exportFigure(f,fullfile(outDir,[fileName '.png']),cfg);
end

function compareControls(outputs,EXP,ids,labels,family,SIG,outDir,cfg,fileName,figTitle)
    idx = indicesForIds(EXP,ids);
    if any(cellfun(@isempty,outputs(idx)))
        warning('Confronto %s saltato: una simulazione non e'' disponibile.',fileName);
        return
    end

    f = newFigure(cfg);
    tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

    nexttile; hold on; grid on;
    for i = 1:numel(idx)
        logs = getLogsout(outputs{idx(i)});
        [u1,~] = controlSignals(logs,family,SIG);
        plotSemanticTrace(u1.Time,u1.Data,labels{i},1.5);
    end
    ylabel('$\Delta F_{1,cmd}$ [N]','Interpreter','latex');
    title('Main-rotor command','Interpreter','latex');
    legend('Location','best');

    nexttile; hold on; grid on;
    for i = 1:numel(idx)
        logs = getLogsout(outputs{idx(i)});
        [~,u2] = controlSignals(logs,family,SIG);
        plotSemanticTrace(u2.Time,u2.Data,labels{i},1.5);
    end
    xlabel('Time [s]','Interpreter','latex');
    ylabel('$\Delta F_{2,cmd}$ [N]','Interpreter','latex');
    title('Tail-rotor command','Interpreter','latex');
    legend('Location','best');

    sgtitle(figTitle,'Interpreter','none');
    exportFigure(f,fullfile(outDir,[fileName '.png']),cfg);
end

function h = plotSemanticTrace(x,y,labelText,lineWidth)
%PLOTSEMANTICTRACE Usa il colore globale quando labelText e' un controllore.
    c = controller_plot_color(labelText);
    if isempty(c)
        h = plot(x,y,'LineWidth',lineWidth,'DisplayName',labelText);
    else
        h = plot(x,y, ...
            'LineWidth',lineWidth, ...
            'Color',c, ...
            'DisplayName',labelText);
    end
end

function [aRef,bRef,a,b] = trackingSignals(logs,family,SIG)
    aRef = tsFromLogs(logs,SIG.alphaRef);
    bRef = tsFromLogs(logs,SIG.betaRef);
    if strcmpi(family,'LQG')
        a = tsFromLogs(logs,SIG.alphaLQG);
        b = tsFromLogs(logs,SIG.betaLQG);
    else
        a = tsFromLogs(logs,SIG.alphaRob);
        b = tsFromLogs(logs,SIG.betaRob);
    end
end

function [ea,eb] = errorSignals(logs,family,SIG)
    if strcmpi(family,'LQG')
        ea = tsFromLogs(logs,SIG.eAlphaLQG);
        eb = tsFromLogs(logs,SIG.eBetaLQG);
    else
        ea = tsFromLogs(logs,SIG.eAlphaRob);
        eb = tsFromLogs(logs,SIG.eBetaRob);
    end
end

function [u1,u2] = controlSignals(logs,family,SIG)
    if strcmpi(family,'LQG')
        u1 = tsFromLogs(logs,SIG.u1cmdLQG);
        u2 = tsFromLogs(logs,SIG.u2cmdLQG);
    else
        u1 = tsFromLogs(logs,SIG.u1cmdRob);
        u2 = tsFromLogs(logs,SIG.u2cmdRob);
    end
end

function [u1,u2] = actuatorSignals(logs,family,SIG)
    if strcmpi(family,'LQG')
        u1 = tsFromLogs(logs,SIG.u1actLQG);
        u2 = tsFromLogs(logs,SIG.u2actLQG);
    else
        u1 = tsFromLogs(logs,SIG.u1actRob);
        u2 = tsFromLogs(logs,SIG.u2actRob);
    end
end

function [al,bl,an,bn] = linearNonlinearSignals(logs,family,SIG)
    if strcmpi(family,'LQG')
        al = tsFromLogs(logs,SIG.alphaLinLQG);
        bl = tsFromLogs(logs,SIG.betaLinLQG);
        an = tsFromLogs(logs,SIG.alphaNLLQG);
        bn = tsFromLogs(logs,SIG.betaNLLQG);
    else
        al = tsFromLogs(logs,SIG.alphaLinRob);
        bl = tsFromLogs(logs,SIG.betaLinRob);
        an = tsFromLogs(logs,SIG.alphaNLRob);
        bn = tsFromLogs(logs,SIG.betaNLRob);
    end
end

function ts = tsFromLogs(logs,name)
    el = logs.get(name);
    if isempty(el)
        available = string(logs.getElementNames);
        error('Segnale "%s" non trovato in logsout. Disponibili: %s', ...
            name,strjoin(available,', '));
    end

    values = el.Values;
    if isa(values,'timeseries')
        ts = values;
    elseif isa(values,'Simulink.Timeseries')
        ts = timeseries(values.Data,values.Time);
    else
        try
            ts = timeseries(values.Data,values.Time);
        catch
            error('Formato non supportato per il segnale "%s".',name);
        end
    end

    ts.Data = squeeze(ts.Data);
end

function f = newFigure(cfg)
    vis = 'off';
    if cfg.showFigures
        vis = 'on';
    end
    f = figure('Visible',vis,'Color','w','Position',[100 100 1050 700]);
end

function exportFigure(f,path,cfg)
    exportgraphics(f,path,'Resolution',cfg.pngResolution);
    close(f);
end

function idx = indicesForIds(EXP,ids)
    idx = zeros(1,numel(ids));
    allIds = string({EXP.id});
    for k = 1:numel(ids)
        q = find(allIds == string(ids{k}),1);
        if isempty(q)
            error('Esperimento "%s" non trovato.',ids{k});
        end
        idx(k) = q;
    end
end

function warnMissing(what,id,ME)
    warning('Plot %s non creato per %s: %s',what,id,ME.message);
end

function runFirstExistingScriptInBase(names)
%RUNFIRSTEXISTINGSCRIPTINBASE Esegue il primo script esistente nel Base WS.
%
% Il file viene letto con FILEREAD e valutato direttamente nel Base
% Workspace. Questo evita sia il problema dei workspace locali sia
% qualsiasi conflitto con un file chiamato run.m.

    for k = 1:numel(names)
        scriptFile = char(names{k});

        if isfile(scriptFile)
            fullName = which(scriptFile);
            if isempty(fullName)
                fullName = fullfile(pwd,scriptFile);
            end

            fprintf('Esecuzione script nel Base Workspace: %s\n',fullName);

            code = fileread(fullName);
            evalin('base',code);
            return
        end
    end

    error('Nessuno degli script richiesti trovato: %s',strjoin(names,', '));
end

function ensureSimulationProfilesInBase()
%ENSURESIMULATIONPROFILESINBASE Garantisce la presenza di ref e aero.

    if ~evalin('base',"exist('alpha0','var')")
        error('alpha0 non esiste dopo l''inizializzazione.');
    end
    if ~evalin('base',"exist('beta0','var')")
        error('beta0 non esiste dopo l''inizializzazione.');
    end

    alpha0 = evalin('base','alpha0');
    beta0  = evalin('base','beta0');

    % ---- Riferimento -----------------------------------------------------
    if evalin('base',"exist('ref','var')")
        ref = evalin('base','ref');
    else
        ref = struct;
    end

    if ~isfield(ref,'alpha') || ~isstruct(ref.alpha)
        ref.alpha = struct;
    end
    if ~isfield(ref,'beta') || ~isstruct(ref.beta)
        ref.beta = struct;
    end

    if ~isfield(ref.alpha,'initial'), ref.alpha.initial = alpha0; end
    if ~isfield(ref.alpha,'final'),   ref.alpha.final   = alpha0 + deg2rad(3); end
    if ~isfield(ref.alpha,'time'),    ref.alpha.time    = 2; end

    if ~isfield(ref.beta,'initial'),  ref.beta.initial  = beta0; end
    if ~isfield(ref.beta,'final'),    ref.beta.final    = beta0; end
    if ~isfield(ref.beta,'time'),     ref.beta.time     = 2; end

    assignin('base','ref',ref);

    % ---- Disturbi aerodinamici -----------------------------------------
    if evalin('base',"exist('aero','var')")
        aero = evalin('base','aero');
    else
        aero = struct;
    end

    if ~isfield(aero,'enable'), aero.enable = 1; end

    if ~isfield(aero,'alpha') || ~isstruct(aero.alpha)
        aero.alpha = struct;
    end
    if ~isfield(aero,'beta') || ~isstruct(aero.beta)
        aero.beta = struct;
    end

    if ~isfield(aero.alpha,'time'),      aero.alpha.time      = 6; end
    if ~isfield(aero.alpha,'amplitude'), aero.alpha.amplitude = 5e-3; end
    if ~isfield(aero.beta,'time'),       aero.beta.time       = 10; end
    if ~isfield(aero.beta,'amplitude'),  aero.beta.amplitude  = 2e-3; end

    assignin('base','aero',aero);

    fprintf('Profili Simulink disponibili: ref e aero verificati.\n');
end

function loadFirstExistingToBase(names)
%LOADFIRSTEXISTINGTOBASE Carica tutte le variabili di un MAT nel Base WS.

    for k = 1:numel(names)
        if isfile(names{k})
            fprintf('Caricamento nel Base Workspace: %s\n',names{k});
            S = load(names{k});
            vars = fieldnames(S);

            for j = 1:numel(vars)
                assignin('base',vars{j},S.(vars{j}));
            end
            return
        end
    end

    error('Nessuno dei file MAT richiesti trovato: %s',strjoin(names,', '));
end

function value = getBaseVariable(name)
%GETBASEVARIABLE Restituisce una variabile dal Base Workspace.

    if ~evalin('base',sprintf("exist('%s','var')",name))
        error('Variabile mancante nel Base Workspace: %s',name);
    end

    value = evalin('base',name);
end

function ensureKalmanObserverInBase()
%ENSUREKALMANOBSERVERINBASE Crea la realizzazione del Kalman observer.
%
% La realizzazione e' coerente con LQG_2DOF_Synthesis:
%   Aobs = A - Ke*Cmeas
%   Bobs = [B Ke]
%   Cobs = I
%   Dobs = 0
%   xhat0 = 0

    observerVars = {'Aobs','Bobs','Cobs','Dobs','xhat0'};
    allPresent = true;

    for k = 1:numel(observerVars)
        allPresent = allPresent && ...
            logical(evalin('base',sprintf("exist('%s','var')",observerVars{k})));
    end

    if allPresent
        fprintf('Kalman observer gia'' disponibile nel Base Workspace.\n');
        return
    end

    needed = {'A','B','Cmeas','Ke'};
    missing = {};

    for k = 1:numel(needed)
        if ~evalin('base',sprintf("exist('%s','var')",needed{k}))
            missing{end+1} = needed{k}; %#ok<AGROW>
        end
    end

    if ~isempty(missing)
        error(['Impossibile costruire Aobs/Bobs/Cobs/Dobs/xhat0. ', ...
               'Mancano: %s'],strjoin(missing,', '));
    end

    A     = evalin('base','A');
    B     = evalin('base','B');
    Cmeas = evalin('base','Cmeas');
    Ke    = evalin('base','Ke');

    Aobs  = A - Ke*Cmeas;
    Bobs  = [B Ke];
    Cobs  = eye(size(A));
    Dobs  = zeros(size(A,1),size(B,2)+size(Cmeas,1));
    xhat0 = zeros(size(A,1),1);

    assignin('base','Aobs',Aobs);
    assignin('base','Bobs',Bobs);
    assignin('base','Cobs',Cobs);
    assignin('base','Dobs',Dobs);
    assignin('base','xhat0',xhat0);

    fprintf('Kalman observer ricostruito nel Base Workspace.\n');
end

function validateBaseWorkspaceForCampaign()
%VALIDATEBASEWORKSPACEFORCAMPAIGN Verifica le variabili essenziali.

    required = { ...
        'p0','act','sensor','ref','aero','alpha0','beta0', ...
        'q0','qdot0','delta_plant','u0','u0_nominal','u0_uncertain', ...
        'P_nominal_ext','P_uncertain_ext', ...
        'Aobs','Bobs','Cobs','Dobs','xhat0', ...
        'K_LQG','K_LQGI','K_mix','K_hinfsyn','K_pidcomp','K_mu','K_h2'};

    missing = {};

    for k = 1:numel(required)
        if ~evalin('base',sprintf("exist('%s','var')",required{k}))
            missing{end+1} = required{k}; %#ok<AGROW>
        end
    end

    if ~isempty(missing)
        error(['Campagna Simulink non inizializzata correttamente. ', ...
               'Variabili mancanti nel Base Workspace: %s'], ...
               strjoin(missing,', '));
    end

    fprintf('Verifica Base Workspace completata: tutte le variabili richieste sono presenti.\n');
end


function out = simWithWarningsSuppressed(in)
%SIMWITHWARNINGSSUPPRESSED Esegue sim() senza affollare la console.
%
% I warning vengono disabilitati soltanto per la durata della chiamata a
% sim(). Lo stato precedente viene sempre ripristinato, anche se Simulink
% genera un errore. Gli errori non vengono soppressi.

    previousWarningState = warning;
    cleanupObj = onCleanup(@() warning(previousWarningState)); %#ok<NASGU>
    warning('off','all');

    out = sim(in);
end

function setScopeOpenState(model,state)
    scopes = find_system(model, ...
        'LookUnderMasks','all', ...
        'FollowLinks','on', ...
        'MatchFilter',@Simulink.match.allVariants, ...
        'BlockType','Scope');
    for k = 1:numel(scopes)
        try
            set_param(scopes{k},'Open',state);
        catch
            % Alcuni viewer possono non esporre il parametro Open.
        end
    end
end

function cleanupPreviousCampaign(outDir)
    if ~exist(outDir,'dir')
        return
    end
    pngFiles = dir(fullfile(outDir,'**','*.png'));
    for k = 1:numel(pngFiles)
        delete(fullfile(pngFiles(k).folder,pngFiles(k).name));
    end
    csvFile = fullfile(outDir,'campaign_summary.csv');
    if isfile(csvFile)
        delete(csvFile);
    end
end

function info = findRASBoundaryPair(controllerPattern)
    info = struct('available',false,'controllerName','', ...
                  'inside',[],'outside',[]);

    candidates = { ...
        'RAS_vectorized_results_final_v5.mat', ...
        'RAS_vectorized_results.mat'};

    file = '';
    for k = 1:numel(candidates)
        if isfile(candidates{k})
            file = candidates{k};
            break
        end
    end
    if isempty(file)
        return
    end

    S = load(file);
    required = {'results','alphaGridDeg','betaGridDeg'};
    if ~all(isfield(S,required))
        return
    end

    names = string({S.results.name});
    ic = find(contains(lower(names),lower(string(controllerPattern))),1);
    if isempty(ic)
        return
    end

    if isfield(S.results(ic),'stableMap')
        M = logical(S.results(ic).stableMap);
    elseif isfield(S.results(ic),'stable')
        M = logical(S.results(ic).stable);
        if isvector(M) && numel(M) == numel(S.alphaGridDeg)*numel(S.betaGridDeg)
            M = reshape(M,numel(S.betaGridDeg),numel(S.alphaGridDeg));
        end
    else
        return
    end

    % Compatibilita' orientamento matrice/griglie.
    if size(M,1) == numel(S.alphaGridDeg) && size(M,2) == numel(S.betaGridDeg)
        M = M.';
    end
    if size(M,1) ~= numel(S.betaGridDeg) || size(M,2) ~= numel(S.alphaGridDeg)
        return
    end

    % Cerca una coppia 4-connessa stabile/instabile il piu' vicino possibile
    % all'equilibrio (0,0), cosi' la prova non usa un punto estremo arbitrario.
    bestCost = inf;
    bestIn = [];
    bestOut = [];

    for ib = 1:size(M,1)
        for ia = 1:size(M,2)
            if ~M(ib,ia)
                continue
            end
            neigh = [ib-1 ia; ib+1 ia; ib ia-1; ib ia+1];
            for j = 1:4
                jb = neigh(j,1); ja = neigh(j,2);
                if jb < 1 || jb > size(M,1) || ja < 1 || ja > size(M,2)
                    continue
                end
                if M(jb,ja)
                    continue
                end
                pin = [S.alphaGridDeg(ia),S.betaGridDeg(ib)];
                pout = [S.alphaGridDeg(ja),S.betaGridDeg(jb)];
                cost = norm(pin,2);
                if cost < bestCost
                    bestCost = cost;
                    bestIn = pin;
                    bestOut = pout;
                end
            end
        end
    end

    if isempty(bestIn)
        return
    end

    info.available = true;
    info.controllerName = char(names(ic));
    info.inside = bestIn;
    info.outside = bestOut;
end
