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
%                           3 = PID+comp, 4 = mu-synthesis
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

% Verifica preventiva: meglio fermarsi qui che produrre 13 errori Simulink.
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
% robust ctl: 1 mixsyn, 2 hinfsyn, 3 PID+comp, 4 mu-synthesis

EXP = repmat(emptyExperiment(),0,1);

% ---- LQG / LQGI nominali -------------------------------------------------
EXP(end+1) = makeExp('01_LQG_nominal_NL', ...
    'LQG nominale - plant non lineare', ...
    'LQG',2,2,1,refCombined,aeroOff,deltaNominal);

EXP(end+1) = makeExp('02_LQGI_nominal_NL', ...
    'LQGI nominale - plant non lineare', ...
    'LQG',2,1,1,refCombined,aeroOff,deltaNominal);

% ---- Motivazione dell'integratore ---------------------------------------
EXP(end+1) = makeExp('03_LQG_disturbance_NL', ...
    'LQG - reiezione disturbo costante', ...
    'LQG',2,2,1,refCombined,aeroStep,deltaNominal);

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

% ---- Confronto H-infinity ------------------------------------------------
EXP(end+1) = makeExp('07_mixsyn_nominal_NL', ...
    'mixsyn - nominale non lineare', ...
    'ROB',2,1,1,refCombined,aeroOff,deltaNominal);

EXP(end+1) = makeExp('08_hinfsyn_nominal_NL', ...
    'hinfsyn - nominale non lineare', ...
    'ROB',2,1,2,refCombined,aeroOff,deltaNominal);

EXP(end+1) = makeExp('09_PIDcomp_nominal_NL', ...
    'hinfstruct PID + compensatore - nominale non lineare', ...
    'ROB',2,1,3,refCombined,aeroOff,deltaNominal);

% ---- H-infinity vs mu su plant incerto ----------------------------------
EXP(end+1) = makeExp('10_mixsyn_uncertain_NL', ...
    'mixsyn - plant incerto non lineare con disturbo', ...
    'ROB',2,1,1,refCombined,aeroStep,deltaRobustTest);

EXP(end+1) = makeExp('11_mu_uncertain_NL', ...
    'mu-synthesis - plant incerto non lineare con disturbo', ...
    'ROB',2,1,4,refCombined,aeroStep,deltaRobustTest);

%% ========================================================================
% 5. EVENTUALI PUNTI RAS AUTOMATICI
% =========================================================================
% Se esiste il MAT prodotto da RAS_vectorized.m, vengono scelti per il
% controllore mu-synthesis due punti adiacenti al bordo della mappa numerica:
% uno classificato interno e uno esterno rispetto al criterio a orizzonte finito.
% Le relative simulazioni Simulink vengono aggiunte alla campagna per verificare
% se il punto esterno sia realmente divergente oppure semplicemente piu' lento.

rasInfo = findRASBoundaryPair();

if rasInfo.available
    fprintf('\nRAS: trovata coppia automatica vicino al bordo per %s.\n', ...
        rasInfo.controllerName);
    fprintf('  interno:  dAlpha = %.3f deg, dBeta = %.3f deg\n', ...
        rasInfo.inside(1),rasInfo.inside(2));
    fprintf('  esterno:  dAlpha = %.3f deg, dBeta = %.3f deg\n', ...
        rasInfo.outside(1),rasInfo.outside(2));

    eIn = makeExp('12_RAS_inside_mu', ...
        'mu-synthesis - punto interno alla mappa RAS numerica', ...
        'ROB',2,1,4,refEquilibrium,aeroOff,deltaNominal);
    eIn.q0Override = [ ...
        alpha0 + deg2rad(rasInfo.inside(1)); ...
        beta0  + deg2rad(rasInfo.inside(2))];
    eIn.stopTime = cfg.rasValidationStopTime;
    EXP(end+1) = eIn;

    eOut = makeExp('13_RAS_outside_mu', ...
        'mu-synthesis - punto appena esterno alla mappa RAS numerica', ...
        'ROB',2,1,4,refEquilibrium,aeroOff,deltaNominal);
    eOut.q0Override = [ ...
        alpha0 + deg2rad(rasInfo.outside(1)); ...
        beta0  + deg2rad(rasInfo.outside(2))];
    eOut.stopTime = cfg.rasValidationStopTime;
    EXP(end+1) = eOut;
else
    fprintf(['\nRAS: nessuna mappa RAS compatibile trovata. ', ...
             'Le prove 12-13 vengono saltate.\n']);
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

% 8.4 mixsyn vs hinfsyn vs PID+comp
compareRuns(outputs,EXP, ...
    {'07_mixsyn_nominal_NL','08_hinfsyn_nominal_NL','09_PIDcomp_nominal_NL'}, ...
    {'mixsyn','hinfsyn','PID + compensatore'},'ROB',SIG,comparisonDir,cfg, ...
    '04_HINF_controllers_tracking', ...
    'Confronto dei controllori H-infinity sul modello non lineare');

compareControls(outputs,EXP, ...
    {'07_mixsyn_nominal_NL','08_hinfsyn_nominal_NL','09_PIDcomp_nominal_NL'}, ...
    {'mixsyn','hinfsyn','PID + compensatore'},'ROB',SIG,comparisonDir,cfg, ...
    '05_HINF_controllers_control_effort', ...
    'Confronto dello sforzo di controllo H-infinity');

% 8.5 mixsyn vs mu su stesso plant incerto
compareRuns(outputs,EXP, ...
    {'10_mixsyn_uncertain_NL','11_mu_uncertain_NL'}, ...
    {'mixsyn','mu-synthesis'},'ROB',SIG,comparisonDir,cfg, ...
    '06_mixsyn_vs_mu_uncertain_tracking', ...
    'Confronto robusto su plant non lineare incerto');

compareControls(outputs,EXP, ...
    {'10_mixsyn_uncertain_NL','11_mu_uncertain_NL'}, ...
    {'mixsyn','mu-synthesis'},'ROB',SIG,comparisonDir,cfg, ...
    '07_mixsyn_vs_mu_uncertain_control_effort', ...
    'Sforzo di controllo sul plant incerto');

% 8.6 RAS: interno vs esterno, se disponibili
if rasInfo.available
    compareRuns(outputs,EXP, ...
        {'12_RAS_inside_mu','13_RAS_outside_mu'}, ...
        {'Interno al bordo','Esterno al bordo'},'ROB',SIG,comparisonDir,cfg, ...
        '08_RAS_inside_vs_outside_mu', ...
        'Validazione a lungo termine del bordo della mappa RAS numerica');
end

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
        'stopTime',NaN);
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

    % Linear vs nonlinear: entrambi sono simulati in parallelo nel modello.
    try
        [al,bl,an,bn] = linearNonlinearSignals(logs,family,SIG);
        f = newFigure(cfg);
        tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

        nexttile;
        plot(al.Time,rad2deg(al.Data),'--','LineWidth',1.3); hold on;
        plot(an.Time,rad2deg(an.Data),'LineWidth',1.5); grid on;
        ylabel('$\alpha$ [deg]','Interpreter','latex');
        title('Pitch: linearized vs nonlinear model','Interpreter','latex');
        legend({'Linearized','Nonlinear'},'Location','best');

        nexttile;
        plot(bl.Time,rad2deg(bl.Data),'--','LineWidth',1.3); hold on;
        plot(bn.Time,rad2deg(bn.Data),'LineWidth',1.5); grid on;
        xlabel('Time [s]','Interpreter','latex');
        ylabel('$\beta$ [deg]','Interpreter','latex');
        title('Yaw: linearized vs nonlinear model','Interpreter','latex');
        legend({'Linearized','Nonlinear'},'Location','best');

        sgtitle(exp.description,'Interpreter','none');
        exportFigure(f,fullfile(outDir,'06_linear_vs_nonlinear.png'),cfg);

        % Errore di linearizzazione con interpolazione sulla griglia NL.
        alI = interp1(al.Time,al.Data,an.Time,'linear','extrap');
        blI = interp1(bl.Time,bl.Data,bn.Time,'linear','extrap');

        f = newFigure(cfg);
        tiledlayout(2,1,'TileSpacing','compact','Padding','compact');
        nexttile;
        plot(an.Time,rad2deg(an.Data-alI),'LineWidth',1.5); grid on; yline(0,'--');
        ylabel('$\alpha_{NL}-\alpha_{LIN}$ [deg]','Interpreter','latex');
        title('Pitch linearization error','Interpreter','latex');
        nexttile;
        plot(bn.Time,rad2deg(bn.Data-blI),'LineWidth',1.5); grid on; yline(0,'--');
        xlabel('Time [s]','Interpreter','latex');
        ylabel('$\beta_{NL}-\beta_{LIN}$ [deg]','Interpreter','latex');
        title('Yaw linearization error','Interpreter','latex');
        sgtitle(exp.description,'Interpreter','none');
        exportFigure(f,fullfile(outDir,'07_linearization_error.png'),cfg);
    catch ME
        warnMissing('linear/nonlinear comparison',exp.id,ME);
    end
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
        plot(a.Time,rad2deg(a.Data),'LineWidth',1.5,'DisplayName',labels{i});
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
        plot(b.Time,rad2deg(b.Data),'LineWidth',1.5,'DisplayName',labels{i});
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
        plot(ea.Time,rad2deg(ea.Data),'LineWidth',1.5,'DisplayName',labels{i});
    end
    ylabel('$e_\alpha$ [deg]','Interpreter','latex');
    title('Pitch tracking error','Interpreter','latex');
    legend('Location','best');

    nexttile; hold on; grid on; yline(0,'k--','HandleVisibility','off');
    for i = 1:numel(idx)
        logs = getLogsout(outputs{idx(i)});
        [~,eb] = errorSignals(logs,family,SIG);
        plot(eb.Time,rad2deg(eb.Data),'LineWidth',1.5,'DisplayName',labels{i});
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
        plot(u1.Time,u1.Data,'LineWidth',1.5,'DisplayName',labels{i});
    end
    ylabel('$\Delta F_{1,cmd}$ [N]','Interpreter','latex');
    title('Main-rotor command','Interpreter','latex');
    legend('Location','best');

    nexttile; hold on; grid on;
    for i = 1:numel(idx)
        logs = getLogsout(outputs{idx(i)});
        [~,u2] = controlSignals(logs,family,SIG);
        plot(u2.Time,u2.Data,'LineWidth',1.5,'DisplayName',labels{i});
    end
    xlabel('Time [s]','Interpreter','latex');
    ylabel('$\Delta F_{2,cmd}$ [N]','Interpreter','latex');
    title('Tail-rotor command','Interpreter','latex');
    legend('Location','best');

    sgtitle(figTitle,'Interpreter','none');
    exportFigure(f,fullfile(outDir,[fileName '.png']),cfg);
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
        'q0','qdot0','delta_plant','u0', ...
        'Aobs','Bobs','Cobs','Dobs','xhat0', ...
        'K_LQG','K_LQGI','K_mix','K_hinfsyn','K_pidcomp','K_mu'};

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

function info = findRASBoundaryPair()
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
    ic = find(contains(lower(names),'mu'),1);
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

    % Cerca una coppia 4-connessa interno/esterno rispetto al criterio il piu' vicino possibile
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
