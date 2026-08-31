%% NOTE TEORICHE - REGIONE DI STABILITA' ASINTOTICA
% La RAS viene stimata numericamente su un orizzonte finito. Un punto e'
% classificato nella regione se, senza violare i limiti globali, rimane
% entro le tolleranze di stato durante la coda finale della simulazione.
% La mappa e' quindi una stima numerica conservativa della regione di
% attrazione, dipendente da tFinal e dalle tolleranze dichiarate.
%

%% RAS_VECTORIZED_FINAL_V5
% =========================================================================
% RAS numerica vettorializzata - closed-loop CONTINUO
%
% V4:
%   - plant non lineare: RK4 vettorializzato;
%   - controller lineari: propagazione ESATTA via matrice esponenziale
%     per ingresso sensor-ZOH costante nel passo;
%   - motori EMAX: RK4 continuo, pilotati dal comando del controller
%     valutato a t, t+h/2 e t+h;
%   - saturazione a ogni stadio del motore;
%   - sensori VN-100: ZOH;
%   - transport delay: buffer campionato corretto ai sample time.
%
% La propagazione esatta dei controller evita l'instabilità numerica
% dell'RK4 esplicito sul controllore mu di ordine elevato.
%
% MATLAB R2025a / R2026a.
% =========================================================================
close all;
clc;

%% ========================================================================
% 0. CONFIGURAZIONE
% ========================================================================
cfg.alphaRangeDeg = [-95 65];
cfg.betaRangeDeg  = [-180 140];
cfg.nAlpha = 161;
cfg.nBeta  = 241;
cfg.tFinal = 15;             % [s] orizzonte della stima numerica RAS
% 2.5 ms divide esattamente il delay nominale di 15 ms.
% Dopo la prima validazione si può usare 1 ms per uno studio di convergenza.
cfg.dt = 0.001;             % [s]
cfg.tailFraction = 0.10;
cfg.angleToleranceDeg = 0.25;
cfg.rateToleranceDeg  = 0.50;
cfg.maxAlphaDeg = 85;
cfg.maxBetaDeg  = 170;
cfg.earlyReject = true;
cfg.saveMAT = true;
cfg.saveFigures = false;
cfg.controllerNames = { ...
    'LQG', ...
    'LQGI', ...
    'mixsyn', ...
    'hinfsyn', ...
    'PID+comp', ...
    'mu-synthesis', ...
    'H2'};

%% ========================================================================
% 1. INIZIALIZZAZIONE DEL PROGETTO
% ========================================================================
if ~exist('p0','var') || ~exist('act','var') || ...
        ~exist('sensor','var') || ~exist('alpha0','var') || ...
        ~exist('beta0','var') || ~exist('u0','var')
    if isfile('init.m')
        init;
    else
        error('Manca init.m oppure mancano le variabili di inizializzazione.');
    end
end
% RAS deterministica
sensor.noiseEnable = 0;
act.noiseEnable = 0;
xeq = [alpha0; 0; beta0; 0];

%% ========================================================================
% 2. CARICAMENTO DEI CONTROLLORI CONTINUI
% ========================================================================
controllers = loadControllers(cfg.controllerNames);
fprintf('\n============================================================\n');
fprintf('RAS FINAL V5 - CONTROLLORI CARICATI\n');
fprintf('============================================================\n');
for k = 1:numel(controllers)
    fprintf('%-16s ordine = %d, ingressi = %d, uscite = %d\n', ...
        controllers(k).name, ...
        size(controllers(k).A,1), ...
        size(controllers(k).B,2), ...
        size(controllers(k).C,1));
end
% Transizioni esatte del controller per ingresso costante nel passo.
% I sensori sono ZOH, quindi uk è effettivamente costante tra due update.
for k = 1:numel(controllers)
    nk = size(controllers(k).A,1);
    if nk > 0
        [controllers(k).Phi,controllers(k).Gamma] = ...
            exactZOH(controllers(k).A,controllers(k).B,cfg.dt);
        [controllers(k).PhiHalf,controllers(k).GammaHalf] = ...
            exactZOH(controllers(k).A,controllers(k).B,cfg.dt/2);
    else
        controllers(k).Phi = zeros(0);
        controllers(k).Gamma = zeros(0,size(controllers(k).B,2));
        controllers(k).PhiHalf = zeros(0);
        controllers(k).GammaHalf = zeros(0,size(controllers(k).B,2));
    end
end

%% ========================================================================
% 3. VN-100 / WLS PER I CONTROLLORI H-INFINITY
% ========================================================================
g0 = p0.g;
B0 = sensor.mag.B0;
Jsensor = [
    g0*cos(alpha0),             0;
    0,             -B0*sin(beta0);
    0,             -B0*cos(beta0)
];
V_sensor = diag([
    sensor.acc.var
    sensor.mag.var
    sensor.mag.var
]);
Vinv = diag(1./diag(V_sensor));
Hy = (Jsensor.'*Vinv*Jsensor) \ (Jsensor.'*Vinv);
y0_sensor = [
    g0*sin(alpha0);
    B0*cos(beta0);
   -B0*sin(beta0)
];

%% ========================================================================
% 4. DINAMICA CONTINUA DEI MOTORI E TRANSPORT DELAY
% ========================================================================
Am1 = [
    0, 1;
   -act.wn1^2, -2*act.zeta1*act.wn1
];
Bm1 = [0; act.wn1^2];
Cm1 = [1 0];
Am2 = [
    0, 1;
   -act.wn2^2, -2*act.zeta2*act.wn2
];
Bm2 = [0; act.wn2^2];
Cm2 = [1 0];
delaySteps1 = round(act.td1/cfg.dt);
delaySteps2 = round(act.td2/cfg.dt);
delayError1 = abs(delaySteps1*cfg.dt-act.td1);
delayError2 = abs(delaySteps2*cfg.dt-act.td2);
if delayError1 > 1e-12 || delayError2 > 1e-12
    warning(['cfg.dt non divide esattamente il transport delay. ', ...
             'Il solo ritardo viene quantizzato alla griglia temporale.']);
end
delaySteps1 = max(delaySteps1,1);
delaySteps2 = max(delaySteps2,1);
fprintf('\nTransport delay 1: %d campioni = %.6f s\n', ...
    delaySteps1,delaySteps1*cfg.dt);
fprintf('Transport delay 2: %d campioni = %.6f s\n', ...
    delaySteps2,delaySteps2*cfg.dt);

%% ========================================================================
% 5. GRIGLIA REGOLARE DELLE CONDIZIONI INIZIALI
% ========================================================================
alphaGridDeg = linspace( ...
    cfg.alphaRangeDeg(1),cfg.alphaRangeDeg(2),cfg.nAlpha);
betaGridDeg = linspace( ...
    cfg.betaRangeDeg(1),cfg.betaRangeDeg(2),cfg.nBeta);
[DAdeg,DBdeg] = meshgrid(alphaGridDeg,betaGridDeg);
deltaAlpha0 = deg2rad(DAdeg(:)).';
deltaBeta0  = deg2rad(DBdeg(:)).';
Npoints = numel(deltaAlpha0);
fprintf('\n============================================================\n');
fprintf('GRIGLIA RAS FINAL V5\n');
fprintf('============================================================\n');
fprintf('alpha: %d punti\n',cfg.nAlpha);
fprintf('beta : %d punti\n',cfg.nBeta);
fprintf('totale per controllore: %d traiettorie\n',Npoints);
fprintf('Integrazione closed-loop vettorializzata continua.\n');

%% ========================================================================
% 6. SOGLIE
% ========================================================================
angleTol = deg2rad(cfg.angleToleranceDeg);
rateTol  = deg2rad(cfg.rateToleranceDeg);
maxAlpha = deg2rad(cfg.maxAlphaDeg);
maxBeta  = deg2rad(cfg.maxBetaDeg);
tailStartTime = cfg.tFinal*(1-cfg.tailFraction);
Nsteps = ceil(cfg.tFinal/cfg.dt);

%% ========================================================================
% 7. CALCOLO RAS
% ========================================================================
results = struct([]);
for ic = 1:numel(controllers)
    Ctl = controllers(ic);
    fprintf('\n============================================================\n');
    fprintf('RAS FINAL V5: %s\n',upper(Ctl.name));
    fprintf('============================================================\n');
    tic;
    % Stato fisico: [alpha; alpha_dot; beta; beta_dot]
    X = repmat(xeq,1,Npoints);
    X(1,:) = alpha0 + deltaAlpha0;
    X(3,:) = beta0  + deltaBeta0;
    % Stato continuo del controllore
    nk = size(Ctl.A,1);
    Xk = zeros(nk,Npoints);
    % Stati continui dei motori
    Xm1 = zeros(2,Npoints);
    Xm2 = zeros(2,Npoints);
    % Delay buffer dell'uscita dei motori
    % +1 perché il plant al tempo t_n deve leggere y(t_n-td), non
    % il campione già avanzato di un passo.
    delayBuffer1 = zeros(delaySteps1+1,Npoints);
    delayBuffer2 = zeros(delaySteps2+1,Npoints);
    delayPtr1 = 1;
    delayPtr2 = 1;
    % Sensori campionati/ZOH
    alphaHold = X(1,:);
    betaHold  = X(3,:);
    accEvery = max(1,round(sensor.acc.Ts/cfg.dt));
    magEvery = max(1,round(sensor.mag.Ts/cfg.dt));
    active = true(1,Npoints);
    rejected = false(1,Npoints);
    tailMax = zeros(4,Npoints);
    tailStartErrorNorm = nan(1,Npoints);
    maxAbsAlpha = abs(X(1,:));
    maxAbsBeta  = abs(X(3,:));
    maxAbsCommand1 = zeros(1,Npoints);
    maxAbsCommand2 = zeros(1,Npoints);
    for istep = 1:Nsteps
        if ~any(active)
            break
        end
        idx = find(active);
        % -------------------------------------------------------------
        % Sensori deterministici con ZOH
        % -------------------------------------------------------------
        if mod(istep-1,accEvery) == 0
            alphaHold(idx) = X(1,idx);
        end
        if mod(istep-1,magEvery) == 0
            betaHold(idx) = X(3,idx);
        end
        y = [
            p0.g*sin(alphaHold(idx));
            B0*cos(betaHold(idx));
           -B0*sin(betaHold(idx))
        ];
        deltaY = y - y0_sensor;
        % -------------------------------------------------------------
        % Ingresso al controller, mantenuto costante durante il passo RK4
        % perché proviene dai blocchi ZOH dei sensori.
        % -------------------------------------------------------------
        if strcmp(Ctl.family,'LQG')
            uk = [
                zeros(2,numel(idx));
                deltaY
            ];
        else
            deltaAngleEstimated = Hy*deltaY;
            uk = -deltaAngleEstimated;
        end
        % -------------------------------------------------------------
        % Forze DELAYED viste dal plant durante il passo corrente.
        % Il transport delay rende questa uscita già dipendente dal passato.
        % -------------------------------------------------------------
        deltaF1Delayed = delayBuffer1(delayPtr1,idx);
        deltaF2Delayed = delayBuffer2(delayPtr2,idx);
        Fdelayed = [
            u0(1) + deltaF1Delayed;
            u0(2) + deltaF2Delayed
        ];
        % -------------------------------------------------------------
        % RK4 DEL PLANT NON LINEARE
        % -------------------------------------------------------------
        Xi = X(:,idx);
        p1 = helicopterDerivativeVectorized(Xi,Fdelayed,p0);
        p2 = helicopterDerivativeVectorized( ...
            Xi + 0.5*cfg.dt*p1,Fdelayed,p0);
        p3 = helicopterDerivativeVectorized( ...
            Xi + 0.5*cfg.dt*p2,Fdelayed,p0);
        p4 = helicopterDerivativeVectorized( ...
            Xi + cfg.dt*p3,Fdelayed,p0);
        X(:,idx) = Xi + ...
            (cfg.dt/6)*(p1 + 2*p2 + 2*p3 + p4);
        % -------------------------------------------------------------
        % CONTROLLER: PROPAGAZIONE ESATTA; MOTORI: RK4 CONTINUO
        %
        % uk è costante nel passo per effetto degli ZOH sensore.
        % Calcoliamo esattamente lo stato del controller a:
        %   t, t+h/2, t+h
        % e usiamo i relativi comandi per gli stadi RK4 dei motori.
        % -------------------------------------------------------------
        Xk0 = Xk(:,idx);
        if nk > 0
            XkHalf = ...
                Ctl.PhiHalf*Xk0 + ...
                Ctl.GammaHalf*uk;
            XkEnd = ...
                Ctl.Phi*Xk0 + ...
                Ctl.Gamma*uk;
        else
            XkHalf = zeros(0,numel(idx));
            XkEnd  = zeros(0,numel(idx));
        end
        [cmd1_1,cmd2_1] = controllerCommand(Xk0,uk,Ctl,act);
        [cmd1_2,cmd2_2] = controllerCommand(XkHalf,uk,Ctl,act);
        [cmd1_4,cmd2_4] = controllerCommand(XkEnd,uk,Ctl,act);
        % Motore 1
        m1_1 = Am1*Xm1(:,idx) + Bm1*cmd1_1;
        m1_2 = Am1*(Xm1(:,idx)+0.5*cfg.dt*m1_1) + Bm1*cmd1_2;
        m1_3 = Am1*(Xm1(:,idx)+0.5*cfg.dt*m1_2) + Bm1*cmd1_2;
        m1_4 = Am1*(Xm1(:,idx)+cfg.dt*m1_3)     + Bm1*cmd1_4;
        Xm1(:,idx) = Xm1(:,idx) + ...
            (cfg.dt/6)*(m1_1 + 2*m1_2 + 2*m1_3 + m1_4);
        % Motore 2
        m2_1 = Am2*Xm2(:,idx) + Bm2*cmd2_1;
        m2_2 = Am2*(Xm2(:,idx)+0.5*cfg.dt*m2_1) + Bm2*cmd2_2;
        m2_3 = Am2*(Xm2(:,idx)+0.5*cfg.dt*m2_2) + Bm2*cmd2_2;
        m2_4 = Am2*(Xm2(:,idx)+cfg.dt*m2_3)     + Bm2*cmd2_4;
        Xm2(:,idx) = Xm2(:,idx) + ...
            (cfg.dt/6)*(m2_1 + 2*m2_2 + 2*m2_3 + m2_4);
        if nk > 0
            Xk(:,idx) = XkEnd;
        end
        maxAbsCommand1(idx) = max( ...
            maxAbsCommand1(idx), ...
            max([abs(cmd1_1); abs(cmd1_2); abs(cmd1_4)],[],1));
        maxAbsCommand2(idx) = max( ...
            maxAbsCommand2(idx), ...
            max([abs(cmd2_1); abs(cmd2_2); abs(cmd2_4)],[],1));
        % -------------------------------------------------------------
        % Nuova uscita dei motori nel delay buffer
        % -------------------------------------------------------------
        motorOut1 = Cm1*Xm1(:,idx);
        motorOut2 = Cm2*Xm2(:,idx);
        delayBuffer1(delayPtr1,idx) = motorOut1;
        delayBuffer2(delayPtr2,idx) = motorOut2;
        delayPtr1 = delayPtr1 + 1;
        if delayPtr1 > (delaySteps1+1)
            delayPtr1 = 1;
        end
        delayPtr2 = delayPtr2 + 1;
        if delayPtr2 > (delaySteps2+1)
            delayPtr2 = 1;
        end
        % -------------------------------------------------------------
        % Diagnostica / early reject
        % -------------------------------------------------------------
        maxAbsAlpha(idx) = max(maxAbsAlpha(idx),abs(X(1,idx)));
        maxAbsBeta(idx)  = max(maxAbsBeta(idx), abs(X(3,idx)));
        finiteNow = ...
            all(isfinite(X(:,idx)),1) & ...
            all(isfinite(Xm1(:,idx)),1) & ...
            all(isfinite(Xm2(:,idx)),1);
        if nk > 0
            finiteNow = finiteNow & all(isfinite(Xk(:,idx)),1);
        end
        withinNow = ...
            abs(X(1,idx)) < maxAlpha & ...
            abs(X(3,idx)) < maxBeta;
        badLocal = ~finiteNow | ~withinNow;
        if any(badLocal)
            badIdx = idx(badLocal);
            rejected(badIdx) = true;
            if cfg.earlyReject
                active(badIdx) = false;
            end
        end
        % -------------------------------------------------------------
        % Criterio di convergenza nell'ultimo 10 %
        % -------------------------------------------------------------
        tNext = istep*cfg.dt;
        if tNext >= tailStartTime
            idxTail = find(active & ~rejected);
            if ~isempty(idxTail)
                err = abs(X(:,idxTail)-xeq);

                firstTail = isnan(tailStartErrorNorm(idxTail));
                if any(firstTail)
                    idxFirst = idxTail(firstTail);
                    tailStartErrorNorm(idxFirst) = ...
                        vecnorm(X(:,idxFirst)-xeq,2,1);
                end

                tailMax(:,idxTail) = max( ...
                    tailMax(:,idxTail),err);
            end
        end
        if mod(istep,max(1,round(Nsteps/10))) == 0
            fprintf('  %5.1f %% | attive: %d / %d\n', ...
                100*istep/Nsteps,nnz(active),Npoints);
        end
    end

    %% Classificazione finale
    converged = ...
        tailMax(1,:) < angleTol & ...
        tailMax(2,:) < rateTol  & ...
        tailMax(3,:) < angleTol & ...
        tailMax(4,:) < rateTol;
    withinGlobalLimits = ...
        maxAbsAlpha < maxAlpha & ...
        maxAbsBeta  < maxBeta;
    stable = ...
        ~rejected & ...
        withinGlobalLimits & ...
        converged;

    finalErrorNorm = vecnorm(X-xeq,2,1);
    slowConverging = ...
        ~rejected & ...
        withinGlobalLimits & ...
        ~converged & ...
        isfinite(tailStartErrorNorm) & ...
        (finalErrorNorm < tailStartErrorNorm);

    stableMap = reshape(stable,cfg.nBeta,cfg.nAlpha);
    elapsed = toc;
    fprintf('\n%s completato in %.2f s\n',Ctl.name,elapsed);
    fprintf('Punti stabili: %d / %d (%.1f %%)\n', ...
        nnz(stable),Npoints,100*nnz(stable)/Npoints);
    results(ic).name = Ctl.name;
    results(ic).family = Ctl.family;
    results(ic).stable = stable;
    results(ic).stableMap = stableMap;
    results(ic).tailMax = tailMax;
    results(ic).tailStartErrorNorm = tailStartErrorNorm;
    results(ic).finalErrorNorm = finalErrorNorm;
    results(ic).slowConverging = slowConverging;
    results(ic).maxAbsAlpha = maxAbsAlpha;
    results(ic).maxAbsBeta = maxAbsBeta;
    results(ic).maxAbsCommand1 = maxAbsCommand1;
    results(ic).maxAbsCommand2 = maxAbsCommand2;
    results(ic).elapsed = elapsed;
    results(ic).domainFullyStable = all(stable);
    results(ic).domainFullyUnstable = ~any(stable);
end

%% ========================================================================
% 8. PLOT RAS - VERA REGIONE SU GRIGLIA REGOLARE
% ========================================================================
for ic = 1:numel(results)
    Mplot = logical(results(ic).stableMap);
    figure( ...
        'Name',['RAS FINAL V5 - ',results(ic).name], ...
        'Color','w');
    imagesc( ...
        alphaGridDeg, ...
        betaGridDeg, ...
        double(Mplot));
    axis xy
    axis equal
    hold on
    grid on
    box on
    if any(Mplot(:)) && ~all(Mplot(:))
        contour( ...
            alphaGridDeg, ...
            betaGridDeg, ...
            double(Mplot), ...
            [0.5 0.5], ...
            'k', ...
            'LineWidth',1.8);
        domainNote = '';
    elseif all(Mplot(:))
        domainNote = '  [RAS oltre il dominio esplorato]';
    else
        domainNote = '  [nessun punto stabile nel dominio]';
    end
    plot(0,0,'k+','MarkerSize',10,'LineWidth',2);
    xlabel('$\Delta\alpha(0)\;[^{\circ}]$','Interpreter','latex');
    ylabel('$\Delta\beta(0)\;[^{\circ}]$','Interpreter','latex');
    title([ ...
        'Stima numerica della regione di attrazione - ', ...
        results(ic).name, ...
        domainNote], ...
        'Interpreter','latex');
    xlim(cfg.alphaRangeDeg);
    ylim(cfg.betaRangeDeg);
    cb = colorbar;
    cb.Ticks = [0 1];
    cb.TickLabels = {'Fuori dal criterio a T_f','Convergente entro il criterio'};
    if cfg.saveFigures
        exportgraphics( ...
            gcf, ...
            ['RAS_FINAL_V5_',sanitizeName(results(ic).name),'.pdf'], ...
            'ContentType','vector');
    end
end

%% ========================================================================
% 9. FIGURA COMPARATIVA DEI CONFINI
% ========================================================================
figure( ...
    'Name','Confronto confini RAS FINAL V5', ...
    'Color','w');
hold on
grid on
box on
axis equal
hLegend = gobjects(0);

%% Palette globale coerente con gli altri grafici del progetto
for ic = 1:numel(results)
    Mplot = logical(results(ic).stableMap);
    if any(Mplot(:)) && ~all(Mplot(:))
        [~,h] = contour( ...
            alphaGridDeg, ...
            betaGridDeg, ...
            double(Mplot), ...
            [0.5 0.5], ...
            'LineWidth',1.8, ...
            'DisplayName',results(ic).name);
        c = controller_plot_color(results(ic).name);
        if ~isempty(c)
            h.LineColor = c;
        end
        hLegend(end+1) = h; %#ok<SAGROW>
    end
end
plot( ...
    0,0, ...
    'k+', ...
    'MarkerSize',10, ...
    'LineWidth',2, ...
    'HandleVisibility','off');
xlabel('$\Delta\alpha(0)\;[^{\circ}]$','Interpreter','latex');
ylabel('$\Delta\beta(0)\;[^{\circ}]$','Interpreter','latex');
title('Confronto delle stime numeriche delle regioni di attrazione','Interpreter','latex');
xlim(cfg.alphaRangeDeg);
ylim(cfg.betaRangeDeg);
if ~isempty(hLegend)
    legend( ...
        hLegend, ...
        'Location','bestoutside');
end

%% ========================================================================
% 10. RIEPILOGO
% ========================================================================
fprintf('\n============================================================\n');
fprintf('RIEPILOGO RAS FINAL V5\n');
fprintf('============================================================\n');
for ic = 1:numel(results)
    if results(ic).domainFullyStable
        extra = ' - bordo NON ancora raggiunto';
    elseif results(ic).domainFullyUnstable
        extra = ' - nessun punto stabile';
    else
        extra = ' - confine interno al dominio';
    end
    fprintf('%-16s : %5d / %5d nella RAS numerica (%.1f%%), %5d convergenti lenti%s\n', ...
        results(ic).name, ...
        nnz(results(ic).stable), ...
        Npoints, ...
        100*nnz(results(ic).stable)/Npoints, ...
        nnz(results(ic).slowConverging), ...
        extra);
end

%% ========================================================================
% 11. METRICHE QUANTITATIVE DELLA RAS
% ========================================================================
dAlpha = abs(alphaGridDeg(2)-alphaGridDeg(1));
dBeta  = abs(betaGridDeg(2)-betaGridDeg(1));
cellAreaDeg2 = dAlpha*dBeta;
Controller = strings(numel(results),1);
StablePoints = zeros(numel(results),1);
StablePercent = zeros(numel(results),1);
SlowConvergingPoints = zeros(numel(results),1);
AreaDeg2 = zeros(numel(results),1);
DeltaAlphaMinDeg = nan(numel(results),1);
DeltaAlphaMaxDeg = nan(numel(results),1);
DeltaBetaMinDeg = nan(numel(results),1);
DeltaBetaMaxDeg = nan(numel(results),1);
TouchesSearchBoundary = false(numel(results),1);
for ic = 1:numel(results)
    M = logical(results(ic).stableMap);
    Controller(ic) = string(results(ic).name);
    StablePoints(ic) = nnz(M);
    StablePercent(ic) = 100*nnz(M)/numel(M);
    SlowConvergingPoints(ic) = nnz(results(ic).slowConverging);
    % Area numerica di cella. Per una griglia sufficientemente fitta è una
    % stima semplice, trasparente e riproducibile.
    AreaDeg2(ic) = nnz(M)*cellAreaDeg2;
    if any(M(:))
        [ib,ia] = find(M);
        DeltaAlphaMinDeg(ic) = min(alphaGridDeg(ia));
        DeltaAlphaMaxDeg(ic) = max(alphaGridDeg(ia));
        DeltaBetaMinDeg(ic)  = min(betaGridDeg(ib));
        DeltaBetaMaxDeg(ic)  = max(betaGridDeg(ib));
        TouchesSearchBoundary(ic) = ...
            any(M(1,:)) || any(M(end,:)) || ...
            any(M(:,1)) || any(M(:,end));
    end
end
RASmetrics = table( ...
    Controller, ...
    StablePoints, ...
    StablePercent, ...
    SlowConvergingPoints, ...
    AreaDeg2, ...
    DeltaAlphaMinDeg, ...
    DeltaAlphaMaxDeg, ...
    DeltaBetaMinDeg, ...
    DeltaBetaMaxDeg, ...
    TouchesSearchBoundary);
disp(' ');
disp('============================================================');
disp('METRICHE RAS FINALI');
disp('============================================================');
disp(RASmetrics);
writetable(RASmetrics,'RAS_metrics_final_v5.csv');

%% ========================================================================
% 11. SALVATAGGIO
% ========================================================================
if cfg.saveMAT
    save( ...
        'RAS_vectorized_results_final_v5.mat', ...
        'results', ...
        'cfg', ...
        'alphaGridDeg', ...
        'betaGridDeg', ...
        'DAdeg', ...
        'DBdeg', ...
        'RASmetrics');
    fprintf('\nSalvato: RAS_vectorized_results_final_v5.mat\n');
end
disp('RAS FINAL V5 completata.');

%% ========================================================================
% FUNZIONI LOCALI
% ========================================================================
function controllers = loadControllers(requestedNames)
    controllers = struct( ...
        'name',{}, ...
        'family',{}, ...
        'A',{}, ...
        'B',{}, ...
        'C',{}, ...
        'D',{});
    if isfile('LQG_2DOF_Controllers.mat')
        SL = load('LQG_2DOF_Controllers.mat');
    else
        SL = struct;
    end
    if ismember('LQG',requestedNames)
        controllers(end+1) = makeController( ...
            'LQG','LQG',fetchSystem('K_LQG',SL)); %#ok<AGROW>
    end
    if ismember('LQGI',requestedNames)
        controllers(end+1) = makeController( ...
            'LQGI','LQG',fetchSystem('K_LQGI',SL)); %#ok<AGROW>
    end
    if isfile('HINF_controllers.mat')
        SH = load('HINF_controllers.mat');
    else
        SH = struct;
    end
    if ismember('mixsyn',requestedNames)
        controllers(end+1) = makeController( ...
            'mixsyn','HINF',fetchSystem('K_mix',SH)); %#ok<AGROW>
    end
    if ismember('hinfsyn',requestedNames)
        controllers(end+1) = makeController( ...
            'hinfsyn','HINF',fetchSystem('K_hinfsyn',SH)); %#ok<AGROW>
    end
    if ismember('PID+comp',requestedNames)
        controllers(end+1) = makeController( ...
            'PID+comp','HINF',fetchSystem('K_pidcomp',SH)); %#ok<AGROW>
    end
    if ismember('mu-synthesis',requestedNames)
        if isfile('MU_controller.mat')
            SM = load('MU_controller.mat');
        else
            SM = struct;
        end
        controllers(end+1) = makeController( ...
            'mu-synthesis','HINF',fetchSystem('K_mu',SM)); %#ok<AGROW>
    end
    if ismember('H2',requestedNames)
        if isfile('H2_controller.mat')
            S2 = load('H2_controller.mat');
        else
            S2 = struct;
        end
        controllers(end+1) = makeController( ...
            'H2','HINF',fetchSystem('K_h2',S2)); %#ok<AGROW>
    end
end
function K = fetchSystem(varName,S)
    if isfield(S,varName)
        K = S.(varName);
        return
    end
    if evalin('base',sprintf("exist('%s','var')",varName))
        K = evalin('base',varName);
        return
    end
    error([ ...
        'Controllore %s non trovato. ', ...
        'Eseguire prima gli script di sintesi.'],varName);
end
function Ctl = makeController(name,family,K)
    K = ss(K);
    [A,B,C,D] = ssdata(K);
    Ctl.name = name;
    Ctl.family = family;
    Ctl.A = double(A);
    Ctl.B = double(B);
    Ctl.C = double(C);
    Ctl.D = double(D);
end
function dX = helicopterDerivativeVectorized(X,F,p)
    alpha    = X(1,:);
    alphaDot = X(2,:);
    beta     = X(3,:);
    betaDot  = X(4,:);
    F1 = F(1,:);
    F2 = F(2,:);
    Jbeta = ...
        p.J_y.*sin(alpha).^2 + ...
        (p.J_z + p.m*p.l^2).*cos(alpha).^2 + ...
        p.I_b;
    tauAlpha = ...
        p.l.*( ...
            cos(beta).*F1 + ...
            p.epsilon_p.*sin(beta).*F2) ...
        - p.c_alpha.*alphaDot ...
        - p.m*p.g*p.l.*sin(alpha);
    tauBeta = ...
        p.l.*( ...
            p.epsilon_y.*sin(alpha).*F1 + ...
            cos(alpha).*F2) ...
        - p.c_beta.*betaDot;
    alphaDDot = tauAlpha ./ p.J_alpha;
    betaDDot  = tauBeta  ./ Jbeta;
    dX = [
        alphaDot;
        alphaDDot;
        betaDot;
        betaDDot
    ];
end
function [cmd1,cmd2] = controllerCommand(Xk,uk,Ctl,act)
    if isempty(Xk)
        raw = Ctl.D*uk;
    else
        raw = Ctl.C*Xk + Ctl.D*uk;
    end
    cmd1 = min(max( ...
        raw(1,:), ...
        act.deltaF1_min), ...
        act.deltaF1_max);
    cmd2 = min(max( ...
        raw(2,:), ...
        act.deltaF2_min), ...
        act.deltaF2_max);
end
function [Phi,Gamma] = exactZOH(A,B,h)
    % Exact state transition for xdot=A*x+B*u with u constant over h.
    n = size(A,1);
    m = size(B,2);
    if n == 0
        Phi = zeros(0);
        Gamma = zeros(0,m);
        return
    end
    E = expm([A, B; zeros(m,n+m)]*h);
    Phi = E(1:n,1:n);
    Gamma = E(1:n,n+(1:m));
end
function out = sanitizeName(in)
    out = regexprep(in,'[^a-zA-Z0-9_-]','_');
end
