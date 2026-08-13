%% HINF_SETUP
%
% Setup per il punto 2 - controllo H-infinity.
%
% Controllori:
%
%   1) mixsyn
%   2) hinfsyn
%   3) hinfstruct:
%
%        PID_alpha -> compensatore dinamico F_alpha(s)
%        PID_beta  -> compensatore dinamico F_beta(s)
%
% Sintesi eseguita sul plant normalizzato:
%
%       y_bar = Dy^-1 y
%       u     = Du u_bar
%
%       G_scaled = Dy^-1 G_nominal Du

close all;
clc;

%% ========================================================================
% 1. VERIFICA VARIABILI
% ========================================================================

requiredVariables = {
    'P_nominal'
    'P_uncertain'
    'alpha0'
    'beta0'
    'p0'
    'act'
    'sensor'
};

for k = 1:numel(requiredVariables)

    if ~exist(requiredVariables{k},'var')

        error( ...
            'Variabile mancante: %s', ...
            requiredVariables{k});

    end

end

%% ========================================================================
% 2. PLANT NOMINALE ELICOTTERO
% ========================================================================

if isa(P_nominal,'uss')

    Pnom = ss(P_nominal.NominalValue);

else

    Pnom = ss(P_nominal);

end

Pnom = minreal(Pnom,1e-9);

% Uscite:
% 1 = delta_alpha
% 3 = delta_beta

Pq_nominal = Pnom([1 3],:);

Pq_nominal.InputName = {
    'delta_F1'
    'delta_F2'
};

Pq_nominal.OutputName = {
    'delta_alpha'
    'delta_beta'
};

%% Plant incerto

% Pq_uncertain = P_uncertain([1 3],:);
% 
% Pq_uncertain.InputName = {
%     'delta_F1'
%     'delta_F2'
% };
% 
% Pq_uncertain.OutputName = {
%     'delta_alpha'
%     'delta_beta'
% };

%% Plant incerto

Pq_uncertain = P_uncertain([1 3],:);

% --- NUOVO BLOCCO PER RIDURRE LE INCERTEZZE A 2 ---
parametersToNominal = {
    'J_y'
    'J_z'
    'm'
    'l'
    'epsilon_y'
    };

for k = 1:numel(parametersToNominal)
    parName = parametersToNominal{k};
    if isfield(Pq_uncertain.Uncertainty,parName)
        block = Pq_uncertain.Uncertainty.(parName);
        Pq_uncertain = usubs(Pq_uncertain, parName, block.NominalValue);
    end
end
% --------------------------------------------------

Pq_uncertain.InputName = {
    'delta_F1'
    'delta_F2'
    };

Pq_uncertain.OutputName = {
    'delta_alpha'
    'delta_beta'
    };


%% ========================================================================
% 3. ATTUATORI NOMINALI
% ========================================================================

s = tf('s');

%% Motore 1

Gm1_nominal = ...
    act.wn1^2 / ...
    ( ...
        s^2 ...
        + 2*act.zeta1*act.wn1*s ...
        + act.wn1^2 ...
    );

Gpade1_nominal = ...
    (1 - act.td1*s/2) / ...
    (1 + act.td1*s/2);

Gact1_nominal = minreal( ...
    ss(Gm1_nominal*Gpade1_nominal), ...
    1e-8);

%% Motore 2

Gm2_nominal = ...
    act.wn2^2 / ...
    ( ...
        s^2 ...
        + 2*act.zeta2*act.wn2*s ...
        + act.wn2^2 ...
    );

Gpade2_nominal = ...
    (1 - act.td2*s/2) / ...
    (1 + act.td2*s/2);

Gact2_nominal = minreal( ...
    ss(Gm2_nominal*Gpade2_nominal), ...
    1e-8);

%% MIMO

Gact_nominal = blkdiag( ...
    Gact1_nominal, ...
    Gact2_nominal);

%% ========================================================================
% 4. PLANT NOMINALE COMPLETO
% ========================================================================

G_nominal = minreal( ...
    Pq_nominal*Gact_nominal, ...
    1e-7);

G_nominal.InputName = {
    'delta_F1_cmd'
    'delta_F2_cmd'
};

G_nominal.OutputName = {
    'delta_alpha'
    'delta_beta'
};

%% ========================================================================
% 5. ATTUATORI INCERTI
% ========================================================================

wn1_uncertain = ureal( ...
    'wn1', ...
    act.wn1, ...
    'Percentage',20);

wn2_uncertain = ureal( ...
    'wn2', ...
    act.wn2, ...
    'Percentage',20);

td1_uncertain = ureal( ...
    'td1', ...
    act.td1, ...
    'Percentage',30);

td2_uncertain = ureal( ...
    'td2', ...
    act.td2, ...
    'Percentage',30);

%% Motore 1

Gm1_uncertain = ...
    wn1_uncertain^2 / ...
    ( ...
        s^2 ...
        + 2*act.zeta1*wn1_uncertain*s ...
        + wn1_uncertain^2 ...
    );

Gpade1_uncertain = ...
    (1 - td1_uncertain*s/2) / ...
    (1 + td1_uncertain*s/2);

%% Motore 2

Gm2_uncertain = ...
    wn2_uncertain^2 / ...
    ( ...
        s^2 ...
        + 2*act.zeta2*wn2_uncertain*s ...
        + wn2_uncertain^2 ...
    );

Gpade2_uncertain = ...
    (1 - td2_uncertain*s/2) / ...
    (1 + td2_uncertain*s/2);

Gact_uncertain = blkdiag( ...
    Gm1_uncertain*Gpade1_uncertain, ...
    Gm2_uncertain*Gpade2_uncertain);

%% ========================================================================
% 6. PLANT INCERTO COMPLETO
% ========================================================================

G_uncertain = ...
    Pq_uncertain * ...
    Gact_uncertain;

%% ========================================================================
% 7. VN-100 LINEARIZZATO
% ========================================================================

g0 = p0.g;
Bmag0 = sensor.mag.B0;

Jsensor = [
    g0*cos(alpha0),                    0;
    0,                  -Bmag0*sin(beta0);
    0,                  -Bmag0*cos(beta0)
];

V_sensor = diag([
    sensor.acc.var
    sensor.mag.var
    sensor.mag.var
]);

Vinv = diag(1./diag(V_sensor));

Hy = ...
    (Jsensor.'*Vinv*Jsensor) \ ...
    (Jsensor.'*Vinv);

R_angle = ...
    Hy*V_sensor*Hy.';

R_angle = ...
    0.5*(R_angle+R_angle.');

Wnoise_angle = chol( ...
    R_angle + 1e-14*eye(2), ...
    'lower');

y0_sensor = [
    g0*sin(alpha0);
    Bmag0*cos(beta0);
   -Bmag0*sin(beta0)
];

%% ========================================================================
% 8. ANALISI PRELIMINARE
% ========================================================================

plantPoles = pole(G_nominal);
plantZeros = tzero(G_nominal);

fprintf('\n============================================================\n');
fprintf('PLANT NOMINALE\n');
fprintf('============================================================\n');

disp('Poli:');
disp(plantPoles);

disp('Zeri:');
disp(plantZeros);

Gdc = dcgain(G_nominal);

%% ========================================================================
% ANALISI RGA DEL PLANT A BASSA FREQUENZA
% ========================================================================

RGA0 = Gdc .* (inv(Gdc)).';

fprintf('\n============================================================\n');
fprintf('RGA A OMEGA = 0\n');
fprintf('============================================================\n');

disp(RGA0);

fprintf('cond(G(0)) = %.6e\n',cond(Gdc));

fprintf('G(0) =\n');
disp(Gdc);

fprintf('cond(G(0)) = %.6e\n', ...
    cond(Gdc));

[Udc,Sdc,Vdc] = svd(Gdc);

weakInputDirection = ...
    Vdc(:,end);

weakOutputDirection = ...
    Udc(:,end);

figure('Name','Plant nominale');
sigma(G_nominal);
grid on;
title('Valori singolari del plant nominale');

%% ========================================================================
% 9. NORMALIZZAZIONE
% ========================================================================

scale.alpha = deg2rad(2);
scale.beta  = deg2rad(3);

scale.F1 = 0.50;
scale.F2 = 0.50;

Dy = diag([
    scale.alpha
    scale.beta
]);

Du = diag([
    scale.F1
    scale.F2
]);

Dy_inv = inv(Dy);
Du_inv = inv(Du);

%% Plant nominale normalizzato

G_scaled = minreal( ...
    Dy_inv*G_nominal*Du, ...
    1e-7);

%% Plant incerto normalizzato

G_uncertain_scaled = ...
    Dy_inv*G_uncertain*Du;

fprintf('\n============================================================\n');
fprintf('PLANT NORMALIZZATO\n');
fprintf('============================================================\n');

disp('G_scaled(0) =');
disp(dcgain(G_scaled));

%% ========================================================================
% 10. PESO WS
%
% Nuovo tuning:
%
% maggiore banda
% minore Ms
%
% -> risposta più veloce
% -> maggiore smorzamento richiesto
% ========================================================================

weight.Ms_alpha = 1.35;
weight.Ms_beta  = 1.40;

weight.As_alpha = 0.01;
weight.As_beta  = 0.01;

weight.wb_alpha = 4.0;
weight.wb_beta  = 3.2;

WS_alpha = ...
    (s/weight.Ms_alpha + weight.wb_alpha) / ...
    (s + weight.wb_alpha*weight.As_alpha);

WS_beta = ...
    (s/weight.Ms_beta + weight.wb_beta) / ...
    (s + weight.wb_beta*weight.As_beta);

WS = blkdiag( ...
    WS_alpha, ...
    WS_beta);

%% ========================================================================
% 11. PESO WU
% ========================================================================

WU = ss( ...
    [], ...
    [], ...
    [], ...
    eye(2));

%% ========================================================================
% 12. PESO WT
%
% Spostato a frequenze maggiori per consentire
% maggiore banda al controller.
% ========================================================================

weight.Mt_alpha = 1.50;
weight.Mt_beta  = 1.50;

weight.At_alpha = 0.01;
weight.At_beta  = 0.01;

weight.wt_alpha = 22;
weight.wt_beta  = 18;

WT_alpha = ...
    ( ...
        s + ...
        weight.wt_alpha*weight.At_alpha ...
    ) / ...
    ( ...
        s/weight.Mt_alpha + ...
        weight.wt_alpha ...
    );

WT_beta = ...
    ( ...
        s + ...
        weight.wt_beta*weight.At_beta ...
    ) / ...
    ( ...
        s/weight.Mt_beta + ...
        weight.wt_beta ...
    );

WT = blkdiag( ...
    WT_alpha, ...
    WT_beta);

%% ========================================================================
% 13. GRAFICO PESI
% ========================================================================

omegaWeights = ...
    logspace(-2,3,500);

figure('Name','HINF requirements');

subplot(3,1,1);

sigma( ...
    inv(WS), ...
    omegaWeights);

grid on;
title('W_S^{-1}');

subplot(3,1,2);

sigma( ...
    inv(WU), ...
    omegaWeights);

grid on;
title('W_U^{-1}');

subplot(3,1,3);

sigma( ...
    inv(WT), ...
    omegaWeights);

grid on;
title('W_T^{-1}');

%% ========================================================================
% 14. PLANT GENERALIZZATO
% ========================================================================

P_mix = augw( ...
    G_scaled, ...
    WS, ...
    WU, ...
    WT);

%% ========================================================================
% 15. PARAMETRI ANALISI
% ========================================================================

omegaHinf = ...
    logspace(-2,3,500);

reference.alphaStep = ...
    scale.alpha;

reference.betaStep = ...
    scale.beta;

reference.alphaTime = 2;
reference.betaTime  = 5;

disturbance.force1 = 0.05;
disturbance.force2 = 0.05;

%% Requisiti temporali di progetto

timeRequirements.pitchSettling = 2.0;
timeRequirements.yawSettling   = 2.5;

timeRequirements.pitchOvershoot = 10;
timeRequirements.yawOvershoot   = 10;

%% ========================================================================
% 16. SALVATAGGIO
% ========================================================================

save('HINF_setup.mat', ...
    'G_nominal', ...
    'G_uncertain', ...
    'G_scaled', ...
    'G_uncertain_scaled', ...
    'Gact_nominal', ...
    'Gact_uncertain', ...
    'P_mix', ...
    'WS', ...
    'WU', ...
    'WT', ...
    'Dy', ...
    'Du', ...
    'Dy_inv', ...
    'Du_inv', ...
    'scale', ...
    'Hy', ...
    'Jsensor', ...
    'R_angle', ...
    'Wnoise_angle', ...
    'y0_sensor', ...
    'omegaHinf', ...
    'reference', ...
    'disturbance', ...
    'timeRequirements', ...
    'weight', ...
    'weakInputDirection', ...
    'weakOutputDirection', ...
    'RGA0');

fprintf('\nHINF_setup.mat creato correttamente.\n');