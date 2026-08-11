%% HINF_00_SETUP
%
% Costruzione del modello nominale e incerto per la sintesi H-infinity.
%
% Plant di controllo:
%
%   delta_F_cmd -> attuatori -> elicottero -> [delta_alpha; delta_beta]
%
% Gli attuatori comprendono:
%   - secondo ordine;
%   - ritardo approssimato mediante Pade di ordine 1.
%
% Il modello incerto contiene:
%   - incertezze parametriche dell'elicottero;
%   - incertezze sulle frequenze naturali degli attuatori;
%   - incertezze sui ritardi degli attuatori.

close all;
clc;

%% ========================================================================
%  1. VERIFICA DELLE VARIABILI
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
        error('Variabile mancante nel workspace: %s', ...
            requiredVariables{k});
    end
end

%% ========================================================================
%  2. MODELLO NOMINALE NUMERICO DELL'ELICOTTERO
% ========================================================================

if isa(P_nominal,'uss')
    Pnom = ss(P_nominal.NominalValue);
else
    Pnom = ss(P_nominal);
end

Pnom = minreal(Pnom,1e-9);

% P_nominal ha come uscite:
%
% 1 = delta_alpha
% 2 = delta_alpha_dot
% 3 = delta_beta
% 4 = delta_beta_dot
%
% Per la sintesi output-feedback H-infinity controlliamo gli angoli.

Pq_nominal = Pnom([1 3],:);

Pq_nominal.InputName = {
    'delta_F1'
    'delta_F2'
};

Pq_nominal.OutputName = {
    'delta_alpha'
    'delta_beta'
};

%% Plant incerto con le sole uscite angolari

Pq_uncertain = P_uncertain([1 3],:);

Pq_uncertain.InputName = {
    'delta_F1'
    'delta_F2'
};

Pq_uncertain.OutputName = {
    'delta_alpha'
    'delta_beta'
};

%% ========================================================================
%  3. ATTUATORI NOMINALI CON PADE DI ORDINE 1
% ========================================================================

s = tf('s');

%% Attuatore 1

Gm1_nominal = ...
    act.wn1^2 / ...
    (s^2 + 2*act.zeta1*act.wn1*s + act.wn1^2);

Gpade1_nominal = ...
    (1 - act.td1*s/2) / ...
    (1 + act.td1*s/2);

Gact1_nominal = minreal( ...
    ss(Gm1_nominal*Gpade1_nominal), ...
    1e-8);

%% Attuatore 2

Gm2_nominal = ...
    act.wn2^2 / ...
    (s^2 + 2*act.zeta2*act.wn2*s + act.wn2^2);

Gpade2_nominal = ...
    (1 - act.td2*s/2) / ...
    (1 + act.td2*s/2);

Gact2_nominal = minreal( ...
    ss(Gm2_nominal*Gpade2_nominal), ...
    1e-8);

%% Modello MIMO degli attuatori

Gact_nominal = blkdiag( ...
    Gact1_nominal, ...
    Gact2_nominal);

Gact_nominal.InputName = {
    'delta_F1_cmd'
    'delta_F2_cmd'
};

Gact_nominal.OutputName = {
    'delta_F1'
    'delta_F2'
};

%% Plant nominale completo

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
%  4. ATTUATORI INCERTI
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

%% Dinamica incerta attuatore 1

Gm1_uncertain = ...
    wn1_uncertain^2 / ...
    (s^2 ...
    + 2*act.zeta1*wn1_uncertain*s ...
    + wn1_uncertain^2);

Gpade1_uncertain = ...
    (1 - td1_uncertain*s/2) / ...
    (1 + td1_uncertain*s/2);

%% Dinamica incerta attuatore 2

Gm2_uncertain = ...
    wn2_uncertain^2 / ...
    (s^2 ...
    + 2*act.zeta2*wn2_uncertain*s ...
    + wn2_uncertain^2);

Gpade2_uncertain = ...
    (1 - td2_uncertain*s/2) / ...
    (1 + td2_uncertain*s/2);

%% Modello incerto MIMO

Gact_uncertain = blkdiag( ...
    Gm1_uncertain*Gpade1_uncertain, ...
    Gm2_uncertain*Gpade2_uncertain);

Gact_uncertain.InputName = {
    'delta_F1_cmd'
    'delta_F2_cmd'
};

Gact_uncertain.OutputName = {
    'delta_F1'
    'delta_F2'
};

%% Plant incerto completo

G_uncertain = ...
    Pq_uncertain*Gact_uncertain;

G_uncertain.InputName = {
    'delta_F1_cmd'
    'delta_F2_cmd'
};

G_uncertain.OutputName = {
    'delta_alpha'
    'delta_beta'
};

%% ========================================================================
%  5. RICOSTRUZIONE DEGLI ANGOLI DAL VN-100
%
%  Sensore linearizzato:
%
%   delta_y_sensor = Jsensor * delta_q + v
%
%  con:
%
%   delta_q = [delta_alpha; delta_beta]
%
%  La ricostruzione WLS è:
%
%   delta_q_meas = Hy * delta_y_sensor
% ========================================================================

g0 = p0.g;
B0 = sensor.mag.B0;

Jsensor = [
    g0*cos(alpha0),                  0;
    0,                -B0*sin(beta0);
    0,                -B0*cos(beta0)
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

R_angle = Hy*V_sensor*Hy.';
R_angle = 0.5*(R_angle + R_angle.');

Wnoise_angle = chol( ...
    R_angle + 1e-14*eye(2), ...
    'lower');

fprintf('\n============================================================\n');
fprintf('RICOSTRUZIONE ANGOLARE DAL VN-100\n');
fprintf('============================================================\n');

disp('Hy = ');
disp(Hy);

fprintf('Sigma alpha equivalente: %.6f deg\n', ...
    rad2deg(sqrt(R_angle(1,1))));

fprintf('Sigma beta equivalente:  %.6f deg\n', ...
    rad2deg(sqrt(R_angle(2,2))));

%% Valore nominale del sensore all'equilibrio

y0_sensor = [
    g0*sin(alpha0);
    B0*cos(beta0);
   -B0*sin(beta0)
];

%% ========================================================================
%  6. ANALISI PRELIMINARE DEL PLANT
% ========================================================================

plantPoles = pole(G_nominal);
plantZeros = tzero(G_nominal);

fprintf('\n============================================================\n');
fprintf('ANALISI DEL PLANT NOMINALE\n');
fprintf('============================================================\n');

disp('Poli del plant completo:');
disp(plantPoles);

disp('Zeri di trasmissione:');
disp(plantZeros);

RHPzeros = plantZeros(real(plantZeros)>1e-8);

if isempty(RHPzeros)
    fprintf('Nessuno zero di trasmissione RHP rilevato.\n');
else
    fprintf('Zeri di trasmissione RHP:\n');
    disp(RHPzeros);
end

figure('Name','Plant nominale - singular values');

sigma(G_nominal);

grid on;
title('Valori singolari del plant nominale completo');

%% Direzione più debole in bassa frequenza

Gdc = dcgain(G_nominal);

[Udc,Sdc,Vdc] = svd(Gdc);

weakInputDirection = Vdc(:,end);
weakOutputDirection = Udc(:,end);

fprintf('Valori singolari del guadagno statico:\n');
disp(diag(Sdc));

fprintf('Direzione di ingresso meno efficace:\n');
disp(weakInputDirection);

%% ========================================================================
%  7. PESI DEL PROBLEMA MIXED-SENSITIVITY
%
%  Obiettivo:
%
%        || WS*S  ||
%        || WU*KS ||  < gamma
%        || WT*T  ||
%
%  con:
%
%    S  = (I + G*K)^(-1)
%    KS = K*S
%    T  = G*K*S
% ========================================================================

%% Peso WS: tracking e reiezione dei disturbi

weight.Ms_alpha = 1.7;
weight.Ms_beta  = 1.8;

weight.As_alpha = 0.01;
weight.As_beta  = 0.01;

weight.wb_alpha = 2.5;      % rad/s
weight.wb_beta  = 2.0;      % rad/s

WS_alpha = ...
    (s/weight.Ms_alpha + weight.wb_alpha) / ...
    (s + weight.wb_alpha*weight.As_alpha);

WS_beta = ...
    (s/weight.Ms_beta + weight.wb_beta) / ...
    (s + weight.wb_beta*weight.As_beta);

WS = blkdiag(WS_alpha,WS_beta);

%% Peso WU: limitazione dei comandi

weight.deltaF1_max = 0.50;  % N
weight.deltaF2_max = 0.50;  % N

WU = ss([],[],[],diag([
    1/weight.deltaF1_max
    1/weight.deltaF2_max
]));

%% Peso WT: roll-off e attenuazione del rumore

weight.Mt_alpha = 2.0;
weight.Mt_beta  = 2.0;

weight.At_alpha = 0.01;
weight.At_beta  = 0.01;

weight.wt_alpha = 15;       % rad/s
weight.wt_beta  = 12;       % rad/s

WT_alpha = ...
    (s + weight.wt_alpha*weight.At_alpha) / ...
    (s/weight.Mt_alpha + weight.wt_alpha);

WT_beta = ...
    (s + weight.wt_beta*weight.At_beta) / ...
    (s/weight.Mt_beta + weight.wt_beta);

WT = blkdiag(WT_alpha,WT_beta);

%% ========================================================================
%  8. PLANT AUMENTATO STANDARD
% ========================================================================

P_mix = augw( ...
    G_nominal, ...
    WS, ...
    WU, ...
    WT);

%% ========================================================================
%  9. FREQUENZE E RIFERIMENTI DI TEST
% ========================================================================

omegaHinf = logspace(-2,3,400);

reference.alphaStep = deg2rad(2);
reference.betaStep  = deg2rad(3);

reference.alphaTime = 2;
reference.betaTime  = 5;

disturbance.force1 = 0.05;
disturbance.force2 = 0.05;

%% ========================================================================
%  10. SALVATAGGIO
% ========================================================================

save('HINF_setup.mat', ...
    'G_nominal', ...
    'G_uncertain', ...
    'Gact_nominal', ...
    'Gact_uncertain', ...
    'P_mix', ...
    'WS', ...
    'WU', ...
    'WT', ...
    'Hy', ...
    'Jsensor', ...
    'R_angle', ...
    'Wnoise_angle', ...
    'y0_sensor', ...
    'omegaHinf', ...
    'reference', ...
    'disturbance', ...
    'weight', ...
    'weakInputDirection', ...
    'weakOutputDirection');

fprintf('\nHINF_setup.mat creato correttamente.\n');