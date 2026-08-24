%% NOTE TEORICHE - SINTESI LQG/LQGI
% Il progetto LQG combina una retroazione ottima LQR con una stima di stato
% di Kalman. La variante LQGI aggiunge stati integrali sugli errori angolari
% per migliorare l'inseguimento a regime. La sintesi e' nominale; la
% robustezza rispetto alle incertezze parametriche viene poi verificata
% separatamente sui campioni del plant incerto.
%

%% LQG_2DOF_SYNTHESIS
%
% Progetto LQG dell'elicottero 2DOF:
%   1) LQG senza azione integrale
%   2) LQG con azione integrale su alpha e beta
%
% Il modello di progetto comprende:
%   - dinamica linearizzata dell'elicottero;
%   - due attuatori EMAX modellati come secondo ordine + Pade ordine 1;
%   - sensore VN-100 linearizzato attorno all'equilibrio;
%   - rumore di attuazione;
%   - rumore di misura accelerometro/magnetometro.
%
% Il progetto LQG viene effettuato sul plant nominale.
% Le incertezze vengono utilizzate per la validazione successiva.
close all;
clc;

%% ========================================================================
%  1. VERIFICA VARIABILI
% ========================================================================
requiredVariables = {
    'P_nominal'
    'P_uncertain'
    'alpha0'
    'beta0'
    'x0'
    'u0'
    'act'
    'sensor'
};
for k = 1:numel(requiredVariables)
    assert(evalin('base', ...
        sprintf("exist('%s','var')",requiredVariables{k})) == 1, ...
        'Variabile mancante nel workspace: %s', requiredVariables{k});
end

%% ========================================================================
%  2. MODELLO NOMINALE DEGLI ATTUATORI
% ========================================================================
s = tf('s');
Gm1 = act.wn1^2 / ...
    (s^2 + 2*act.zeta1*act.wn1*s + act.wn1^2);
Gm2 = act.wn2^2 / ...
    (s^2 + 2*act.zeta2*act.wn2*s + act.wn2^2);
% Pade del primo ordine
Gd1 = (1 - act.td1*s/2) / ...
      (1 + act.td1*s/2);
Gd2 = (1 - act.td2*s/2) / ...
      (1 + act.td2*s/2);
Gact1_nominal = minreal(Gm1*Gd1);
Gact2_nominal = minreal(Gm2*Gd2);
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

%% ========================================================================
%  3. REALIZZAZIONE AUMENTATA ATTUATORI + ELICOTTERO
%
%  Stati:
%
%    xa = stati degli attuatori
%    xh = [delta_alpha;
%          delta_alpha_dot;
%          delta_beta;
%          delta_beta_dot]
%
%  xaug = [xa; xh]
% ========================================================================
[Aact,Bact,Cact,Dact] = ssdata(Gact_nominal);
Aact = double(Aact);
Bact = double(Bact);
Cact = double(Cact);
Dact = double(Dact);

%% Conversione esplicita del plant nominale in modello numerico
if isa(P_nominal,'uss')
    Pnom = ss(P_nominal.NominalValue);
else
    Pnom = ss(P_nominal);
end
Ah = double(Pnom.A);
Bh = double(Pnom.B);
Ch = double(Pnom.C);
Dh = double(Pnom.D);
na = size(Aact,1);
nh = size(Ah,1);
nu = size(Bact,2);
A = [
    Aact,                zeros(na,nh);
    Bh*Cact,             Ah
];
B = [
    Bact;
    Bh*Dact
];
n = size(A,1);

%% Matrice che restituisce gli stati fisici dell'elicottero
Cx_helicopter = [
    zeros(nh,na), eye(nh)
];

%% Matrice che restituisce le forze incrementali degli attuatori
CdeltaF = [
    Cact, zeros(2,nh)
];
DdeltaF = Dact;
Plant_augmented = ss( ...
    A, B, Cx_helicopter, zeros(nh,nu));
Plant_augmented.StateName = [
    compose("x_act_%d",(1:na)')
    {
    'delta_alpha'
    'delta_alpha_dot'
    'delta_beta'
    'delta_beta_dot'
    }
];
Plant_augmented.InputName = {
    'delta_F1_cmd'
    'delta_F2_cmd'
};
Plant_augmented.OutputName = {
    'delta_alpha'
    'delta_alpha_dot'
    'delta_beta'
    'delta_beta_dot'
};

%% ========================================================================
%  4. LINEARIZZAZIONE DEL MODELLO DEL VN-100
%
%  Accelerometro:
%     yacc = g*sin(alpha)
%
%  Magnetometro planare:
%     mx = B0*cos(beta)
%     my = -B0*sin(beta)
%
%  Il filtro usa le variazioni:
%     delta_y = y - y0
% ========================================================================
g0 = p0.g;
B0 = sensor.mag.B0;
% Jacobiano rispetto agli stati fisici:
% xh = [delta_alpha, delta_alpha_dot,
%       delta_beta,  delta_beta_dot]
Csensor_h = [
    g0*cos(alpha0), 0, 0,                  0;
    0,              0, -B0*sin(beta0),     0;
    0,              0, -B0*cos(beta0),     0
];
% Il sensore non misura direttamente gli stati degli attuatori
Cmeas = [
    zeros(3,na), Csensor_h
];
Dmeas = zeros(3,nu);
ny = size(Cmeas,1);

%% Valori nominali delle misure
y0_sensor = [
    g0*sin(alpha0);
    B0*cos(beta0);
   -B0*sin(beta0)
];

%% Uscite da controllare: alpha e beta
Ctrack_h = [
    1, 0, 0, 0;
    0, 0, 1, 0
];
Ctrack = [
    zeros(2,na), Ctrack_h
];
nr = size(Ctrack,1);

%% ========================================================================
%  5. VERIFICHE STRUTTURALI
% ========================================================================
controllabilityRank = rank(ctrb(A,B));
observabilityRank   = rank(obsv(A,Cmeas));
fprintf('\n============================================================\n');
fprintf('MODELLO AUMENTATO\n');
fprintf('============================================================\n');
fprintf('Numero stati attuatori:      %d\n',na);
fprintf('Numero stati elicottero:     %d\n',nh);
fprintf('Numero stati complessivi:    %d\n',n);
fprintf('Rango controllabilita'':      %d / %d\n', ...
    controllabilityRank,n);
fprintf('Rango osservabilita'':        %d / %d\n', ...
    observabilityRank,n);
if controllabilityRank < n
    warning('Il modello aumentato non è completamente controllabile.');
end
if observabilityRank < n
    warning(['Il modello aumentato non è completamente osservabile. ', ...
             'Verificare almeno la rilevabilita''.']);
end
fprintf('Massimo Re(lambda) modi non osservabili: controllo manuale richiesto.\n');

%% ========================================================================
%  6. PESI LQR
%
%  Si usa una penalizzazione sulle grandezze fisiche:
%
%  z = [delta_F1;
%       delta_F2;
%       delta_alpha;
%       delta_alpha_dot;
%       delta_beta;
%       delta_beta_dot]
%
%  secondo la regola di Bryson.
% ========================================================================
Cphysical = [
    CdeltaF;
    Cx_helicopter
];
% Massimi scostamenti considerati ragionevoli
deltaF1_max = 0.40;          % [N]
deltaF2_max = 0.40;          % [N]
alpha_max   = deg2rad(10);   % [rad]
alphaDot_max = 1.0;          % [rad/s]
beta_max    = deg2rad(15);   % [rad]
betaDot_max = 1.0;           % [rad/s]
zmax = [
    deltaF1_max;
    deltaF2_max;
    alpha_max;
    alphaDot_max;
    beta_max;
    betaDot_max
];
Wphysical = diag(1./zmax.^2);
% Piccolo termine regolarizzante sugli stati interni
Q = Cphysical'*Wphysical*Cphysical ...
    + 1e-7*eye(n);
% Penalizzazione sui comandi incrementali
deltaFcmd_max = [
    0.50;
    0.50
];
R = diag(1./deltaFcmd_max.^2);

%% ========================================================================
%  7. COVARIANZE PER IL FILTRO DI KALMAN
% ========================================================================
% Il rumore di attuazione entra prima della dinamica degli attuatori.
Gnoise = B;
W = diag([
    act.sigma_F1^2
    act.sigma_F2^2
]);
% Rumore delle misure VN-100:
% y = [accelerometro; magnetometro x; magnetometro y]
V = diag([
    sensor.acc.var
    sensor.mag.var
    sensor.mag.var
]);

%% Ricostruzione WLS degli angoli dalle misure VN-100
Jsensor = [
    g0*cos(alpha0),                  0;
    0,                -B0*sin(beta0);
    0,                -B0*cos(beta0)
    ];
Vinv = ...
    diag(1./diag(V));
Hy_LQG = ...
    (Jsensor.'*Vinv*Jsensor) \ ...
    (Jsensor.'*Vinv);
% Piccola regolarizzazione numerica
V = V + 1e-12*eye(ny);

%% Verifica che tutte le matrici del Kalman siano numeriche
A      = double(A);
B      = double(B);
Gnoise = double(Gnoise);
Cmeas  = double(Cmeas);
W      = double(W);
V      = double(V);
disp(class(A));
disp(class(Gnoise));
disp(class(Cmeas));
disp(class(W));
disp(class(V));

%% Guadagno del filtro di Kalman continuo
[Ke,KalmanCovariance,KalmanPoles] = ...
    lqe(A,Gnoise,Cmeas,W,V);
fprintf('\nPoli del filtro di Kalman:\n');
disp(KalmanPoles);
Aobs = A - Ke*Cmeas;
Bobs = [B Ke];
Cobs = eye(size(A));
Dobs = zeros(size(A,1), size(B,2) + size(Cmeas,1));
xhat0 = zeros(size(A,1),1);

%% ========================================================================
%  8. LQG SENZA INTEGRATORE
% ========================================================================
Kx_LQG = lqr(A,B,Q,R);
Acl_stateFeedback = A - B*Kx_LQG;

%% Prefiltro statico per riferimenti costanti alpha e beta
%
% u = -Kx*xhat + Nbar*r
DCreference = ...
    -Ctrack * (Acl_stateFeedback \ B);
if rcond(DCreference) < 1e-10
    warning(['La matrice per il prefiltro è quasi singolare. ', ...
             'Il tracking senza integratore può essere scarso.']);
    Nbar = pinv(DCreference);
else
    Nbar = inv(DCreference);
end

%% Realizzazione del controllore
%
% Ingressi:
%   [delta_r_alpha;
%    delta_r_beta;
%    delta_y_acc;
%    delta_y_mx;
%    delta_y_my]
%
% Uscita:
%   delta_F_cmd
Ac_LQG = A - B*Kx_LQG - Ke*Cmeas;
Bc_LQG = [
    B*Nbar, Ke
];
Cc_LQG = -Kx_LQG;
Dc_LQG = [
    Nbar, zeros(nu,ny)
];
K_LQG = ss( ...
    Ac_LQG, ...
    Bc_LQG, ...
    Cc_LQG, ...
    Dc_LQG);
K_LQG.InputName = {
    'delta_r_alpha'
    'delta_r_beta'
    'delta_y_acc'
    'delta_y_mx'
    'delta_y_my'
};
K_LQG.OutputName = {
    'delta_F1_cmd'
    'delta_F2_cmd'
};
K_LQG.StateName = compose("xhat_%d",(1:n)');

%% ========================================================================
%  9. LQG CON AZIONE INTEGRALE
%
%  xi_dot = delta_r - Ctrack*xhat
%
%  Stati aumentati per l'LQR:
%     xa_LQI = [xaug; xi]
% ========================================================================
A_LQI = [
    A,                  zeros(n,nr);
   -Ctrack,             zeros(nr,nr)
];
B_LQI = [
    B;
    zeros(nr,nu)
];
% Peso sugli integratori
Qi = diag([
    150
    150
]);
Q_LQI = blkdiag(Q,Qi);
Kaug_LQI = lqr(A_LQI,B_LQI,Q_LQI,R);
Kx_LQGI = Kaug_LQI(:,1:n);
Ki_LQGI = Kaug_LQI(:,n+1:end);

%% Il filtro di Kalman stima solo il plant fisico aumentato,
%  non gli stati integrali.
Ac_LQGI = [
    A - B*Kx_LQGI - Ke*Cmeas,   -B*Ki_LQGI;
    zeros(nr,n),                 zeros(nr,nr)
    ];
Bc_LQGI = [
    zeros(n,nr),      Ke;
    eye(nr),         -Hy_LQG
    ];
Cc_LQGI = [
    -Kx_LQGI, -Ki_LQGI
];
Dc_LQGI = zeros(nu,nr+ny);
K_LQGI = ss( ...
    Ac_LQGI, ...
    Bc_LQGI, ...
    Cc_LQGI, ...
    Dc_LQGI);
K_LQGI.InputName = K_LQG.InputName;
K_LQGI.OutputName = {
    'delta_F1_cmd'
    'delta_F2_cmd'
};
K_LQGI.StateName = [
    compose("xhat_%d",(1:n)')
    {
    'xi_alpha'
    'xi_beta'
    }
];

%% ========================================================================
%  10. ANALISI NOMINALE DEI SISTEMI CHIUSI
% ========================================================================
% Costruzione esplicita dei sistemi chiusi con stato:
%
% [x;
%  xhat]
%
% oppure:
%
% [x;
%  xhat;
%  xi]

%% LQG senza integratore, riferimento nullo e rumori nulli
Acl_LQG = [
    A,                         -B*Kx_LQG;
    Ke*Cmeas,      A-B*Kx_LQG-Ke*Cmeas
];

%% LQGI con integratore
Acl_LQGI = [
    A,                         -B*Kx_LQGI,               -B*Ki_LQGI;
    Ke*Cmeas,      A-B*Kx_LQGI-Ke*Cmeas,               -B*Ki_LQGI;
    -Hy_LQG*Cmeas,              zeros(nr,n),              zeros(nr,nr)
    ];
eig_LQG  = eig(Acl_LQG);
eig_LQGI = eig(Acl_LQGI);
fprintf('\n============================================================\n');
fprintf('AUTOVALORI LQG SENZA INTEGRATORE\n');
fprintf('============================================================\n');
disp(eig_LQG);
fprintf('Massima parte reale: %.6g\n',max(real(eig_LQG)));
fprintf('\n============================================================\n');
fprintf('AUTOVALORI LQG CON INTEGRATORE\n');
fprintf('============================================================\n');
disp(eig_LQGI);
fprintf('Massima parte reale: %.6g\n',max(real(eig_LQGI)));
assert(all(real(eig_LQG)<0), ...
    'Il sistema LQG nominale non è asintoticamente stabile.');
assert(all(real(eig_LQGI)<0), ...
    'Il sistema LQGI nominale non è asintoticamente stabile.');

%% ========================================================================
%  11. REGIONE DI ASINTOTICA STABILITA' DEL MODELLO LINEARE
% ========================================================================
fprintf('\n============================================================\n');
fprintf('RAS DEL MODELLO LINEARE\n');
fprintf('============================================================\n');
fprintf(['Il modello lineare nominale in anello chiuso è Hurwitz.\n', ...
         'In assenza di saturazioni e altre nonlinearità:\n\n']);
fprintf('RAS lineare LQG  = R^%d\n',size(Acl_LQG,1));
fprintf('RAS lineare LQGI = R^%d\n',size(Acl_LQGI,1));

%% Matrici di Lyapunov utilizzabili per le stime non lineari
Plyap_LQG = lyap(Acl_LQG',eye(size(Acl_LQG)));
Plyap_LQGI = lyap(Acl_LQGI',eye(size(Acl_LQGI)));

%% ========================================================================
%  12. MODELLO INCERTO DEGLI ATTUATORI
% ========================================================================
wn1_unc = ureal( ...
    'wn1',act.wn1, ...
    'Percentage',20);
wn2_unc = ureal( ...
    'wn2',act.wn2, ...
    'Percentage',20);
td1_unc = ureal( ...
    'td1',act.td1, ...
    'Percentage',30);
td2_unc = ureal( ...
    'td2',act.td2, ...
    'Percentage',30);
zeta1 = act.zeta1;
zeta2 = act.zeta2;
Gm1_unc = wn1_unc^2 / ...
    (s^2 + 2*zeta1*wn1_unc*s + wn1_unc^2);
Gm2_unc = wn2_unc^2 / ...
    (s^2 + 2*zeta2*wn2_unc*s + wn2_unc^2);
Gd1_unc = ...
    (1 - td1_unc*s/2) / ...
    (1 + td1_unc*s/2);
Gd2_unc = ...
    (1 - td2_unc*s/2) / ...
    (1 + td2_unc*s/2);
Gact_uncertain = blkdiag( ...
    Gm1_unc*Gd1_unc, ...
    Gm2_unc*Gd2_unc);

%% Plant incerto dal comando alle misure fisiche dello stato
Pcomplete_uncertain = ...
    P_uncertain * Gact_uncertain;

%% Plant incerto dal comando alle misure linearizzate del VN-100
SensorLinearSS = ss([],[],[],Csensor_h);
Py_uncertain = ...
    SensorLinearSS * Pcomplete_uncertain;

%% ========================================================================
%  13. CONTROLLO DI STABILITA' SU CAMPIONI INCERTI
% ========================================================================
numberOfSamples = 30;
maxRealPole_LQG  = zeros(numberOfSamples,1);
maxRealPole_LQGI = zeros(numberOfSamples,1);
% Parte del controllore LQG che va dalle misure al comando,
% ponendo il riferimento a zero.
K_LQG_y = K_LQG(:,nr+1:end);
K_LQGI_y = K_LQGI(:,nr+1:end);
for k = 1:numberOfSamples
    Py_sample = usample(Py_uncertain);
    % Il segno meno è già contenuto nelle matrici di uscita
    % del controllore; si usa quindi feedback positivo.
    CL_sample_LQG = feedback( ...
        Py_sample, K_LQG_y, +1);
    CL_sample_LQGI = feedback( ...
        Py_sample, K_LQGI_y, +1);
    maxRealPole_LQG(k) = ...
        max(real(pole(CL_sample_LQG)));
    maxRealPole_LQGI(k) = ...
        max(real(pole(CL_sample_LQGI)));
end
fprintf('\n============================================================\n');
fprintf('VALIDAZIONE LINEARE INCERTA — %d CAMPIONI\n',numberOfSamples);
fprintf('============================================================\n');
fprintf('LQG:  massimo Re(polo) = %.6g\n', ...
    max(maxRealPole_LQG));
fprintf('LQGI: massimo Re(polo) = %.6g\n', ...
    max(maxRealPole_LQGI));
fprintf('Campioni LQG instabili:  %d / %d\n', ...
    sum(maxRealPole_LQG>=0),numberOfSamples);
fprintf('Campioni LQGI instabili: %d / %d\n', ...
    sum(maxRealPole_LQGI>=0),numberOfSamples);
figure('Name','LQG-LQGI - Uncertain-sample pole stability');
plot(maxRealPole_LQG,'o-');
hold on;
plot(maxRealPole_LQGI,'x-');
yline(0,'--');
xlabel('Indice del campione incerto');
ylabel('$\max\,\Re\{\lambda(A_{cl})\}$','Interpreter','latex');
legend( ...
    'LQG', ...
    'LQGI', ...
    'Limite di stabilità', ...
    'Location','best');
grid on;
title('Stabilita'' dei campioni parametrici incerti: LQG vs LQGI','Interpreter','latex');

%% ========================================================================
%  14. SALVATAGGIO DEI CONTROLLORI
% ========================================================================
save('LQG_2DOF_Controllers.mat', ...
    'K_LQG', ...
    'K_LQGI', ...
    'Kx_LQG', ...
    'Nbar', ...
    'Kx_LQGI', ...
    'Ki_LQGI', ...
    'Ke', ...
    'Hy_LQG', ...
    'A', ...
    'B', ...
    'Cmeas', ...
    'Ctrack', ...
    'y0_sensor', ...
    'Plyap_LQG', ...
    'Plyap_LQGI', ...
    'Plant_augmented', ...
    'Gact_nominal', ...
    'Gact_uncertain', ...
    'Pcomplete_uncertain');
fprintf('\nControllori salvati in LQG_2DOF_Controllers.mat\n');
