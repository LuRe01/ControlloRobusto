%% NOTE TEORICHE - MODELLAZIONE INCERTA E LINEARIZZAZIONE
% Questo script costruisce il modello linearizzato dell'elicottero 2DOF
% attorno a un equilibrio non nullo e rappresenta le principali variazioni
% parametriche mediante oggetti ureal. In controllo robusto, questa
% rappresentazione separa il modello nominale dalla famiglia di plant
% ammissibili e costituisce la base per analisi di stabilita' e prestazione
% robuste. L'ordine delle istruzioni eseguibili e i parametri numerici
% originali sono mantenuti invariati.
%

%% BUILD_UNCERTAIN_LINEAR_MODEL
%
% Costruzione del modello linearizzato incerto dell'elicottero 2DOF
% attorno a un punto di equilibrio non nullo.
%
% Richiede:
%   - Control System Toolbox
%   - Robust Control Toolbox
%
% Stato:
%
%   x = [alpha;
%        alpha_dot;
%        beta;
%        beta_dot]
%
% Ingresso:
%
%   u = [F1;
%        F2]
%
% Il modello linearizzato è espresso nelle variabili incrementali:
%
%   delta_x = x - x0
%   delta_u = u - u0
%
%   delta_x_dot = A * delta_x + B * delta_u
clearvars;
clc;

%% ================================================================
%  1. PARAMETRI NOMINALI E INCERTI
%  ================================================================
% Inerzia dell'asse pitch: ±10 %
J_alpha = ureal( ...
    'J_alpha', ...
    0.012, ...
    'Percentage', 10);
% Componenti dell'inerzia yaw
J_y = ureal( ...
    'J_y', ...
    0.00023, ...
    'Percentage', 15);
J_z = ureal( ...
    'J_z', ...
    0.00364, ...
    'Percentage', 10);
% Inerzia del corpo considerata perfettamente nota
I_b = 0.00023;
% Massa: ±10 %
m = ureal( ...
    'm', ...
    0.2, ...
    'Percentage', 10);
% Lunghezza del braccio: ±5 %
l = ureal( ...
    'l', ...
    0.2, ...
    'Percentage', 5);
% Attriti viscosi considerati inizialmente noti
c_alpha = 0.01;
c_beta  = 0.01;
% Coefficienti di cross-coupling aerodinamico: ±30 %
epsilon_p = ureal( ...
    'epsilon_p', ...
    0.1, ...
    'Percentage', 30);
epsilon_y = ureal( ...
    'epsilon_y', ...
    0.1, ...
    'Percentage', 30);
% Accelerazione gravitazionale
g = 9.81;

%% Struttura contenente i parametri
params.J_alpha  = J_alpha;
params.J_y      = J_y;
params.J_z      = J_z;
params.I_b      = I_b;
params.m        = m;
params.l        = l;
params.c_alpha  = c_alpha;
params.c_beta   = c_beta;
params.epsilon_p = epsilon_p;
params.epsilon_y = epsilon_y;
params.g = g;

%% ================================================================
%  2. SCELTA DEL PUNTO DI EQUILIBRIO
%  ================================================================
% Si sceglie un equilibrio non nullo per mantenere visibili nella
% linearizzazione gli accoppiamenti geometrici e aerodinamici.
alpha0 = deg2rad(15);
beta0  = deg2rad(20);
alpha_dot0 = 0;
beta_dot0  = 0;
x0 = [
    alpha0;
    alpha_dot0;
    beta0;
    beta_dot0
];

%% ================================================================
%  3. CALCOLO DELLE FORZE DI EQUILIBRIO
%  ================================================================
% All'equilibrio:
%
% F10*cos(beta0) + epsilon_p*F20*sin(beta0)
%       = m*g*sin(alpha0)
%
% epsilon_y*F10*sin(alpha0) + F20*cos(alpha0)
%       = 0
%
% Il fattore l è stato semplificato da entrambi i membri.
equilibrium_matrix = [
    cos(beta0),                   epsilon_p * sin(beta0);
    epsilon_y * sin(alpha0),      cos(alpha0)
];
equilibrium_vector = [
    m * g * sin(alpha0);
    0
];
% Forze di equilibrio dipendenti dai parametri incerti
F0_uncertain = equilibrium_matrix \ equilibrium_vector;
F10 = F0_uncertain(1);
F20 = F0_uncertain(2);
% Valori nominali delle forze di equilibrio
F0_nominal = F0_uncertain.NominalValue;
F10_nominal = F0_nominal(1);
F20_nominal = F0_nominal(2);
u0_uncertain = [
    F10;
    F20
];
u0_nominal = [
    F10_nominal;
    F20_nominal
];

%% Visualizzazione del punto di equilibrio nominale
fprintf('\n=================================================\n');
fprintf('PUNTO DI EQUILIBRIO NOMINALE\n');
fprintf('=================================================\n');
fprintf('alpha0 = %.4f rad = %.2f deg\n', ...
    alpha0, rad2deg(alpha0));
fprintf('beta0  = %.4f rad = %.2f deg\n', ...
    beta0, rad2deg(beta0));
fprintf('F10 = %.6f N\n', F10_nominal);
fprintf('F20 = %.6f N\n', F20_nominal);

%% ================================================================
%  4. INERZIA YAW NEL PUNTO DI EQUILIBRIO
%  ================================================================
J_beta0 = ...
    J_y * sin(alpha0)^2 ...
    + (J_z + m * l^2) * cos(alpha0)^2 ...
    + I_b;
J_beta0_nominal = J_beta0.NominalValue;
fprintf('J_beta(alpha0) nominale = %.8f kg m^2\n', ...
    J_beta0_nominal);

%% ================================================================
%  5. MATRICE A DEL MODELLO LINEARIZZATO
%  ================================================================
% Derivata della dinamica di pitch rispetto ad alpha
a21 = ...
    -(m * g * l * cos(alpha0)) ...
    / J_alpha;
% Derivata della dinamica di pitch rispetto ad alpha_dot
a22 = -c_alpha / J_alpha;
% Derivata della dinamica di pitch rispetto a beta
a23 = ...
    l * ( ...
        -F10 * sin(beta0) ...
        + epsilon_p * F20 * cos(beta0) ...
    ) / J_alpha;
% Derivata della dinamica yaw rispetto ad alpha
%
% Il termine contenente dJ_beta/dalpha si annulla nel punto di
% equilibrio perché il numeratore della dinamica yaw è nullo.
a41 = ...
    l * ( ...
        -F20 * sin(alpha0) ...
        + epsilon_y * F10 * cos(alpha0) ...
    ) / J_beta0;
% Derivata della dinamica yaw rispetto a beta_dot
a44 = -c_beta / J_beta0;
A_uncertain = [
    0,      1,      0,      0;
    a21,    a22,    a23,    0;
    0,      0,      0,      1;
    a41,    0,      0,      a44
];

%% ================================================================
%  6. MATRICE B DEL MODELLO LINEARIZZATO
%  ================================================================
b21 = l * cos(beta0) ...
    / J_alpha;
b22 = epsilon_p * l * sin(beta0) ...
    / J_alpha;
b41 = epsilon_y * l * sin(alpha0) ...
    / J_beta0;
b42 = l * cos(alpha0) ...
    / J_beta0;
B_uncertain = [
    0,      0;
    b21,    b22;
    0,      0;
    b41,    b42
];

%% ================================================================
%  7. MATRICI DI USCITA
%  ================================================================
% In questa prima rappresentazione tutti gli stati sono disponibili
% in uscita. La matrice di misura potrà essere modificata durante
% la progettazione del filtro di Kalman.
C_state = eye(4);
D_state = zeros(4,2);

%% ================================================================
%  8. COSTRUZIONE DEL MODELLO STATE-SPACE INCERTO
%  ================================================================
P_uncertain = uss( ...
    A_uncertain, ...
    B_uncertain, ...
    C_state, ...
    D_state);

%% Nomi degli stati, ingressi e uscite
P_uncertain.StateName = {
    'delta_alpha'
    'delta_alpha_dot'
    'delta_beta'
    'delta_beta_dot'
};
P_uncertain.InputName = {
    'delta_F1'
    'delta_F2'
};
P_uncertain.OutputName = {
    'delta_alpha'
    'delta_alpha_dot'
    'delta_beta'
    'delta_beta_dot'
};

%% Modello nominale
P_nominal = ss(P_uncertain.NominalValue);

%% ================================================================
% INGRESSI DI DISTURBO AERODINAMICO
%
% d = [d_alpha; d_beta] [N*m]
% ================================================================
Bd_uncertain = [
    0,              0;
    1/J_alpha,      0;
    0,              0;
    0,              1/J_beta0
    ];
Bext_uncertain = [
    B_uncertain, Bd_uncertain
    ];
P_uncertain_ext = uss( ...
    A_uncertain, ...
    Bext_uncertain, ...
    C_state, ...
    zeros(4,4));
P_uncertain_ext.StateName = P_uncertain.StateName;
P_uncertain_ext.InputName = {
    'delta_F1'
    'delta_F2'
    'd_alpha'
    'd_beta'
    };
P_uncertain_ext.OutputName = P_uncertain.OutputName;
P_nominal_ext = ss(P_uncertain_ext.NominalValue);

%% Trasferimento disturbi -> angoli
Gd_uncertain = P_uncertain_ext([1 3],[3 4]);
Gd_uncertain.InputName = {
    'd_alpha'
    'd_beta'
    };
Gd_uncertain.OutputName = {
    'delta_alpha'
    'delta_beta'
    };
Gd_nominal = ss(Gd_uncertain.NominalValue);

%% ================================================================
%  9. VISUALIZZAZIONE DEI RISULTATI
%  ================================================================
fprintf('\n=================================================\n');
fprintf('MATRICE A NOMINALE\n');
fprintf('=================================================\n');
disp(P_nominal.A);
fprintf('=================================================\n');
fprintf('MATRICE B NOMINALE\n');
fprintf('=================================================\n');
disp(P_nominal.B);
%fprintf('=================================================\n');
%fprintf('AUTOVALORI DEL MODELLO NOMINALE OPEN-LOOP\n');
%fprintf('=================================================\n');
%disp(eig(P_nominal.A));
fprintf('=================================================\n');
fprintf('RANGO DI CONTROLLABILITA''\n');
fprintf('=================================================\n');
controllability_rank = rank( ...
    ctrb(P_nominal.A, P_nominal.B));
fprintf('rank(ctrb(A,B)) = %d su %d\n', ...
    controllability_rank, size(P_nominal.A,1));
fprintf('=================================================\n');
fprintf('RANGO DI OSSERVABILITA'' CON C = I\n');
fprintf('=================================================\n');
observability_rank = rank( ...
    obsv(P_nominal.A, P_nominal.C));
fprintf('rank(obsv(A,C)) = %d su %d\n', ...
    observability_rank, size(P_nominal.A,1));

%% ================================================================
%  10. MODELLO CON SOLI ANGOLI MISURATI
%  ================================================================
% Questa rappresentazione sarà utile per il successivo LQG:
%
%   y = [delta_alpha;
%        delta_beta]
%
% Le velocità verranno stimate dal filtro di Kalman.
C_angles = [
    1, 0, 0, 0;
    0, 0, 1, 0
];
D_angles = zeros(2,2);
P_uncertain_angles = uss( ...
    A_uncertain, ...
    B_uncertain, ...
    C_angles, ...
    D_angles);
P_uncertain_angles.StateName = P_uncertain.StateName;
P_uncertain_angles.InputName = P_uncertain.InputName;
P_uncertain_angles.OutputName = {
    'delta_alpha'
    'delta_beta'
};
P_nominal_angles = ss(P_uncertain_angles.NominalValue);
angle_observability_rank = rank( ...
    obsv(P_nominal_angles.A, P_nominal_angles.C));
fprintf('=================================================\n');
fprintf('OSSERVABILITA'' MISURANDO SOLO ALPHA E BETA\n');
fprintf('=================================================\n');
fprintf('rank(obsv(A,C_angles)) = %d su %d\n', ...
    angle_observability_rank, size(P_nominal_angles.A,1));

%% Le principali variabili rimangono disponibili nel workspace:
%
% params
% x0
% u0_nominal
% u0_uncertain
% P_uncertain
% P_nominal
% P_uncertain_angles
% P_nominal_angles
u0_nominal = double(F0_nominal);
u0 = u0_nominal;
