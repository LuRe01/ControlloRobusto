function q_ddot = fcn(q, q_dot, F, d, delta, p0)
%#codegen
% DINAMICA NON LINEARE INCERTA DELL'ELICOTTERO 2DOF
%
% Coordinate generalizzate:
%   q = [alpha; beta]                         [rad]
%
% Velocita' generalizzate:
%   q_dot = [alpha_dot; beta_dot]             [rad/s]
%
% Forze effettivamente prodotte dai rotori:
%   F = [F1; F2]                              [N]
%
% Incertezze normalizzate:
%
%   delta(1) = delta_Jalpha    in [-1,1]
%   delta(2) = delta_Jy        in [-1,1]
%   delta(3) = delta_Jz        in [-1,1]
%   delta(4) = delta_m         in [-1,1]
%   delta(5) = delta_l         in [-1,1]
%   delta(6) = delta_epsilon_p in [-1,1]
%   delta(7) = delta_epsilon_y in [-1,1]
%
% Parametri nominali contenuti nella struttura p0:
%
%   p0.J_alpha
%   p0.J_y
%   p0.J_z
%   p0.I_b
%   p0.m
%   p0.l
%   p0.c_alpha
%   p0.c_beta
%   p0.epsilon_p
%   p0.epsilon_y
%   p0.g
%
% Modello di incertezza:
%
%   p = p_nominale * (1 + r_p * delta_p)
%
% con delta_p appartenente all'intervallo [-1,1].

%% ================================================================
%  1. ESTRAZIONE DEGLI STATI E DEGLI INGRESSI
%  ================================================================

alpha = q(1);
beta  = q(2);

alpha_dot = q_dot(1);
beta_dot  = q_dot(2);

F1 = F(1);
F2 = F(2);

%% ================================================================
%  2. ESTRAZIONE DELLE INCERTEZZE NORMALIZZATE
%  ================================================================

delta_Jalpha   = delta(1);
delta_Jy       = delta(2);
delta_Jz       = delta(3);
delta_m        = delta(4);
delta_l        = delta(5);
delta_epsilonp = delta(6);
delta_epsilony = delta(7);

%% ================================================================
%  3. PERCENTUALI DI INCERTEZZA
%  ================================================================

% J_alpha: +/-10 %
r_Jalpha = 0.10;

% J_y: +/-15 %
r_Jy = 0.15;

% J_z: +/-10 %
r_Jz = 0.10;

% Massa: +/-10 %
r_m = 0.10;

% Lunghezza del braccio: +/-5 %
r_l = 0.05;

% Coefficienti di coupling aerodinamico: +/-30 %
r_epsilonp = 0.30;
r_epsilony = 0.30;

%% ================================================================
%  4. CALCOLO DEI PARAMETRI INCERTI
%  ================================================================

J_alpha = p0.J_alpha * ...
    (1 + r_Jalpha * delta_Jalpha);

J_y = p0.J_y * ...
    (1 + r_Jy * delta_Jy);

J_z = p0.J_z * ...
    (1 + r_Jz * delta_Jz);

m = p0.m * ...
    (1 + r_m * delta_m);

l = p0.l * ...
    (1 + r_l * delta_l);

epsilon_p = p0.epsilon_p * ...
    (1 + r_epsilonp * delta_epsilonp);

epsilon_y = p0.epsilon_y * ...
    (1 + r_epsilony * delta_epsilony);

%% Parametri considerati perfettamente noti

I_b = p0.I_b;

c_alpha = p0.c_alpha;
c_beta  = p0.c_beta;

g = p0.g;

%% ================================================================
%  5. INERZIA VARIABILE DELL'ASSE YAW
%  ================================================================

J_beta = ...
    J_y * sin(alpha)^2 ...
    + (J_z + m * l^2) * cos(alpha)^2 ...
    + I_b;

%% ================================================================
%  6. MATRICE DI INERZIA
%  ================================================================

M = [
    J_alpha, 0;
    0,       J_beta
];

%% ================================================================
%  7. ATTRITI VISCHIOSI E GRAVITA'
%  ================================================================

n = [
    c_alpha * alpha_dot ...
        + m * g * l * sin(alpha);

    c_beta * beta_dot
];

%% ================================================================
%  8. MATRICE NON LINEARE DI DISTRIBUZIONE DELLE FORZE
%  ================================================================

G = l * [
    cos(beta),                  epsilon_p * sin(beta);
    epsilon_y * sin(alpha),     cos(alpha)
];

%% ================================================================
%  9. DISTURBI AERODINAMICI
%  ================================================================

d_alpha = d(1);
d_beta  = d(2);

tau_dist = [
    d_alpha;
    d_beta
    ];

%% ================================================================
%  10. DINAMICA DIRETTA
%  ================================================================

q_ddot = M \ (G * [F1; F2] - n + tau_dist);

end