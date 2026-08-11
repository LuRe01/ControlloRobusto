function q_ddot = helicopter_nonlinear_uncertain(q, q_dot, F, params)
% HELICOPTER_NONLINEAR_UNCERTAIN
% Dinamica non lineare dell'elicottero 2DOF.
%
% La funzione accetta sia parametri numerici sia parametri incerti UREAL.
%
% INGRESSI
%   q       = [alpha; beta]                 [rad]
%   q_dot   = [alpha_dot; beta_dot]         [rad/s]
%   F       = [F1; F2]                      [N]
%   params  = struttura contenente:
%
%       params.J_alpha
%       params.J_y
%       params.J_z
%       params.I_b
%       params.m
%       params.l
%       params.c_alpha
%       params.c_beta
%       params.epsilon_p
%       params.epsilon_y
%       params.g
%
% USCITA
%   q_ddot = [alpha_ddot; beta_ddot]        [rad/s^2]
%
% La funzione può essere utilizzata:
%   - con parametri numerici;
%   - con oggetti ureal per analisi MATLAB;
%   - con campioni numerici ottenuti mediante usample per Simulink.
%
% NOTA:
% Gli oggetti UREAL non possono essere usati direttamente all'interno
% di un blocco Simulink MATLAB Function soggetto a code generation.
% Per il modello Simulink non lineare occorre prima campionare i
% parametri incerti e passare valori numerici al blocco.

%% Estrazione delle coordinate generalizzate

alpha = q(1);
beta  = q(2);

alpha_dot = q_dot(1);
beta_dot  = q_dot(2);

F1 = F(1);
F2 = F(2);

%% Inerzia variabile dell'asse yaw

J_beta = params.J_y * sin(alpha)^2 ...
       + (params.J_z + params.m * params.l^2) * cos(alpha)^2 ...
       + params.I_b;

%% Matrice di inerzia

M = [params.J_alpha, 0;
     0,              J_beta];

%% Gravità e attriti viscosi

n = [
    params.c_alpha * alpha_dot ...
        + params.m * params.g * params.l * sin(alpha);

    params.c_beta * beta_dot
];

%% Matrice non lineare di distribuzione delle forze

G = params.l * [
    cos(beta),                       params.epsilon_p * sin(beta);
    params.epsilon_y * sin(alpha),   cos(alpha)
];

%% Dinamica diretta

q_ddot = M \ (G * [F1; F2] - n);

end