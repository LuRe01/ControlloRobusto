%% NOTE TEORICHE - INIZIALIZZAZIONE DEL MODELLO NON LINEARE
% Definisce parametri nominali, punto di equilibrio, attuatori, sensori,
% rumori, disturbi e riferimenti usati dal modello Simulink e dalle
% validazioni. Questi dati costituiscono il riferimento nominale rispetto
% al quale vengono introdotte e campionate le incertezze.
%

%% Parametri nominali del modello non lineare
p0.J_alpha = 0.012;       % kg m^2
p0.J_y = 0.00023;         % kg m^2
p0.J_z = 0.00364;         % kg m^2
p0.I_b = 0.00023;         % kg m^2
p0.m = 0.2;               % kg
p0.l = 0.2;               % m
p0.c_alpha = 0.01;        % N m s/rad
p0.c_beta  = 0.01;        % N m s/rad
p0.epsilon_p = 0.1;
p0.epsilon_y = 0.1;
p0.g = 9.81;              % m/s^2

%% Punto di equilibrio
alpha0 = deg2rad(15);
beta0  = deg2rad(20);
q0     = [alpha0; beta0];
qdot0  = [0; 0];
x0 = [alpha0; 0; beta0; 0];

%% Incertezze normalizzate del plant non lineare
delta_plant = zeros(7,1);   % modello nominale

%% Forze nominali di equilibrio
E0 = [
    cos(beta0),                  p0.epsilon_p*sin(beta0);
    p0.epsilon_y*sin(alpha0),    cos(alpha0)
    ];
u0 = E0 \ [
    p0.m*p0.g*sin(alpha0);
    0
    ];

%% Attuatori
act.wn1 = 40;
act.wn2 = 40;
act.zeta1 = 0.75;
act.zeta2 = 0.75;
act.td1 = 0.015;
act.td2 = 0.015;

%% Rumore di attuazione
act.Ts_noise = 1e-3;
act.sigma_F1 = 0.01;   % N
act.sigma_F2 = 0.01;   % N
act.noisePower_F1 = act.sigma_F1^2 * act.Ts_noise;
act.noisePower_F2 = act.sigma_F2^2 * act.Ts_noise;
act.noiseEnable = 0;

%% Saturazioni sulle variazioni di forza
act.deltaF1_min = -0.45;
act.deltaF1_max =  2.20;
act.deltaF2_min = -2.50;
act.deltaF2_max =  2.50;

%% ========================================================================
% SENSORI
% ========================================================================
sensor.seed = 12345;
sensor.noiseEnable = 0; % mettere a 0 per la RAS

%% ========================================================================
% VECTORNAV VN-100
% ========================================================================
% Accelerometro
sensor.acc.fs = 230;              % [Hz] Frequenza di simulazione scelta
sensor.acc.Ts = 1/sensor.acc.fs;  % [s]
% Noise Density (datasheet VN-100)
sensor.acc.ND = 0.14e-3 * 9.81;   % [m/s^2/sqrt(Hz)]
% Deviazione standard per campione
sensor.acc.sigma = sensor.acc.ND * sqrt(sensor.acc.fs);
% Varianza
sensor.acc.var = sensor.acc.sigma^2;

%% ========================================================================
% Magnetometro
% ========================================================================
sensor.mag.fs = 200;              % [Hz]
sensor.mag.Ts = 1/sensor.mag.fs;  % [s]
% Intensità del campo magnetico locale
% (da aggiornare eventualmente con il valore reale del sito sperimentale)
sensor.mag.B0 = 46.5;             % [uT]
% Noise Density VN-100
sensor.mag.ND = 0.014;            % [uT/sqrt(Hz)]
sensor.mag.sigma = sensor.mag.ND * sqrt(sensor.mag.fs);
sensor.mag.var = sensor.mag.sigma^2;

%% ========================================================================
% DISTURBI AERODINAMICI DI TEST
% ========================================================================
aero.enable = 1;
aero.alpha.time      = 6;
aero.alpha.amplitude = 5e-3;      % [N*m]
aero.beta.time       = 10;
aero.beta.amplitude  = 2e-3;      % [N*m]

%% ========================================================================
% Riferimenti
% ========================================================================

%% test 1: solo pitch
ref.alpha.initial = alpha0;
ref.alpha.final   = alpha0 + deg2rad(3);
ref.alpha.time    = 2;
ref.beta.initial  = beta0;
ref.beta.final    = beta0;
ref.beta.time     = 2;

%% test 2: solo yaw
% ref.alpha.final = alpha0;
% ref.beta.final = beta0 + deg2rad(5);
% ref.beta.time  = 2;

%% test 3: movimento combinato
% ref.alpha.final = alpha0 + deg2rad(3);
% ref.alpha.time  = 2;
% ref.beta.final  = beta0 - deg2rad(5);
% ref.beta.time   = 5;

%% ========================================================================
% HINF controller selection
% ========================================================================
HINF_controller_id = 4;  % 1: mixsyn, 2: hinfsyn, 3: PID+comp, 4: mu-synthesis, 5: H2
LQG_controller_id = 1;  % 1 = LQGI, 2 = LQG
