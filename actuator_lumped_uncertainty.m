%% ========================================================================
% ACTUATOR_LUMPED_UNCERTAINTY
%
% Costruzione di un modello concentrato moltiplicativo degli attuatori:
%
% Gact_i_unc = Gact_i_nom * (1 + WI_i * Delta_i)
%
% da confrontare con il modello parametrico wn/td.
% ========================================================================

close all;
clc;

assert(exist('P_uncertain','var') == 1, ...
    'Eseguire prima build_uncertain_linear_model.m');

load('HINF_setup.mat');

rng(10);

%% ========================================================================
% 1. ATTUATORI PARAMETRICI SISO
% ========================================================================

G1_nom = Gact_nominal(1,1);
G2_nom = Gact_nominal(2,2);

G1_unc_param = Gact_uncertain(1,1);
G2_unc_param = Gact_uncertain(2,2);

Nsample = 200;

G1_samples = usample( ...
    G1_unc_param, ...
    Nsample);

G2_samples = usample( ...
    G2_unc_param, ...
    Nsample);

%% ========================================================================
% 2. FIT DELLE INCERTEZZE MOLTIPLICATIVE
% ========================================================================

OrderWt = 2;

[~,info1] = ...
    ucover( ...
        G1_samples, ...
        G1_nom, ...
        OrderWt, ...
        'InputMult');

[~,info2] = ...
    ucover( ...
        G2_samples, ...
        G2_nom, ...
        OrderWt, ...
        'InputMult');

WI1 = minreal(tf(info1.W1));
WI2 = minreal(tf(info2.W1));

fprintf('\n============================================================\n');
fprintf('PESI DI INCERTEZZA CONCENTRATA ATTUATORI\n');
fprintf('============================================================\n');

disp('WI1 =');
WI1

disp('WI2 =');
WI2

%% ========================================================================
% 3. VERIFICA GRAFICA DEL COVER
% ========================================================================

omega = logspace(-1,3,500);

Rel1 = ...
    (G1_samples-G1_nom)/G1_nom;

Rel2 = ...
    (G2_samples-G2_nom)/G2_nom;

figure('Name','Actuator 1 - lumped uncertainty');
sigma(Rel1,omega);
hold on;
sigma(WI1,omega);
grid on;
title('Attuatore 1: errori relativi e peso W_{I1}');

figure('Name','Actuator 2 - lumped uncertainty');
sigma(Rel2,omega);
hold on;
sigma(WI2,omega);
grid on;
title('Attuatore 2: errori relativi e peso W_{I2}');

%% ========================================================================
% 4. BLOCCHI ULTIDYN
% ========================================================================

Delta_act1 = ...
    ultidyn( ...
        'Delta_act1', ...
        [1 1]);

Delta_act2 = ...
    ultidyn( ...
        'Delta_act2', ...
        [1 1]);

G1_unc_lumped = ...
    G1_nom * ...
    (1 + WI1*Delta_act1);

G2_unc_lumped = ...
    G2_nom * ...
    (1 + WI2*Delta_act2);

Gact_lumped = blkdiag( ...
    G1_unc_lumped, ...
    G2_unc_lumped);

%% ========================================================================
% 5. RIDUZIONE DELLE INCERTEZZE MECCANICHE
%
% Modificare questa scelta se lo screening indica diversamente.
% ========================================================================

Pmech_reduced = P_uncertain;

parametersToNominal = {
    'J_y'
    'J_z'
};

for k = 1:numel(parametersToNominal)

    parName = parametersToNominal{k};

    if isfield(Pmech_reduced.Uncertainty,parName)

        block = ...
            Pmech_reduced.Uncertainty.(parName);

        Pmech_reduced = ...
            usubs( ...
                Pmech_reduced, ...
                parName, ...
                block.NominalValue);
    end
end

Pq_mech_reduced = ...
    Pmech_reduced([1 3],:);

%% ========================================================================
% 6. MODELLO FINALE PER MU-SYNTHESIS
% ========================================================================

G_uncertain_lumped = ...
    Pq_mech_reduced * ...
    Gact_lumped;

G_uncertain_lumped.InputName = {
    'delta_F1_cmd'
    'delta_F2_cmd'
};

G_uncertain_lumped.OutputName = {
    'delta_alpha'
    'delta_beta'
};

G_uncertain_lumped_scaled = ...
    Dy_inv * ...
    G_uncertain_lumped * ...
    Du;

%% ========================================================================
% 7. SALVATAGGIO
% ========================================================================

save( ...
    'ACTUATOR_LUMPED.mat', ...
    'WI1', ...
    'WI2', ...
    'Delta_act1', ...
    'Delta_act2', ...
    'Gact_lumped', ...
    'Pmech_reduced', ...
    'G_uncertain_lumped', ...
    'G_uncertain_lumped_scaled');

fprintf('\nACTUATOR_LUMPED.mat creato correttamente.\n');