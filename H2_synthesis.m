%% NOTE TEORICHE - SINTESI H2 PURA
% Il controllo H2 minimizza l'energia quadratica media del closed loop,
% cioe' la norma H2 del trasferimento dagli ingressi esogeni alle uscite
% di prestazione. Per rendere finita la norma H2 in continuo, il peso sulla
% sensibilita' deve essere strettamente proprio: viene quindi ricavato dal
% peso WS usato nel progetto H-infinity aggiungendo un roll-off passa-basso
% ad alta frequenza. WU e WT restano invariati, cosi' il confronto con i
% controllori robusti mantiene lo stesso significato fisico.
%
% Il problema sintetizzato e':
%
%   min_K || [ WS_H2*S ; WU*K*S ; WT*T ] ||_2
%
% sul medesimo plant nominale normalizzato G_scaled. L'ottimo H2 non offre
% per costruzione garanzie worst-case: il controllore viene quindi sottoposto
% in seguito alle stesse analisi H-infinity, mu, Monte Carlo, RAS e Simulink.
%
% MATLAB / Robust Control Toolbox R2026a.

close all;
clc;

%% ========================================================================
% 1. DATI DI PROGETTO
% =========================================================================
if ~isfile('HINF_setup.mat')
    error('HINF_setup.mat non trovato. Eseguire prima HINF_setup.m.');
end
load('HINF_setup.mat');

I2 = eye(2);
s = tf('s');
nmeas = 2;
ncont = 2;

%% ========================================================================
% 2. PESO H2 SULLA SENSIBILITA'
% =========================================================================
% WS originale ha guadagno non nullo ad alta frequenza. Poiche' S -> I per
% omega -> infinito, WS*S non sarebbe strettamente proprio e la norma H2
% sarebbe infinita. Si introduce quindi un roll-off un decennio circa sopra
% la banda richiesta. Alle frequenze di tracking il peso resta praticamente
% uguale a WS; alle alte frequenze WT continua a penalizzare T.

h2Design.rolloffFactor = 10;
h2Design.rolloffAlpha = h2Design.rolloffFactor * weight.wb_alpha;
h2Design.rolloffBeta  = h2Design.rolloffFactor * weight.wb_beta;

FrollAlpha = h2Design.rolloffAlpha / (s + h2Design.rolloffAlpha);
FrollBeta  = h2Design.rolloffBeta  / (s + h2Design.rolloffBeta);

WS_H2_alpha = minreal(ss(WS(1,1) * FrollAlpha),1e-9);
WS_H2_beta  = minreal(ss(WS(2,2) * FrollBeta),1e-9);
WS_H2 = blkdiag(WS_H2_alpha,WS_H2_beta);

%% ========================================================================
% 3. PLANT GENERALIZZATO H2
% =========================================================================
% augw usa lo stesso schema mixed-sensitivity del progetto H-infinity:
% w -> errore di tracking, u -> comando, y -> errore misurato dal controller.
% Con WS_H2 strettamente proprio il closed loop H2 ha norma finita.

P_H2 = augw(G_scaled,WS_H2,WU,WT);

%% ========================================================================
% 4. SINTESI H2
% =========================================================================
fprintf('\n============================================================\n');
fprintf('SINTESI H2 PURA\n');
fprintf('============================================================\n');

% h2syn assume che gli ultimi nmeas output siano le misure e gli ultimi
% ncont input siano i comandi. La regolarizzazione automatica viene lasciata
% attiva per robustezza numerica.
[K_h2_scaled,CL_h2,gamma_h2,info_h2] = ...
    h2syn(P_H2,nmeas,ncont);

K_h2_scaled = minreal(ss(K_h2_scaled),1e-7);
K_h2 = minreal(Du*K_h2_scaled*Dy_inv,1e-7);

% Verifica indipendente della norma H2 del closed loop generalizzato.
CL_h2_check = minreal(lft(P_H2,K_h2_scaled),1e-7);
gamma_h2_verified = norm(CL_h2_check,2);

%% ========================================================================
% 5. VERIFICHE NOMINALI AGGIUNTIVE
% =========================================================================
L_h2 = G_scaled*K_h2_scaled;
S_h2 = feedback(I2,L_h2);
T_h2 = feedback(L_h2,I2);
KS_h2 = K_h2_scaled*S_h2;

stable_h2 = all(real(pole(T_h2))<0);
if ~stable_h2 || ~isfinite(gamma_h2_verified)
    error('La sintesi H2 non ha prodotto un closed loop nominale stabile con norma finita.');
end
weightedHinf_h2 = hinfnorm(minreal([ ...
    WS*S_h2; ...
    WU*KS_h2; ...
    WT*T_h2],1e-7));

fprintf('Norma H2 ottima       = %.8f\n',gamma_h2);
fprintf('Norma H2 verificata   = %.8f\n',gamma_h2_verified);
fprintf('Norma Hinf pesata     = %.8f\n',weightedHinf_h2);
fprintf('Ordine K_H2           = %d\n',order(K_h2_scaled));
fprintf('Stabilita'' nominale   = %d\n',stable_h2);

%% ========================================================================
% 6. FIGURA DEL PESO H2
% =========================================================================
omegaH2 = omegaHinf;
figure('Name','H2 sensitivity weight and H-infinity baseline');

% I pesi sono diagonali: si mostrano esplicitamente i canali di pitch
% (linea continua) e yaw (linea tratteggiata). Lo stesso colore identifica
% la stessa famiglia di peso, evitando ambiguita' tra le quattro tracce.
weightColors = lines(2);
magWSAlpha   = squeeze(abs(freqresp(WS(1,1),omegaH2)));
magWSBeta    = squeeze(abs(freqresp(WS(2,2),omegaH2)));
magH2Alpha   = squeeze(abs(freqresp(WS_H2(1,1),omegaH2)));
magH2Beta    = squeeze(abs(freqresp(WS_H2(2,2),omegaH2)));

semilogx(omegaH2,20*log10(magWSAlpha),'-', ...
    'Color',weightColors(1,:),'LineWidth',1.4); hold on;
semilogx(omegaH2,20*log10(magWSBeta),'--', ...
    'Color',weightColors(1,:),'LineWidth',1.4);
semilogx(omegaH2,20*log10(magH2Alpha),'-', ...
    'Color',weightColors(2,:),'LineWidth',1.4);
semilogx(omegaH2,20*log10(magH2Beta),'--', ...
    'Color',weightColors(2,:),'LineWidth',1.4);

grid on;
xlabel('$\omega$ [rad/s]','Interpreter','latex');
ylabel('Magnitude [dB]');
title('Confronto tra $W_S$ e il peso strettamente proprio $W_{S,H_2}$', ...
    'Interpreter','latex');
legend({'$W_{S,\alpha}$','$W_{S,\beta}$', ...
        '$W_{S,H_2,\alpha}$','$W_{S,H_2,\beta}$'}, ...
    'Interpreter','latex','Location','best');

%% ========================================================================
% 7. SALVATAGGIO
% =========================================================================
save('H2_controller.mat', ...
    'K_h2_scaled','K_h2','CL_h2','CL_h2_check', ...
    'gamma_h2','gamma_h2_verified','info_h2', ...
    'WS_H2','WS_H2_alpha','WS_H2_beta', ...
    'FrollAlpha','FrollBeta','h2Design', ...
    'weightedHinf_h2','stable_h2');

H2SynthesisResults = table( ...
    string('H2'),order(K_h2_scaled),gamma_h2,gamma_h2_verified, ...
    weightedHinf_h2,stable_h2,h2Design.rolloffAlpha,h2Design.rolloffBeta, ...
    'VariableNames',{ ...
    'Controller','Order','H2NormSynthesis','H2NormVerified', ...
    'WeightedHinfNorm','NominalStable','WSH2RolloffAlpha_rad_s', ...
    'WSH2RolloffBeta_rad_s'});

writetable(H2SynthesisResults,'H2_synthesis_results.csv');

fprintf('H2_controller.mat creato correttamente.\n');
