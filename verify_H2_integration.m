%% VERIFY_H2_INTEGRATION
% Verifica statica minima della catena H2 prima della campagna completa.

clc;
requiredFiles = { ...
    'H2_synthesis.m','H2_analysis.m','HINF_setup.mat', ...
    'helicopter_2DOF.slx'};
for k = 1:numel(requiredFiles)
    assert(isfile(requiredFiles{k}),'File mancante: %s',requiredFiles{k});
end

if ~isfile('H2_controller.mat')
    fprintf('H2_controller.mat non ancora presente: lancio H2_synthesis.m...\n');
    H2_synthesis;
end
load('H2_controller.mat','K_h2','K_h2_scaled','gamma_h2');
assert(isa(K_h2,'ss') || isa(K_h2,'tf') || isa(K_h2,'zpk'), ...
    'K_h2 non e'' un modello LTI valido.');
assert(size(K_h2,1)==2 && size(K_h2,2)==2,'K_h2 deve essere 2x2.');

load_system('helicopter_2DOF');
h2blk = 'helicopter_2DOF/HINF_Controller/h2_controller';
assert(~isempty(find_system(h2blk,'SearchDepth',0)), ...
    'Blocco Simulink H2 non trovato. Eseguire install_H2_controller_in_simulink.m.');

fprintf('Verifica H2 completata. Norma H2 di sintesi: %.6g\n',gamma_h2);
fprintf('Selettore Simulink: HINF_controller_id = 5.\n');
