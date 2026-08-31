%% INSTALL_H2_CONTROLLER_IN_SIMULINK
% Utility di verifica/ripristino dell'integrazione del controllore H2 nel
% modello helicopter_2DOF.slx. Il pacchetto distribuito contiene gia' il
% blocco H2; questo script serve come fallback qualora il modello venga
% sostituito con una versione precedente.

close all;
clc;

model = 'helicopter_2DOF';
if ~isfile([model '.slx'])
    error('Modello %s.slx non trovato.',model);
end
if ~isfile('H2_controller.mat')
    error('H2_controller.mat non trovato. Eseguire prima H2_synthesis.m.');
end

load_system(model);
subsys = [model '/HINF_Controller'];
sw = [subsys '/Multiport Switch4'];
h2blk = [subsys '/h2_controller'];

if ~isempty(find_system(subsys,'SearchDepth',1,'Name','h2_controller'))
    fprintf('Il blocco H2 e'' gia'' presente nel modello.\n');
    set_param(sw,'Inputs','5');
    save_system(model);
    return
end

add_block('simulink/Continuous/State-Space',h2blk, ...
    'A','K_h2.A', ...
    'B','K_h2.B', ...
    'C','K_h2.C', ...
    'D','K_h2.D', ...
    'InitialCondition','zeros(size(K_h2.A,1),1)', ...
    'Position',[1970 468 2030 502]);

% Il controllore riceve lo stesso errore stimato degli altri controllori.
add_line(subsys,'Sum7/1','h2_controller/1','autorouting','on');

% Multiport Switch: selector + 5 ingressi dati.
set_param(sw,'Inputs','5');
add_line(subsys,'h2_controller/1','Multiport Switch4/6','autorouting','on');

save_system(model);
fprintf('Controllore H2 aggiunto. Usare HINF_controller_id = 5.\n');
