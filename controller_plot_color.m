function color = controller_plot_color(name)
%CONTROLLER_PLOT_COLOR Colore globale coerente per ciascun controllore.
%
% Questa funzione centralizza la palette usata in tutti i grafici di
% confronto del progetto. In questo modo lo stesso controllore mantiene
% sempre lo stesso colore indipendentemente dall'ordine con cui viene
% rappresentato nella figura.
%
% Restituisce [] per etichette che non identificano un controllore
% (ad esempio "Linearizzato" o "Interno al criterio"), lasciando in quel
% caso a MATLAB la scelta del colore.

    key = lower(regexprep(char(string(name)),'[^a-zA-Z0-9]',''));

    if contains(key,'lqgi')
        color = [0.8500 0.3250 0.0980];  % arancio
    elseif contains(key,'lqg')
        color = [0.0000 0.4470 0.7410];  % blu
    elseif contains(key,'mixsyn')
        color = [0.9290 0.6940 0.1250];  % giallo/oro
    elseif contains(key,'hinfsyn')
        color = [0.4940 0.1840 0.5560];  % viola
    elseif contains(key,'pid')
        color = [0.4660 0.6740 0.1880];  % verde
    elseif contains(key,'musynthesis') || strcmp(key,'mu')
        color = [0.3010 0.7450 0.9330];  % azzurro/ciano
    elseif strcmp(key,'h2') || contains(key,'h2controller')
        color = [0.6350 0.0780 0.1840];  % rosso scuro
    else
        color = [];
    end
end
