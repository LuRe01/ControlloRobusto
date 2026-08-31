%% RUN - ESECUZIONE COMPLETA DEL PROGETTO DI CONTROLLO ROBUSTO
% Esegue gli script nell'ordine richiesto dalle loro dipendenze, salva
% automaticamente tutte le figure in PNG (300 dpi) nella cartella
% result_plot e raccoglie i risultati CSV nella cartella analysis_result.
%
% IMPORTANTE:
% - il file orchestra la baseline robusta e la nuova sintesi H2;
% - build_uncertain_linear_model.m viene eseguito per primo perche' crea
%   P_nominal/P_uncertain e contiene intenzionalmente clearvars;
% - init.m viene eseguito subito dopo per ripristinare i parametri nominali,
%   attuatori e sensori richiesti dagli script successivi;
% - il workspace viene poi mantenuto tra gli script, come richiesto dalle
%   dipendenze originali del progetto.

close all;
clc;

% Porta MATLAB nella cartella del pacchetto.
projectRoot = fileparts(mfilename('fullpath'));
if isempty(projectRoot)
    projectRoot = pwd;
end
cd(projectRoot);
addpath(projectRoot);

fprintf('\n============================================================\n');
fprintf('ROBUST CONTROL 2DOF - ESECUZIONE COMPLETA\n');
fprintf('============================================================\n');

%% 1. Modello lineare incerto
fprintf('\n[01/12] build_uncertain_linear_model.m\n');
build_uncertain_linear_model;

%% 2. Parametri del modello non lineare / Simulink
fprintf('\n[02/12] init.m\n');
init;

% build_uncertain_linear_model contiene clearvars: le cartelle di output
% vengono quindi definite soltanto dopo la sua esecuzione.
projectRoot = pwd;
resultPlotDir = fullfile(projectRoot,'result_plot');
analysisResultDir = fullfile(projectRoot,'analysis_result');

if ~exist(resultPlotDir,'dir')
    mkdir(resultPlotDir);
end
if ~exist(analysisResultDir,'dir')
    mkdir(analysisResultDir);
end

% Pulisce solo i vecchi PNG/CSV generati da precedenti esecuzioni complete.
deleteIfPresent(fullfile(resultPlotDir,'*.png'));
deleteIfPresent(fullfile(analysisResultDir,'*.csv'));

%% 3. Sintesi LQG/LQGI
fprintf('\n[03/12] LQG_2DOF_Synthesis.m\n');
LQG_2DOF_Synthesis;
saveOpenFigures(resultPlotDir,'03_LQG_2DOF_Synthesis');

%% 4. Setup comune H-infinity / H2
fprintf('\n[04/12] HINF_setup.m\n');
HINF_setup;
saveOpenFigures(resultPlotDir,'04_HINF_setup');

%% 5. Sintesi H-infinity
fprintf('\n[05/12] HINF_synthesis.m\n');
HINF_synthesis;
saveOpenFigures(resultPlotDir,'05_HINF_synthesis');

%% 6. Sintesi H2 pura
fprintf('\n[06/12] H2_synthesis.m\n');
H2_synthesis;
saveOpenFigures(resultPlotDir,'06_H2_synthesis');

%% 7. Analisi H2 dedicata
fprintf('\n[07/12] H2_analysis.m\n');
H2_analysis;
saveOpenFigures(resultPlotDir,'07_H2_analysis');

%% 8. Analisi comparativa nominale/robusta
fprintf('\n[08/12] HINF_analysis.m\n');
HINF_analysis;
saveOpenFigures(resultPlotDir,'08_HINF_analysis');

%% 9. Incertezza concentrata degli attuatori
fprintf('\n[09/12] actuator_lumped_uncertainty.m\n');
actuator_lumped_uncertainty;
saveOpenFigures(resultPlotDir,'09_actuator_lumped_uncertainty');

%% 10. Mu-synthesis
fprintf('\n[10/12] MU_synthesis.m\n');
MU_synthesis;
saveOpenFigures(resultPlotDir,'10_MU_synthesis');

%% 11. Analisi mu e validazione Monte Carlo
fprintf('\n[11/12] MU_analysis_comparison.m + Monte_Carlo_robustness.m\n');
MU_analysis_comparison;
saveOpenFigures(resultPlotDir,'11a_MU_analysis_comparison');

Monte_Carlo_robustness;
saveOpenFigures(resultPlotDir,'11b_Monte_Carlo_robustness');

%% 12. Stima numerica della regione di attrazione
fprintf('\n[12/12] RAS_vectorized.m\n');
RAS_vectorized;
saveOpenFigures(resultPlotDir,'12_RAS_vectorized');

%% Raccolta dei file CSV prodotti dagli script
csvFiles = dir(fullfile(projectRoot,'*.csv'));
for k = 1:numel(csvFiles)
    sourceFile = fullfile(csvFiles(k).folder,csvFiles(k).name);
    destinationFile = fullfile(analysisResultDir,csvFiles(k).name);

    if isfile(destinationFile)
        delete(destinationFile);
    end
    movefile(sourceFile,destinationFile);
end

fprintf('\n============================================================\n');
fprintf('ESECUZIONE COMPLETATA\n');
fprintf('Figure PNG: %s\n',resultPlotDir);
fprintf('Risultati CSV: %s\n',analysisResultDir);
fprintf('LANCIO DELLE SIMULAZIONI SIMULINK\n');
fprintf('============================================================\n');
run_simulink_simulations;

%% ========================================================================
% FUNZIONI LOCALI DI SOLA REPORTISTICA
% ========================================================================

function saveOpenFigures(outputDir,scriptTag)
%SAVEOPENFIGURES Esporta in PNG tutte le figure aperte dallo script appena
% eseguito. I nomi derivano dalla proprieta' Figure.Name e vengono resi
% compatibili con il filesystem. Nessun dato del grafico viene modificato.

    figs = findall(groot,'Type','figure');

    if isempty(figs)
        return;
    end

    % Ordine crescente per numero figura, per rendere deterministico l'output.
    [~,idx] = sort([figs.Number]);
    figs = figs(idx);

    usedNames = strings(0,1);

    for iFig = 1:numel(figs)
        fig = figs(iFig);

        figName = string(fig.Name);
        if strlength(strtrim(figName)) == 0
            figName = "Figure_" + string(fig.Number);
        end

        safeName = regexprep(figName,'[^A-Za-z0-9_-]+','_');
        safeName = regexprep(safeName,'_+','_');
        safeName = strip(safeName,'_');

        if strlength(safeName) == 0
            safeName = "Figure_" + string(fig.Number);
        end

        baseName = string(scriptTag) + "__" + safeName;
        candidate = baseName;
        suffix = 2;

        while any(usedNames == candidate)
            candidate = baseName + "_" + string(suffix);
            suffix = suffix + 1;
        end
        usedNames(end+1,1) = candidate; %#ok<AGROW>

        pngPath = fullfile(outputDir,char(candidate + ".png"));

        try
            exportgraphics(fig,pngPath,'Resolution',300);
        catch
            % Fallback compatibile con figure per cui exportgraphics fallisce.
            print(fig,pngPath,'-dpng','-r300');
        end
    end
end

function deleteIfPresent(pattern)
%DELETEIFPRESENT Elimina i file che corrispondono al pattern indicato.

    files = dir(pattern);
    for iFile = 1:numel(files)
        delete(fullfile(files(iFile).folder,files(iFile).name));
    end
end
