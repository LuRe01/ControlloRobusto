# Robust Control 2DOF Helicopter — MATLAB/Simulink package

Pacchetto finale del progetto di controllo robusto dell'elicottero 2DOF. Lo script orchestratore e' `run_project.m`.

## Avvio

1. Aprire MATLAB nella cartella del pacchetto.
2. Verificare Control System Toolbox, Robust Control Toolbox e Simulink.
3. Eseguire:

```matlab
run_project
```

`run_project.m` mantiene il workspace tra le fasi dipendenti, esporta le figure in PNG a 300 dpi, raccoglie i CSV in `analysis_result/` e al termine lancia la campagna Simulink.

## Ordine di esecuzione

1. `build_uncertain_linear_model.m`
2. `init.m`
3. `LQG_2DOF_Synthesis.m`
4. `HINF_setup.m`
5. `HINF_synthesis.m`
6. `HINF_analysis.m`
7. `actuator_lumped_uncertainty.m`
8. `MU_synthesis.m`
9. `MU_analysis_comparison.m`
10. `Monte_Carlo_robustness.m`
11. `RAS_vectorized.m`
12. `run_simulink_simulations.m`

`fcn.m` contiene la dinamica non lineare usata dal modello Simulink.

## Output

- `result_plot/`: figure MATLAB esportate a 300 dpi.
- `analysis_result/`: CSV riassuntivi delle analisi.
- `simulink_result/`: risultati e confronti della campagna Simulink.
- i `.mat` intermedi/finali restano nella cartella principale per preservare le dipendenze degli script.

## Convenzioni per l'interpretazione dei risultati

- Per il confronto mu ufficiale H-infinity vs mu-synthesis usare `MU_analysis_comparison.m` e `analysis_result/MU_vs_HINF_results.csv`. La mu esplicita contenuta in `HINF_analysis.m` e' una diagnostica ausiliaria e usa una formulazione storica leggermente diversa del blocco fittizio di performance; per questo i numeri possono differire di pochi punti percentuali senza cambiare le conclusioni.
- Il Monte Carlo e' una validazione numerica complementare: non sostituisce la certificazione worst-case/mu.
- Le mappe prodotte da `RAS_vectorized.m` sono stime numeriche a orizzonte finito. La classificazione indica il raggiungimento o meno del criterio di convergenza entro `T_f`, non una prova analitica di stabilita'/instabilita' asintotica.
- La cache compilata `.slxc` non e' distribuita: Simulink la rigenera localmente quando necessaria.

## Configurazione H-infinity definitiva

- `Ms_alpha = 1.45`, `Ms_beta = 1.50`
- `As_alpha = As_beta = 0.015`
- `wb_alpha = 3.8 rad/s`, `wb_beta = 3.0 rad/s`
- `WU = 0.90 I`
- `wt_alpha = 22 rad/s`, `wt_beta = 18 rad/s`
- requisiti temporali: settling pitch <= 2.0 s, settling yaw <= 2.5 s, overshoot <= 10% su entrambi gli assi.

La configurazione finale privilegia un compromesso tra prestazione nominale, azione di controllo e robustezza; non va interpretata come massimizzazione della sola banda.
