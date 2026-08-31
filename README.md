# Robust Control 2DOF Helicopter — MATLAB/Simulink package

Pacchetto per MATLAB/Simulink R2026a relativo alla Traccia n. 3 di Controllo Robusto sul modello di elicottero 2DOF.

La versione corrente aggiunge un **controllore H2 puro** ai controllori gia' presenti:

- LQG;
- LQGI;
- H-infinity `mixsyn`;
- H-infinity `hinfsyn`;
- PID + compensatore sintetizzato con `hinfstruct`;
- mu-synthesis;
- **H2 con `h2syn`**.

## Requisiti

- MATLAB R2026a;
- Control System Toolbox;
- Robust Control Toolbox;
- Simulink.

## Avvio consigliato

Aprire MATLAB nella cartella del pacchetto ed eseguire:

```matlab
run_project
```

`run_project.m` rigenera i controllori, tutte le analisi, i CSV, le figure e infine la campagna Simulink.

## Ordine di esecuzione

1. `build_uncertain_linear_model.m`
2. `init.m`
3. `LQG_2DOF_Synthesis.m`
4. `HINF_setup.m`
5. `HINF_synthesis.m`
6. `H2_synthesis.m`
7. `H2_analysis.m`
8. `HINF_analysis.m`
9. `actuator_lumped_uncertainty.m`
10. `MU_synthesis.m`
11. `MU_analysis_comparison.m` + `Monte_Carlo_robustness.m`
12. `RAS_vectorized.m`
13. `run_simulink_simulations.m`

## Formulazione H2

Il controllore H2 viene sintetizzato sullo stesso plant normalizzato usato per H-infinity.
L'obiettivo e' una mixed-sensitivity in norma H2:

```text
min_K || [ WS_H2*S ; WU*K*S ; WT*T ] ||_2
```

Il peso `WS_H2` deriva dal peso H-infinity `WS`, ma viene reso strettamente proprio tramite un roll-off passa-basso ad alta frequenza. Questo passaggio e' necessario affinche' la norma H2 continua sia finita, dato che `S(jw) -> I` ad alta frequenza.

La sintesi e' eseguita da:

```matlab
[K_h2_scaled,CL_h2,gamma_h2,info_h2] = h2syn(P_H2,2,2);
```

Il controllore fisico viene ricostruito con la stessa normalizzazione del progetto:

```matlab
K_h2 = Du*K_h2_scaled*Dy_inv;
```

`H2_controller.mat` viene creato automaticamente da `H2_synthesis.m`; non e' necessario fornirlo gia' calcolato nel pacchetto.

## Integrazione Simulink

`helicopter_2DOF.slx` contiene gia' il blocco `HINF_Controller/h2_controller`.

La convenzione del selettore e':

```text
HINF_controller_id = 1  -> mixsyn
HINF_controller_id = 2  -> hinfsyn
HINF_controller_id = 3  -> PID + compensatore
HINF_controller_id = 4  -> mu-synthesis
HINF_controller_id = 5  -> H2
```

Per verificare l'integrazione:

```matlab
verify_H2_integration
```

Se si sostituisce il modello con una versione precedente priva del blocco H2, usare:

```matlab
install_H2_controller_in_simulink
```

## Validazione del controllore H2

Il controllore H2 viene incluso automaticamente in:

- `H2_analysis.m`: norma H2, metriche nominali e confronto con le specifiche H-infinity;
- `HINF_analysis.m`: NS, NP, RS, RP e analisi robusta comune;
- `MU_analysis_comparison.m`: confronto mu con mixsyn, hinfsyn e mu-synthesis;
- `Monte_Carlo_robustness.m`: 10 campioni incerti, tracking, banda, accoppiamento e disturbi;
- `RAS_vectorized.m`: stima numerica della regione di attrazione/convergenza;
- `run_simulink_simulations.m`: nominale non lineare, stress test incerto + disturbo e validazione del bordo RAS.

## Output

- `result_plot/`: figure MATLAB a 300 dpi;
- `analysis_result/`: CSV prodotti dagli script;
- `simulink_result/`: risultati e confronti della campagna Simulink;
- `baseline_results_pre_H2/`: risultati della precedente versione senza H2, conservati solo come riferimento storico.

Le cartelle di output principali sono inizialmente vuote per evitare di mescolare risultati della baseline con quelli del nuovo controllore. Eseguire `run_project` per rigenerarle.

## Nota sulla RAS

La mappa prodotta da `RAS_vectorized.m` e' una **stima numerica a orizzonte finito**, non una prova analitica della regione di attrazione asintotica. La classificazione dipende da `tFinal`, dalle tolleranze e dai limiti globali. Le prove Simulink a 40 s servono a distinguere punti realmente divergenti da punti che convergono piu' lentamente del criterio a 15 s.

## Nota sui risultati H2

Questo archivio contiene il codice completo per sintetizzare e validare H2, ma non inventa risultati numerici senza eseguire MATLAB/Robust Control Toolbox. I risultati H2 vengono generati localmente al primo `run_project`.
