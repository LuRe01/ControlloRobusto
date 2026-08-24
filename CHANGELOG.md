# Changelog — versione finale

- Normalizzati i nomi dei file e mantenuto `run_project.m` come unico orchestratore dell'esecuzione completa.
- Corretto il mapping della tabella `results` di `HINF_analysis.m` tra metriche e frequenze di picco.
- Consolidato il tuning H-infinity definitivo: Ms = 1.45/1.50, As = 0.015/0.015, wb = 3.8/3.0 rad/s, WU = 0.90 I, wt = 22/18 rad/s.
- Esteso il Monte Carlo a 30 s e aggiunti i flag `PitchSettledWithinHorizon` e `YawSettledWithinHorizon` per distinguere un transitorio lento da un dato non valutabile.
- Chiarita la natura della RAS come stima numerica a orizzonte finito; aggiunta la metrica `SlowConvergingPoints` e corretta la terminologia grafica in convergente/fuori criterio a T_f.
- Estese a 40 s le verifiche Simulink dei punti adiacenti al bordo della mappa RAS e corretta la descrizione interno/esterno al criterio numerico.
- Le prove nominali H-infinity della campagna Simulink usano esplicitamente il plant nominale; gli stress test incerti restano separati.
- I warning MATLAB/Simulink vengono silenziati solo durante `sim()` e ripristinati subito dopo; gli errori restano visibili.
- Definito `MU_analysis_comparison.m` / `MU_vs_HINF_results.csv` come riferimento ufficiale per i valori mu comparativi; la mu esplicita in `HINF_analysis.m` resta diagnostica ausiliaria.
- Rimossa dal pacchetto la cache compilata `helicopter_2DOF.slxc`, rigenerabile localmente.
- Aggiornati README e commenti di progetto senza modificare i controllori salvati o i risultati numerici gia' prodotti.
