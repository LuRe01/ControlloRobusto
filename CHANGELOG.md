# CHANGELOG

## Versione H2 — 26 agosto 2026

- aggiunto `H2_synthesis.m` con sintesi H2 pura tramite `h2syn`;
- introdotto `WS_H2`, ottenuto dal peso di sensibilita' H-infinity con roll-off strettamente proprio;
- aggiunto `H2_analysis.m` con norma H2, metriche nominali e confronto H-infinity;
- esteso `HINF_analysis.m` al controllore H2;
- esteso `MU_analysis_comparison.m` al controllore H2;
- esteso `Monte_Carlo_robustness.m` al controllore H2;
- esteso `RAS_vectorized.m` al controllore H2;
- integrato `K_h2` direttamente nel modello `helicopter_2DOF.slx`;
- aggiunto `HINF_controller_id = 5` per selezionare H2;
- estesa la campagna Simulink con tracking nominale, stress test incerto e validazione RAS per H2;
- aggiunti `install_H2_controller_in_simulink.m` e `verify_H2_integration.m`;
- aggiornato `run_project.m` all'ordine di esecuzione a 12 fasi prima della campagna Simulink;
- rimossa la cache `helicopter_2DOF.slxc` dalla distribuzione;
- spostati i vecchi risultati in `baseline_results_pre_H2/` per evitare commistione con i nuovi output;
- aggiornati README, commenti del tuning H-infinity e nomenclatura della RAS.

## Baseline precedente

- mixed sensitivity H-infinity definitiva con `gamma < 1`;
- robust stability certificata;
- robust performance non certificata;
- mu-synthesis, Monte Carlo, RAS numerica e campagna Simulink non lineare.

## 2026-08-26 - Uniformazione grafica finale

- Aggiunta `controller_plot_color.m` con palette globale controller-specifica.
- Lo stesso controllore mantiene ora lo stesso colore in HINF analysis, RAS e confronti Simulink.
- Corretto il plot dello sforzo di controllo in `HINF_analysis.m`: i due attuatori sono ora in pannelli separati.
- Corretti i grafici dei valori singolari `S`, `KS`, `T`: le due curve dello stesso controllore condividono il colore e usano stili di linea differenti, evitando il riciclo della `ColorOrder` e legende ambigue.
