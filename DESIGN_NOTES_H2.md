# Nota di progetto — controllore H2

## Perche' non usare direttamente il peso WS H-infinity

Nel problema continuo H2 la funzione di trasferimento chiusa dagli ingressi esogeni alle uscite di prestazione deve avere norma H2 finita. Il peso `WS` usato per H-infinity ha guadagno non nullo ad alta frequenza, mentre `S(jw) -> I`: il termine `WS*S` avrebbe quindi feedthrough/non decadrebbe a zero e la sua norma H2 sarebbe infinita.

Per questo `H2_synthesis.m` usa:

```text
WS_H2 = WS * diag(wr_alpha/(s+wr_alpha), wr_beta/(s+wr_beta))
```

con frequenze di roll-off pari a dieci volte le bande di tracking impostate in `HINF_setup.m`.

L'aggiunta non cambia significativamente la richiesta a bassa/media frequenza, ma rende il canale di prestazione strettamente proprio. `WU` e `WT` rimangono gli stessi della baseline.

## Obiettivo

```text
min_K || [WS_H2*S ; WU*K*S ; WT*T] ||_2
```

H2 minimizza una misura energetica/RMS nominale. Non minimizza il peggior caso e non certifica robustezza. Per questo `K_h2` viene poi giudicato usando anche i pesi H-infinity originali `WS/WU/WT`, l'analisi mu, Monte Carlo, RAS e Simulink.

## Interpretazione corretta

Il confronto con H-infinity e mu-synthesis e' intenzionalmente complementare:

- H2: ottimo nominale energetico;
- H-infinity: minimizzazione worst-case della norma pesata;
- mu-synthesis: robust performance rispetto alla struttura d'incertezza modellata.

Una buona prestazione H2 non implica automaticamente `gamma_Hinf < 1` ne' `mu_RP < 1`.
