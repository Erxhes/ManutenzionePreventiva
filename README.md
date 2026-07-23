# 🤖 Controllo Tollerante ai Guasti (FTC) per Uniciclo a Guida Differenziale

Questo repository contiene l'implementazione in ambiente **MATLAB** e la documentazione in **LaTeX** per il controllo, la diagnosi ed la riconfigurazione di un robot uniciclo a guida differenziale in presenza di guasti agli attuatori.

L'architettura del progetto è suddivisa in **5 cartelle tematiche**, ciascuna corrispondente a un punto specifico della traccia di progetto:

1. **[Punto1_ModelloCinematico/](./Punto1_ModelloCinematico/)**: Modellistica affine al controllo $\dot{z} = g_1(z)u_1 + g_2(z)u_2$ e test a 6 fasi.
2. **[Punto2_ControlloreTracking/](./Punto2_ControlloreTracking/)**: Controllore *Feedback Linearization* per tracking circolare ($R=5\text{m}$) con saturazione attuatori.
3. **[Punto3_SimulazioneGuasti/](./Punto3_SimulazioneGuasti/)**: Modellazione delle perdite di efficienza degli attuatori ed analisi della deriva del controllore nominale.
4. **[Punto4_SistemaFDI/](./Punto4_SistemaFDI/)**: Banco di osservatori geometricamente disaccoppiati per l'isolamento del guasto ($0\%$ falsi positivi).
5. **[Punto5_ControlloTolleranteFTC/](./Punto5_ControlloTolleranteFTC/)**: Algoritmo di stima online dei parametri ($\hat{w}$) e riconfigurazione attiva feedforward.

---

## 📂 Struttura del Repository

```text
.
├── Punto1_ModelloCinematico/
│   └── PUNTO1_26.m                   # Script Modello Cinematico
├── Punto2_ControlloreTracking/
│   └── PUNTO2_26.m                   # Script Controllore Tracking (Feedback Lin.)
├── Punto3_SimulazioneGuasti/
│   ├── punto3_26.m                   # Script Simulazione Guasti
│   └── codicePunto3.png              # Risultati grafici guasto
├── Punto4_SistemaFDI/
│   ├── punto4_26.m                   # Script Banco Osservatori FDI
│   ├── punto4_1.fig ... punto4_3.fig  # Figure MATLAB
│   └── FDI*.png                      # Grafici dei residui e robustezza
├── Punto5_ControlloTolleranteFTC/
│   └── punto5_26.m                   # Script Controllo Tollerante ai Guasti (FTC)
├── Documentazione/
│   ├── F2_Progetto.pdf               # Traccia ufficiale del progetto
│   ├── ManutenzioneProgetto Finale.pdf # Relazione finale in LaTeX
│   └── Spiegazione Dettagliata.txt  # Descrizione estesa del progetto
├── README.md                         # Documentazione di questo repository
└── .gitignore                        # Esclusioni per file temporanei MATLAB
```

---

## 🚀 Come Eseguire i Codici MATLAB

Aprire MATLAB nella cartella del punto desiderato ed eseguire lo script corrispondente:

```matlab
>> run('Punto1_ModelloCinematico/PUNTO1_26.m')   % Validazione modello cinematico
>> run('Punto2_ControlloreTracking/PUNTO2_26.m') % Tracking circolare nominale
>> run('Punto3_SimulazioneGuasti/punto3_26.m')   % Simulazione effetto guasto (FD1)
>> run('Punto4_SistemaFDI/punto4_26.m')         % Diagnosi ed isolamento del guasto (FDI)
>> run('Punto5_ControlloTolleranteFTC/punto5_26.m') % Controllo Tollerante Riconfigurato (FTC)
```

---

## 📊 Sintesi dei Risultati

| Cartella | Obiettivo | Risultato |
| :--- | :--- | :--- |
| **Punto 1** | Validazione Modello | Percorso 12.5m, $z(T) = [4.19, 5.10, 7.50]^T$ coerente con la fisica. |
| **Punto 2** | Tracking Nominale | Errore di posizione $< 2\text{ cm}$ su cerchio di raggio $5\text{ m}$. |
| **Punto 3** | Analisi Guasti | Dimostrazione della deriva a spirale del controllore nominale. |
| **Punto 4** | Isolamento FDI | Isolamento in $0.11\text{s}$, $0\%$ falsi positivi, disaccoppiamento perfetto. |
| **Punto 5** | Riconfigurazione FTC | Stima $\hat{w}_1 = 0.599$ (reale $0.600$), ripristino del moto in autonomia. |

---

## 📄 Documentazione Ufficiale

La relazione completa redatta in LaTeX è disponibile all'interno della cartella **[`Documentazione/ManutenzioneProgetto Finale.pdf`](./Documentazione/ManutenzioneProgetto%20Finale.pdf)**.
