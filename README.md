# 🤖 Controllo Tollerante ai Guasti (FTC) per Uniciclo a Guida Differenziale

**Università Politecnica delle Marche**  
*Corso di Laurea Magistrale in Ingegneria Informatica e dell'Automazione*  
**Progetto MPRAI - Gruppo F2** | Anno Accademico 2024-2025  
*Autore*: Erxhes Dedja  
*Docenti*: Prof. Alessandro Freddi, Prof. Alessandro Baldini  

---

## 📌 Descrizione Generale

Questo repository raccoglie i codici **MATLAB** e la relazione tecnica in **LaTeX** dedicati al controllo, alla diagnosi ed alla riconfigurazione di un robot uniciclo a guida differenziale in presenza di guasti agli attuatori.

L'architettura sviluppata copre l'intero ciclo di vita dell'ingegneria del controllo:
1. **Modellistica Cinematico-Dinamica**: Formulazione affine al controllo $\dot{z} = g_1(z)u_1 + g_2(z)u_2$.
2. **Controllo di Tracking (Feedback Linearization)**: Inseguimento di traiettorie tempo-varianti con gestione della saturazione dei motori.
3. **Simulazione e Valutazione Guasti**: Modellazione delle perdite di efficienza degli attuatori ($FD_1, FS_1, FD_2, FS_2$).
4. **Diagnosi dei Guasti (FDI - Fault Detection & Isolation)**: Banco di osservatori geometricamente disaccoppiati per l'isolamento del guasto senza falsi allarmi.
5. **Controllo Tollerante ai Guasti (FTC - Active Feedforward)**: Algoritmo di stima online dei parametri ($\hat{w}$) e riconfigurazione in tempo reale del comando di controllo.

---

## 📂 Struttura del Repository

```text
.
├── PUNTO1_26.m                   # Modello cinematico e test a ingressi costanti a tratti
├── PUNTO2_26.m                   # Controllore di tracking basato su Feedback Linearization
├── punto3_26.m                   # Simulazione dell'impatto dei guasti sul controllore nominale
├── punto4_26.m                   # Banco osservatori FDI (disaccoppiamento e analisi robustezza)
├── punto5_26.m                   # Controllo Tollerante ai Guasti (FTC) integrato con FDI
├── ManutenzioneProgetto Finale.pdf # Relazione tecnica completa redatta in LaTeX
├── F2_Progetto.pdf               # Specifiche originali della traccia di progetto
└── Spiegazione Dettagliata.txt  # Sintesi esplicativa delle fasi del progetto
```

---

## 🚀 Come Eseguire i Codici MATLAB

Aprire MATLAB nella cartella principale ed eseguire i file nell'ordine desiderato:

```matlab
>> PUNTO1_26   % Valida il modello cinematico affine su 6 fasi di movimento
>> PUNTO2_26   % Esegue il tracking circolare (R=5m) in condizioni nominali
>> punto3_26   % Simula il guasto sulla ruota destra (w1=0.6 a t=20s) senza compensazione
>> punto4_26   % Esegue il banco osservatori FDI ed analizza residui e robustezza
>> punto5_26   % Esegue il controllo tollerante ai guasti (FTC) con stima adattiva online
```

---

## 📊 Sintesi dei Risultati

| Fase del Progetto | Obiettivo Principale | Risultato Chiave |
| :--- | :--- | :--- |
| **Punto 1** | Validazione cinematica | Percorso 12.5m, $z(T) = [4.19, 5.10, 7.50]^T$ coerente con la fisica. |
| **Punto 2** | Tracking nominale | Errore di posizione $< 2\text{ cm}$ su cerchio di raggio $5\text{ m}$. |
| **Punto 3** | Analisi guasti | Evidenza del fallimento del controllore nominale (deriva a spirale). |
| **Punto 4** | Isolamento FDI | Isolamento univoco in $0.11\text{s}$, $0\%$ falsi positivi. |
| **Punto 5** | Tolleranza FTC | Stima $\hat{w}_1 = 0.599$ (reale $0.600$), mantenimento del tracking. |

---

## 📄 Documentazione

La relazione completa è disponibile nel file **`ManutenzioneProgetto Finale.pdf`**, contenente tutte le dimostrazioni matematiche, le tabelle comparative e le figure generate.
