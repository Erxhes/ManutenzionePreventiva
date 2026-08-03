%% ========================================================================
%  PROGETTO MPRAI - GRUPPO F2
%  PUNTO 3: Simulazione dei Guasti (FD1, FS1, FD2, FS2)
%  Descrizione: Test del controllore nominale in presenza di guasti attivi
%% ========================================================================

clear all; close all; clc;

%% 1. SELEZIONE DELLO SCENARIO DI GUASTO
% Scegli qui quale guasto simulare
% 0 = Nessun Guasto (Nominale)
% 1 = FD1 (Rallentamento Ruota Destra)
% 2 = FS1 (Rallentamento Ruota Sinistra)
% 3 = FD2 (Blocco Ruota Destra - Guasto Totale)
% 4 = FS2 (Blocco Ruota Sinistra - Guasto Totale)

SCENARIO = 1;  % <--- CAMBIA QUESTO NUMERO PER TESTARE DIVERSI GUASTI

%% 2. PARAMETRI DEL SISTEMA
r1 = 0.1;      % Raggio ruota destra [m]
r2 = 0.1;      % Raggio ruota sinistra [m]
l = 0.4;       % Lunghezza asse ruote [m]

% Matrici del modello cinematico (g1 e g2)
g1 = @(theta) [(1/2)*r1*cos(theta); (1/2)*r1*sin(theta); (1/l)*r1];
g2 = @(theta) [(1/2)*r2*cos(theta); (1/2)*r2*sin(theta); -(1/l)*r2];

% Dinamica reale del sistema (che include il parametro di guasto w)
f_system = @(t, z, u, w) g1(z(3))*u(1)*w(1) + g2(z(3))*u(2)*w(2);

%% 3. PARAMETRI DI SIMULAZIONE
T_total = 40;       % Durata simulazione [s]
T_fault = 20;       % Istante in cui avviene il guasto [s]
dt = 0.01;          
time = 0:dt:T_total;
N = length(time);

% Traiettoria Circolare (Riferimento)
R_circle = 5;
omega_ref = 0.3;
x_center = 5; y_center = 5;

x_ref = @(t) x_center + R_circle * cos(omega_ref * t);
y_ref = @(t) y_center + R_circle * sin(omega_ref * t);
dx_ref = @(t) -R_circle * omega_ref * sin(omega_ref * t);
dy_ref = @(t) R_circle * omega_ref * cos(omega_ref * t);
theta_ref = @(t) atan2(dy_ref(t), dx_ref(t));
dtheta_ref = omega_ref;

%% 4. PARAMETRI CONTROLLORE
kx = 1.0; ky = 1.0; k_theta = 2.0;
u_max = 25; u_min = -25;  % Saturazione aumentata per robustezza

%% 5. INIZIALIZZAZIONE
z = zeros(3, N);
z(:,1) = [x_center + R_circle; y_center; pi/2]; % Stato iniziale corretto
u = zeros(2, N);
w_history = zeros(2, N); % Per tracciare il guasto nei grafici
errors = zeros(3, N);
w = [1; 1]; % Fix: inizializzazione esplicita (nominale) prima del loop

%% 6. LOOP DI SIMULAZIONE
fprintf('Simulazione scenario %d in corso...\n', SCENARIO);

for i = 1:N
    t = time(i);
    
    % --- GESTIONE GUASTI (Tabella 2 PDF) ---
    if t >= T_fault
        switch SCENARIO
            case 0 % Nominale
                w = [1; 1];
            case 1 % FD1: Rallentamento DX (w1 in 0..1)
                w = [0.6; 1]; 
            case 2 % FS1: Rallentamento SX (w2 in 0..1)
                w = [1; 0.6];
            case 3 % FD2: Blocco DX (w1 = 0)
                w = [0.0; 1];
            case 4 % FS2: Blocco SX (w2 = 0)
                w = [1; 0.0];
            otherwise % Fix: scenario non valido
                error('SCENARIO %d non riconosciuto. Scegliere un valore tra 0 e 4.', SCENARIO);
        end
    else
        w = [1; 1]; % Funzionamento perfetto prima di T_fault
    end
    w_history(:,i) = w;
    
    % --- CONTROLLORE (Non sa del guasto!) ---
    curr_z_ref = [x_ref(t); y_ref(t); theta_ref(t)];
    curr_dz_ref = [dx_ref(t); dy_ref(t); dtheta_ref];
    
    % Calcolo errore (solo per grafici)
    errors(1:2,i) = z(1:2,i) - curr_z_ref(1:2);
    errors(3,i) = wrapToPi(z(3,i) - curr_z_ref(3));
    
    % Legge di controllo
    u(:,i) = controller_tracking(z(:,i), curr_z_ref, curr_dz_ref, ...
                                 kx, ky, k_theta, r1, r2, l, u_min, u_max);
                             
    % --- DINAMICA REALE ---
    if i < N
        % Qui applichiamo w: il motore esegue u * w invece di u
        dz = f_system(t, z(:,i), u(:,i), w);
        z(:,i+1) = z(:,i) + dz * dt;
        z(3,i+1) = wrapToPi(z(3,i+1));
    end
end

%% 7. VISUALIZZAZIONE RISULTATI
figure('Name', 'Analisi Guasto', 'Position', [100 100 1200 800]);

% Traiettoria XY
subplot(2,2,[1 3]);
plot(x_ref(time), y_ref(time), 'k--', 'LineWidth', 1.5); hold on;
plot(z(1,:), z(2,:), 'b-', 'LineWidth', 2);
% Evidenzia il momento del guasto
idx_fault = find(time >= T_fault, 1);
if ~isempty(idx_fault) && SCENARIO > 0
    plot(z(1,idx_fault), z(2,idx_fault), 'rx', 'MarkerSize', 15, 'LineWidth', 3);
    text(z(1,idx_fault)+0.5, z(2,idx_fault), 'GUASTO', 'Color', 'r', 'FontWeight', 'bold');
end
legend('Riferimento', 'Traiettoria Reale', 'Inizio Guasto');
title(sprintf('Scenario %d: Effetto del guasto sulla traiettoria', SCENARIO));
grid on; axis equal;

% Andamento dei parametri w (Guasti)
subplot(2,2,2);
plot(time, w_history(1,:), 'b-', 'LineWidth', 2); hold on;
plot(time, w_history(2,:), 'r-', 'LineWidth', 2);
ylim([-0.1 1.2]);
legend('w_1 (Ruota DX)', 'w_2 (Ruota SX)');
title('Parametri di Efficienza (w)');
grid on; xlabel('Tempo [s]');

% Errori
subplot(2,2,4);
plot(time, errors(1,:), 'r'); hold on;
plot(time, errors(2,:), 'b');
xline(T_fault, 'k--');
legend('Err X', 'Err Y', 'Start Guasto');
title('Errore di Posizione');
grid on; xlabel('Tempo [s]'); ylabel('Metri');

%% 8. STAMPA INFORMAZIONI SU TERMINALE
err_track = sqrt(errors(1,:).^2 + errors(2,:).^2);
err_pre = mean(err_track(time < T_fault));
err_post_max = max(err_track(time >= T_fault));

fprintf('\n========================================================\n');
fprintf('  PUNTO 3: SIMULAZIONE E ANALISI GUASTI\n');
fprintf('========================================================\n');
fprintf('  Scenario Selezionato: %d\n', SCENARIO);
switch SCENARIO
    case 0, fprintf('  Descrizione: Nominale (Nessun guasto)\n');
    case 1, fprintf('  Descrizione: FD1 - Rallentamento Ruota Destra (w1=%.2f)\n', w_history(1,end));
    case 2, fprintf('  Descrizione: FS1 - Rallentamento Ruota Sinistra (w2=%.2f)\n', w_history(2,end));
    case 3, fprintf('  Descrizione: FD2 - Blocco Ruota Destra (w1=0)\n');
    case 4, fprintf('  Descrizione: FS2 - Blocco Ruota Sinistra (w2=0)\n');
end
fprintf('  Istante Iniezione Guasto: %.2f s\n', T_fault);
fprintf('--------------------------------------------------------\n');
fprintf('  Errore Medio Pre-Guasto (0-%.0fs):  %.4f m\n', T_fault, err_pre);
fprintf('  Errore Massimo Post-Guasto:        %.4f m\n', err_post_max);
fprintf('--------------------------------------------------------\n');
if SCENARIO > 0
    fprintf('  ESITO: Il controllore nominale NON riesce a compensare!\n');
    fprintf('         Il guasto causa la deriva del robot dalla traiettoria.\n');
else
    fprintf('  ESITO: Funzionamento nominale regolare.\n');
end
fprintf('========================================================\n\n');

% Funzione Controllore (identica al Punto 2)
function u = controller_tracking(z, z_ref, dz_ref, kx, ky, k_theta, r1, r2, l, u_min, u_max)
    ex = z(1) - z_ref(1);
    ey = z(2) - z_ref(2);
    etheta = wrapToPi(z(3) - z_ref(3));
    
    vx_global = dz_ref(1) - kx * ex;
    vy_global = dz_ref(2) - ky * ey;
    
    v_des = vx_global * cos(z(3)) + vy_global * sin(z(3));
    omega_des = dz_ref(3) - k_theta * etheta;
    
    % Inversione cinematica (coerente con Punto 2)
    u1 = (v_des + l*omega_des/2) / r1;
    u2 = (v_des - l*omega_des/2) / r2;

    u = [max(min(u1, u_max), u_min); max(min(u2, u_max), u_min)];
end