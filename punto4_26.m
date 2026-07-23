%% ========================================================================
%  PROGETTO MPRAI - GRUPPO F2
%% 
%  PUNTO 4: Isolamento del Guasto (FDI - Fault Detection & Isolation)
%  Obiettivo: Generare residui r1 e r2 per capire QUALE ruota è rotta.
%  Include: Derivazione formale Phi, analisi robustezza, falsi pos/neg,
%           tempo di isolamento.
%% ========================================================================

clear all; close all; clc;

%% 1. SCELTA SCENARIO DI TEST PRINCIPALE
% 1 = Guasto Ruota DESTRA (FD1) -> Ci aspettiamo che salga solo r1
% 2 = Guasto Ruota SINISTRA (FS1) -> Ci aspettiamo che salga solo r2
SCENARIO = 1; 

%% 2. PARAMETRI SISTEMA
r1_param = 0.1; r2_param = 0.1; l = 0.4;

% Matrici cinematiche
g1 = @(theta) [(1/2)*r1_param*cos(theta); (1/2)*r1_param*sin(theta); (1/l)*r1_param];
g2 = @(theta) [(1/2)*r2_param*cos(theta); (1/2)*r2_param*sin(theta); -(1/l)*r2_param];
f_sys = @(t, z, u, w) g1(z(3))*u(1)*w(1) + g2(z(3))*u(2)*w(2);

%% 3. DERIVAZIONE FORMALE DELLE TRASFORMAZIONI Phi1 E Phi2
% -----------------------------------------------------------------------
% Dal PDF (Sezione 4.0.1), cerchiamo Phi1, Phi2 : R^3 -> R^2 tali che:
%   (dPhi1/dz) * g2(z) = 0    (Phi1 insensibile all'ingresso u2)
%   (dPhi2/dz) * g1(z) = 0    (Phi2 insensibile all'ingresso u1)
%
% === CALCOLO DI Phi1 ===
%
% Passo 1: Calcolo ker(g2(z)^T)
%   g2(z)^T = [r2/2*cos(th), r2/2*sin(th), -r2/l]
%   Cerchiamo v = [v1;v2;v3] t.c. g2^T * v = 0:
%     r2/2*cos(th)*v1 + r2/2*sin(th)*v2 - r2/l*v3 = 0
%     => cos(th)*v1/2 + sin(th)*v2/2 = v3/l         ... (*)
%
% Passo 2: Due soluzioni indipendenti
%   Si verifica per sostituzione in (*) che:
%     v_a = [1, 0, (l/2)*cos(th)]    ->  cos(th)/2 = (l/2)*cos(th)/l  ok
%     v_b = [0, 1, (l/2)*sin(th)]    ->  sin(th)/2 = (l/2)*sin(th)/l  ok
%
% Passo 3: Verifica condizione di esattezza (Eq. 36 del PDF)
%   Per v_a(z)^T = [1, 0, (l/2)*cos(th)]:
%     dv_a1/dy = 0 = dv_a2/dx    ok (entrambi 0)
%     dv_a1/dth = 0 = dv_a3/dx   ok (entrambi 0)
%     dv_a2/dth = 0 = dv_a3/dy   ok (entrambi 0)
%   => v_a e' forma differenziale esatta, primitiva: Phi11 = x + (l/2)*sin(th)
%
%   Per v_b(z)^T = [0, 1, (l/2)*sin(th)]:
%     dv_b1/dy = 0 = dv_b2/dx    ok
%     dv_b1/dth = 0 = dv_b3/dx   ok
%     dv_b2/dth = 0 = dv_b3/dy   ok
%   => v_b e' forma differenziale esatta, primitiva: Phi12 = y - (l/2)*cos(th)
%
% Passo 4: Trasformazione risultante
%   Phi1(z) = [x + (l/2)*sin(th) ;  y - (l/2)*cos(th)]
%           = Posizione della RUOTA DESTRA nel piano
%
% Passo 5: Verifica disaccoppiamento
%   dPhi1/dz = [[1, 0, (l/2)*cos(th)]; [0, 1, (l/2)*sin(th)]]
%
%   dPhi1/dz * g2 = [r2/2*cos + (l/2)*cos*(-r2/l) ; ...]
%                  = [r2/2*cos - r2/2*cos ; r2/2*sin - r2/2*sin]
%                  = [0 ; 0]    -> DISACCOPPIATO DA u2
%
%   dPhi1/dz * g1 = [r1/2*cos + (l/2)*cos*(r1/l) ; ...]
%                  = [r1*cos(th) ; r1*sin(th)]    -> NON NULLO
%
% === CALCOLO DI Phi2 (per simmetria) ===
%   Phi2(z) = [x - (l/2)*sin(th) ;  y + (l/2)*cos(th)]
%           = Posizione della RUOTA SINISTRA nel piano
%   dPhi2/dz * g1 = [0 ; 0]     -> DISACCOPPIATO DA u1
%   dPhi2/dz * g2 = [r2*cos(th) ; r2*sin(th)]  -> NON NULLO
%
% CONCLUSIONE:
%   r1 = ||Phi1(z) - Phi1_hat|| e' sensibile SOLO a w1 (guasto ruota DX)
%   r2 = ||Phi2(z) - Phi2_hat|| e' sensibile SOLO a w2 (guasto ruota SX)
% -----------------------------------------------------------------------

%% 4. PARAMETRI SIMULAZIONE & FDI
T_tot = 40; T_fault = 20; dt = 0.01;
time = 0:dt:T_tot; N = length(time);

% Matrice di Guadagno dell'Osservatore (H)
% -H deve essere Hurwitz: scegliamo autovalori -5 (convergenza rapida)
H = diag([-5, -5]);

% Soglia di rilevamento (politica a soglia)
THRESHOLD = 0.05;

%% 5. ESECUZIONE SIMULAZIONE PRINCIPALE (Scenario scelto)
[r1, r2, z, ~] = run_fdi_simulation(SCENARIO, 0.6, ...
    r1_param, r2_param, l, g1, g2, f_sys, T_tot, T_fault, dt, H);

%% 6. VISUALIZZAZIONE SCENARIO SINGOLO
figure('Name', 'FDI Analysis - Scenario Singolo', 'Color', 'w', ...
       'Position', [100 100 1000 700]);

% Plot Residui
subplot(2,1,1);
plot(time, r1, 'b', 'LineWidth', 2); hold on;
plot(time, r2, 'r', 'LineWidth', 2);
xline(T_fault, 'k--', 'Inizio Guasto', 'LineWidth', 1.5);
yline(THRESHOLD, 'g--', 'Soglia', 'LineWidth', 1.5);
grid on;
ylabel('Ampiezza Residuo');
title(sprintf('Analisi Residui - Scenario %d', SCENARIO));
legend('r_1 (Monitor Ruota DX)', 'r_2 (Monitor Ruota SX)', ...
       'Location', 'northwest');

% Calcolo e visualizzazione tempo di isolamento
[t_iso, which_wheel] = compute_isolation_time(time, r1, r2, T_fault, THRESHOLD);
if t_iso > 0
    xline(t_iso, 'm-', sprintf('Isolamento: %.2fs', t_iso - T_fault), ...
          'LineWidth', 2, 'LabelOrientation', 'horizontal');
end

% Plot Traiettoria
subplot(2,1,2);
R = 5; w_ref = 0.3;
x_ref_t = @(t) 5 + R*cos(w_ref*t);
y_ref_t = @(t) 5 + R*sin(w_ref*t);
plot(x_ref_t(time), y_ref_t(time), 'k--', 'LineWidth', 1.5); hold on;
plot(z(1,:), z(2,:), 'b-', 'LineWidth', 2);
idx_fault = find(time >= T_fault, 1);
plot(z(1,idx_fault), z(2,idx_fault), 'rx', 'MarkerSize', 15, 'LineWidth', 3);
legend('Riferimento', 'Traiettoria Reale', 'Inizio Guasto');
title('Traiettoria con Guasto'); grid on; axis equal;

%% 7. OUTPUT DIAGNOSI SCENARIO SINGOLO
avg_r1_post = mean(r1(time > T_fault + 2));
avg_r2_post = mean(r2(time > T_fault + 2));

fprintf('\n=== DIAGNOSI SCENARIO %d ===\n', SCENARIO);
fprintf('  Media r1 post-guasto: %.4f\n', avg_r1_post);
fprintf('  Media r2 post-guasto: %.4f\n', avg_r2_post);
if t_iso > 0
    fprintf('  Tempo di isolamento:  %.3f s (dopo guasto)\n', t_iso - T_fault);
    fprintf('  Ruota identificata:   %s\n', which_wheel);
else
    fprintf('  ATTENZIONE: Nessun guasto rilevato (falso negativo)\n');
end

%% ========================================================================
%  8. ANALISI COMPLETA FDI: Robustezza, Falsi Positivi/Negativi
%% ========================================================================

fprintf('\n\n========================================================\n');
fprintf('  ANALISI COMPLETA FDI\n');
fprintf('========================================================\n');

% --- 8a. TEST FALSI POSITIVI (Scenario Nominale) ---
fprintf('\n--- 8a. ANALISI FALSI POSITIVI (Scenario Nominale) ---\n');

[r1_nom, r2_nom, ~, ~] = run_fdi_simulation(0, 1.0, ...
    r1_param, r2_param, l, g1, g2, f_sys, T_tot, T_fault, dt, H);

max_r1_nom = max(r1_nom);
max_r2_nom = max(r2_nom);
fp_r1 = any(r1_nom > THRESHOLD);
fp_r2 = any(r2_nom > THRESHOLD);

fprintf('  Residuo r1 max (nominale): %.6f  -> Falso Positivo: %s\n', ...
    max_r1_nom, string_bool(fp_r1));
fprintf('  Residuo r2 max (nominale): %.6f  -> Falso Positivo: %s\n', ...
    max_r2_nom, string_bool(fp_r2));
if ~fp_r1 && ~fp_r2
    fprintf('  Nessun falso positivo con soglia = %.3f\n', THRESHOLD);
else
    fprintf('  ATTENZIONE: Soglia troppo bassa, generati falsi positivi!\n');
end

% --- 8b. ANALISI FALSI NEGATIVI E ROBUSTEZZA ---
fprintf('\n--- 8b. ANALISI ROBUSTEZZA E FALSI NEGATIVI ---\n');

% Test con diverse severita' di guasto sulla ruota destra
w_test_values = [0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.0];
n_tests = length(w_test_values);

% Risultati
iso_times_dx = zeros(1, n_tests);
r1_peaks = zeros(1, n_tests);
r2_peaks = zeros(1, n_tests);
detected_dx = false(1, n_tests);

fprintf('\n  %-8s | %-10s | %-10s | %-12s | %-10s\n', ...
    'w_fault', 'r1_max', 'r2_max', 'T_iso [s]', 'Rilevato?');
fprintf('  %s\n', repmat('-', 1, 60));

for k = 1:n_tests
    w_val = w_test_values(k);
    
    % Test guasto ruota DESTRA (Scenario 1)
    [r1_t, r2_t, ~, ~] = run_fdi_simulation(1, w_val, ...
        r1_param, r2_param, l, g1, g2, f_sys, T_tot, T_fault, dt, H);
    
    r1_peaks(k) = max(r1_t(time > T_fault));
    r2_peaks(k) = max(r2_t(time > T_fault));
    
    [t_iso_k, ~] = compute_isolation_time(time, r1_t, r2_t, T_fault, THRESHOLD);
    if t_iso_k > 0
        iso_times_dx(k) = t_iso_k - T_fault;
        detected_dx(k) = true;
    end
    
    fprintf('  w=%.1f    | r1=%.4f  | r2=%.4f  | ', ...
        w_val, r1_peaks(k), r2_peaks(k));
    if detected_dx(k)
        fprintf('T=%.3f s    | SI\n', iso_times_dx(k));
    else
        fprintf('N/A          | NO (Falso Neg.)\n');
    end
end

% Test guasto ruota SINISTRA (verifica simmetria)
fprintf('\n  --- Test simmetria (guasto ruota SX, w2=0.6) ---\n');
[r1_sx, r2_sx, ~, ~] = run_fdi_simulation(2, 0.6, ...
    r1_param, r2_param, l, g1, g2, f_sys, T_tot, T_fault, dt, H);
[t_iso_sx, which_sx] = compute_isolation_time(time, r1_sx, r2_sx, T_fault, THRESHOLD);
fprintf('  r1_max=%.4f (atteso ~0), r2_max=%.4f (atteso >>0)\n', ...
    max(r1_sx(time > T_fault)), max(r2_sx(time > T_fault)));
if t_iso_sx > 0
    fprintf('  Isolamento: %s in %.3f s -> Simmetria verificata\n', ...
        which_sx, t_iso_sx - T_fault);
end

% --- 8c. CONTEGGIO FALSI NEGATIVI ---
n_false_neg = sum(~detected_dx & w_test_values < 1);
fprintf('\n--- 8c. RIEPILOGO FALSI NEGATIVI ---\n');
fprintf('  Guasti testati: %d\n', sum(w_test_values < 1));
fprintf('  Falsi negativi: %d\n', n_false_neg);
if n_false_neg > 0
    fprintf('  Guasti non rilevati per w = ');
    fprintf('%.1f ', w_test_values(~detected_dx & w_test_values < 1));
    fprintf('\n  -> Guasti lievi (w vicino a 1) possono non essere rilevati\n');
    fprintf('     se il residuo resta sotto la soglia %.3f\n', THRESHOLD);
else
    fprintf('  Tutti i guasti rilevati correttamente\n');
end

%% 9. FIGURE DI ANALISI COMPLETA

% Figura 2: Robustezza - Ampiezza residui vs severita' guasto
figure('Name', 'FDI - Analisi Robustezza', 'Color', 'w', ...
       'Position', [150 100 1100 700]);

subplot(2,2,1);
bar(1-w_test_values, r1_peaks, 'FaceColor', [0.2 0.4 0.8]);
hold on;
yline(THRESHOLD, 'r--', 'Soglia', 'LineWidth', 2);
xlabel('Severita guasto (1-w)');
ylabel('Ampiezza picco r_1');
title('Robustezza: Residuo r_1 vs Severita (Ruota DX)');
grid on;

subplot(2,2,2);
detected_times = iso_times_dx;
detected_times(~detected_dx) = NaN;
bar(1-w_test_values, detected_times, 'FaceColor', [0.8 0.4 0.2]);
xlabel('Severita guasto (1-w)');
ylabel('Tempo di isolamento [s]');
title('Tempo di Isolamento vs Severita');
grid on;

% Confronto residui per diversi w
subplot(2,2,[3 4]);
colors = parula(n_tests);
legend_entries = {};
for k = 1:n_tests
    if w_test_values(k) < 1
        [r1_plot, ~, ~, ~] = run_fdi_simulation(1, w_test_values(k), ...
            r1_param, r2_param, l, g1, g2, f_sys, T_tot, T_fault, dt, H);
        plot(time, r1_plot, 'Color', colors(k,:), 'LineWidth', 1.5); hold on;
        legend_entries{end+1} = sprintf('w=%.1f', w_test_values(k));
    end
end
xline(T_fault, 'k--', 'LineWidth', 1.5);
yline(THRESHOLD, 'r--', 'Soglia', 'LineWidth', 2);
xlabel('Tempo [s]'); ylabel('Residuo r_1');
title('Evoluzione Residuo r_1 per Diverse Severita di Guasto');
legend(legend_entries, 'Location', 'northwest');
grid on;

sgtitle('PUNTO 4: Analisi Completa FDI', 'FontSize', 14, 'FontWeight', 'bold');

% Figura 3: Falsi Positivi e Disaccoppiamento
figure('Name', 'FDI - Falsi Positivi', 'Color', 'w', ...
       'Position', [200 100 900 400]);
subplot(1,2,1);
plot(time, r1_nom, 'b', 'LineWidth', 1.5); hold on;
plot(time, r2_nom, 'r', 'LineWidth', 1.5);
yline(THRESHOLD, 'g--', 'Soglia', 'LineWidth', 2);
xlabel('Tempo [s]'); ylabel('Residuo');
title('Residui in Condizioni Nominali (w_1=w_2=1)');
legend('r_1', 'r_2', 'Soglia');
grid on;

subplot(1,2,2);
plot(time, r1, 'b', 'LineWidth', 1.5); hold on;
plot(time, r2, 'r', 'LineWidth', 1.5);
yline(THRESHOLD, 'g--', 'Soglia', 'LineWidth', 2);
xline(T_fault, 'k--');
xlabel('Tempo [s]'); ylabel('Residuo');
title(sprintf('Disaccoppiamento (Scenario %d)', SCENARIO));
legend('r_1 (Ruota DX)', 'r_2 (Ruota SX)');
grid on;

sgtitle('Verifica Assenza Falsi Positivi e Disaccoppiamento', ...
        'FontSize', 13, 'FontWeight', 'bold');

fprintf('\n========================================================\n');
fprintf('  ANALISI FDI COMPLETATA\n');
fprintf('========================================================\n\n');

%% ========================================================================
%  FUNZIONI LOCALI
%% ========================================================================

function [r1, r2, z, w_history] = run_fdi_simulation(scenario, w_fault_val, ...
    r1_p, r2_p, l, g1, g2, f_sys, T_tot, T_fault, dt, H)
% RUN_FDI_SIMULATION Esegue una singola simulazione FDI
%   scenario: 0=Nominale, 1=FD1(DX), 2=FS1(SX), 3=FD2(Blocco DX), 4=FS2(Blocco SX)
%   w_fault_val: valore di efficienza della ruota guasta (0..1)

    time = 0:dt:T_tot; N = length(time);
    
    % Traiettoria circolare
    R = 5; w_ref = 0.3;
    x_ref = @(t) 5 + R*cos(w_ref*t);  dx_ref = @(t) -R*w_ref*sin(w_ref*t);
    y_ref = @(t) 5 + R*sin(w_ref*t);  dy_ref = @(t) R*w_ref*cos(w_ref*t);
    theta_ref = @(t) atan2(dy_ref(t), dx_ref(t)); dtheta_ref = w_ref;
    
    % Inizializzazione
    z = zeros(3, N);
    z(:,1) = [x_ref(0); y_ref(0); theta_ref(0)];
    
    z_hat1 = zeros(2, N); % Stimatore Phi1 (ruota DX)
    z_hat2 = zeros(2, N); % Stimatore Phi2 (ruota SX)
    r1 = zeros(1, N);
    r2 = zeros(1, N);
    w_history = zeros(2, N);
    
    % Inizializzazione stimatori alla posizione corretta
    theta0 = z(3,1);
    z_hat1(:,1) = [z(1,1) + (l/2)*sin(theta0); z(2,1) - (l/2)*cos(theta0)];
    z_hat2(:,1) = [z(1,1) - (l/2)*sin(theta0); z(2,1) + (l/2)*cos(theta0)];
    
    % Parametri controllo
    kx=1; ky=1; kth=2; u_max=25; u_min=-25;
    
    for i = 1:N
        t = time(i);
        
        % Gestione Guasto
        if t >= T_fault
            switch scenario
                case 0, w = [1; 1];
                case 1, w = [w_fault_val; 1];
                case 2, w = [1; w_fault_val];
                case 3, w = [0; 1];
                case 4, w = [1; 0];
            end
        else
            w = [1; 1];
        end
        w_history(:,i) = w;
        
        % Controllore (Nominale)
        z_curr = z(:,i);
        ref = [x_ref(t); y_ref(t); theta_ref(t)];
        dref = [dx_ref(t); dy_ref(t); dtheta_ref];
        
        ex=z_curr(1)-ref(1); ey=z_curr(2)-ref(2); 
        eth=wrapToPi(z_curr(3)-ref(3));
        vx = dref(1)-kx*ex; vy = dref(2)-ky*ey;
        v_des = vx*cos(z_curr(3)) + vy*sin(z_curr(3));
        om_des = dref(3)-kth*eth;
        u1 = (v_des + l*om_des/2)/r1_p;
        u2 = (v_des - l*om_des/2)/r2_p;
        u_curr = [max(min(u1, u_max), u_min); max(min(u2, u_max), u_min)];
        
        % Dinamica Reale
        if i < N
            dz = f_sys(t, z_curr, u_curr, w);
            z(:,i+1) = z(:,i) + dz*dt;
            z(3,i+1) = wrapToPi(z(3,i+1));
            
            % Osservatori FDI (Eq. 27, 28 del PDF)
            theta = z(3,i);
            
            % Phi1(z) = posizione ruota DX (derivazione formale in Sez. 3)
            z1_meas = [z(1,i) + (l/2)*sin(theta); z(2,i) - (l/2)*cos(theta)];
            % Phi2(z) = posizione ruota SX
            z2_meas = [z(1,i) - (l/2)*sin(theta); z(2,i) + (l/2)*cos(theta)];
            
            % dPhi1/dz * g1 = [r1*cos(theta); r1*sin(theta)]
            vec_dir = [cos(theta); sin(theta)];
            
            % Stimatore 1: dz_hat1 = (dPhi1/dz)*g1*u1 + H*(z_hat1 - z1_meas)
            dz_hat1 = (r1_p * vec_dir * u_curr(1)) + H * (z_hat1(:,i) - z1_meas);
            z_hat1(:,i+1) = z_hat1(:,i) + dz_hat1 * dt;
            
            % Stimatore 2: dz_hat2 = (dPhi2/dz)*g2*u2 + H*(z_hat2 - z2_meas)
            dz_hat2 = (r2_p * vec_dir * u_curr(2)) + H * (z_hat2(:,i) - z2_meas);
            z_hat2(:,i+1) = z_hat2(:,i) + dz_hat2 * dt;
            
            % Residui
            r1(i+1) = norm(z1_meas - z_hat1(:,i));
            r2(i+1) = norm(z2_meas - z_hat2(:,i));
        end
    end
end

function [t_iso, which_wheel] = compute_isolation_time(time, r1, r2, T_fault, threshold)
% COMPUTE_ISOLATION_TIME Calcola il tempo necessario per isolare il guasto.
% Richiede che il residuo superi la soglia per almeno 10 campioni consecutivi
% (persistenza) per evitare rilevamenti spuri.

    t_iso = -1;
    which_wheel = 'Nessuno';
    dt = time(2) - time(1);
    persistence = round(0.1 / dt);  % 0.1s di persistenza (10 campioni)
    
    idx_post_fault = find(time >= T_fault);
    
    % Cerca prima occorrenza persistente per r1
    t_iso_r1 = find_persistent_crossing(r1, idx_post_fault, threshold, persistence);
    t_iso_r2 = find_persistent_crossing(r2, idx_post_fault, threshold, persistence);
    
    % Determina quale residuo ha reagito prima
    if t_iso_r1 > 0 && (t_iso_r2 <= 0 || t_iso_r1 <= t_iso_r2)
        t_iso = time(t_iso_r1);
        which_wheel = 'Ruota DESTRA (r1)';
    elseif t_iso_r2 > 0
        t_iso = time(t_iso_r2);
        which_wheel = 'Ruota SINISTRA (r2)';
    end
end

function idx = find_persistent_crossing(signal, search_range, threshold, persistence)
% Trova il primo indice in search_range dove signal > threshold per
% almeno 'persistence' campioni consecutivi.
    idx = -1;
    count = 0;
    for k = 1:length(search_range)
        i = search_range(k);
        if signal(i) > threshold
            count = count + 1;
            if count >= persistence && idx < 0
                idx = search_range(k - persistence + 1);
                return;
            end
        else
            count = 0;
        end
    end
end

function s = string_bool(val)
    if val, s = 'SI'; else, s = 'NO'; end
end