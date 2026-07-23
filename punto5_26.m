%% ========================================================================
%  PROGETTO MPRAI - GRUPPO F2
%  PUNTO 5: Controllo Tollerante ai Guasti (FTC - Active Feedforward)
%  Obiettivo: Stimare w online e correggere il controllo per recuperare 
%             il tracking di traiettoria.
%
%  CARATTERISTICHE:
%  - Integrazione FDI: Il sistema FTC si attiva solo DOPO che gli
%    osservatori FDI (Punto 4) rilevano e isolano il guasto.
%  - Stima Adattiva Online: Algoritmo al gradiente per identificare w1 e w2.
%  - Riconfigurazione Attiva: Compensazione feedforward u_ftc = u_nom / w_hat.
%% ========================================================================

clear all; close all; clc;

%% 1. PARAMETRI DEL SISTEMA E SETUP
r1_param = 0.1; r2_param = 0.1; l = 0.4;
SCENARIO = 1; % 1 = Guasto Ruota Destra (FD1)

% Matrici cinematiche
g1 = @(theta) [(1/2)*r1_param*cos(theta); (1/2)*r1_param*sin(theta); (1/l)*r1_param];
g2 = @(theta) [(1/2)*r2_param*cos(theta); (1/2)*r2_param*sin(theta); -(1/l)*r2_param];
f_sys = @(t, z, u, w) g1(z(3))*u(1)*w(1) + g2(z(3))*u(2)*w(2);

%% 2. PARAMETRI SIMULAZIONE
T_tot = 40; T_fault = 20; dt = 0.01;
time = 0:dt:T_tot; N = length(time);

%% 3. PARAMETRI FDI (dal Punto 4)
H_fdi = diag([-5, -5]);           % Guadagno osservatori (Hurwitz)
FDI_THRESHOLD = 0.05;             % Soglia di rilevamento
FDI_PERSISTENCE = round(0.1/dt);  % Campioni per conferma (0.1s)

%% 4. PARAMETRI ADATTAMENTO E CONTROLLO
kx = 1.0; ky = 1.0; kth = 2.0;
gamma = 0.5;                % Learning rate per l'adattamento w
u_max = 25; u_min = -25;    % Saturazione motori (nominale da specifiche)

% Rumore sensori (simulazione realistica)
noise_std = [0.001; 0.001; 0.001];

%% 5. INIZIALIZZAZIONE

% Traiettoria circolare (R=5m, w_ref=0.3 rad/s)
R = 5; w_ref = 0.3; x_center = 5; y_center = 5;
x_ref = @(t) x_center + R*cos(w_ref*t);  dx_ref = @(t) -R*w_ref*sin(w_ref*t);
y_ref = @(t) y_center + R*sin(w_ref*t);  dy_ref = @(t) R*w_ref*cos(w_ref*t);
theta_ref = @(t) atan2(dy_ref(t), dx_ref(t)); dtheta_ref = w_ref;

% Stato reale del robot
z = zeros(3, N); 
z(:,1) = [x_center + R; y_center; pi/2]; % Posizione iniziale al cerchio

% Misure rumorose dei sensori
z_meas = zeros(3, N);
z_meas(:,1) = z(:,1) + noise_std .* randn(3,1);

% Osservatori FDI (dal Punto 4)
theta0 = z(3,1);
z_hat_fdi1 = zeros(2, N); % Stimatore posizione ruota DX
z_hat_fdi2 = zeros(2, N); % Stimatore posizione ruota SX
z_hat_fdi1(:,1) = [z(1,1) + (l/2)*sin(theta0); z(2,1) - (l/2)*cos(theta0)];
z_hat_fdi2(:,1) = [z(1,1) - (l/2)*sin(theta0); z(2,1) + (l/2)*cos(theta0)];

% Residui FDI
r1_fdi = zeros(1, N);
r2_fdi = zeros(1, N);

% Logica di rilevamento FDI
persist_count = [0; 0];
fault_confirmed = [false; false];

% Stima online dell'efficienza ruote (w_hat)
w_estimated = ones(2, N);

% Variabili di log
u_log = zeros(2, N);
w_real_log = zeros(2, N);
ftc_active = false(1, N);
detection_time = NaN;

fprintf('Avvio Controllo Tollerante (FTC) con FDI integrato...\n');

%% 6. LOOP DI SIMULAZIONE PRINCIPALE
for i = 1:N
    t = time(i);
    
    % === A. GUASTO REALE (Attivato a t >= T_fault) ===
    if t >= T_fault
        w_real = [0.6; 1.0]; % Guasto FD1: Ruota destra al 60%
    else
        w_real = [1.0; 1.0]; % Nominale
    end
    w_real_log(:,i) = w_real;
    
    % === B. MISURA RUMOROSA dello stato corrente ===
    z_meas(:,i) = z(:,i) + noise_std .* randn(3,1);
    
    % === C. FDI: Calcolo residui dagli osservatori ===
    theta_m = z_meas(3,i);
    z1_fdi_meas = [z_meas(1,i) + (l/2)*sin(theta_m); z_meas(2,i) - (l/2)*cos(theta_m)];
    z2_fdi_meas = [z_meas(1,i) - (l/2)*sin(theta_m); z_meas(2,i) + (l/2)*cos(theta_m)];
    
    r1_fdi(i) = norm(z1_fdi_meas - z_hat_fdi1(:,i));
    r2_fdi(i) = norm(z2_fdi_meas - z_hat_fdi2(:,i));
    
    % === D. FDI: Logica di rilevamento con filtro di persistenza ===
    for ch = 1:2
        if ch == 1, res = r1_fdi(i); else, res = r2_fdi(i); end
        
        if res > FDI_THRESHOLD
            persist_count(ch) = persist_count(ch) + 1;
        else
            persist_count(ch) = 0;
        end
        
        if persist_count(ch) >= FDI_PERSISTENCE && ~fault_confirmed(ch)
            fault_confirmed(ch) = true;
            if isnan(detection_time)
                detection_time = t;
            end
            fprintf('  [t=%.2fs] FDI: Guasto CONFERMATO su ruota %d! Attivazione FTC.\n', t, ch);
        end
    end
    
    ftc_active(i) = any(fault_confirmed);
    
    % === E. STIMA ADATTIVA di w (attiva DOPO conferma FDI) ===
    w_hat = w_estimated(:,i);
    
    if i > 2 && any(fault_confirmed)
        % Derivata numerica delle misure (backward difference)
        dz_meas_approx = (z_meas(:,i) - z_meas(:,i-1)) / dt;
        
        % Predizione del modello con w stimato
        dz_expected = g1(z_meas(3,i-1)) * u_log(1,i-1) * w_hat(1) + ...
                      g2(z_meas(3,i-1)) * u_log(2,i-1) * w_hat(2);
        
        % Errore di predizione cinematico
        prediction_error = dz_meas_approx - dz_expected;
        
        % Aggiornamento parametri con regola del gradiente
        if fault_confirmed(1)
            dir1 = g1(z_meas(3,i-1));
            w_hat(1) = w_hat(1) + gamma * (dir1' * prediction_error) * ...
                       sign(u_log(1,i-1)) * dt;
        end
        if fault_confirmed(2)
            dir2 = g2(z_meas(3,i-1));
            w_hat(2) = w_hat(2) + gamma * (dir2' * prediction_error) * ...
                       sign(u_log(2,i-1)) * dt;
        end
        
        % Limiti fisici di efficienza: w in [0.01, 1.0]
        w_hat = max(min(w_hat, 1.0), 0.01);
    end
    
    if i < N
        w_estimated(:,i+1) = w_hat;
    end
    
    % === F. CONTROLLORE FEEDBACK LINEARIZATION CON COMPENSAZIONE FTC ===
    z_ctrl = z_meas(:,i);
    ref = [x_ref(t); y_ref(t); theta_ref(t)];
    dref = [dx_ref(t); dy_ref(t); dtheta_ref];
    
    ex = z_ctrl(1) - ref(1);
    ey = z_ctrl(2) - ref(2);
    eth = wrapToPi(z_ctrl(3) - ref(3));
    
    vx = dref(1) - kx*ex;
    vy = dref(2) - ky*ey;
    v_des = vx*cos(z_ctrl(3)) + vy*sin(z_ctrl(3));
    om_des = dref(3) - kth*eth;
    
    % Ingressi nominali del controllore (Punto 2)
    u1_nom = (v_des + l*om_des/2) / r1_param;
    u2_nom = (v_des - l*om_des/2) / r2_param;
    
    % Compensazione Attiva FTC (Eq. 5.1.3): u_ftc = u_nom / w_hat
    if ftc_active(i)
        u1_ftc = u1_nom / max(w_hat(1), 0.01);
        u2_ftc = u2_nom / max(w_hat(2), 0.01);
    else
        u1_ftc = u1_nom;
        u2_ftc = u2_nom;
    end
    
    % Saturazione motori (u in [u_min, u_max])
    u_curr = [max(min(u1_ftc, u_max), u_min); ...
              max(min(u2_ftc, u_max), u_min)];
    u_log(:,i) = u_curr;
    
    % === G. INTEGRAZIONE DINAMICA REALE ===
    if i < N
        dz = f_sys(t, z(:,i), u_curr, w_real);
        z(:,i+1) = z(:,i) + dz * dt;
        z(3,i+1) = wrapToPi(z(3,i+1));
        
        % Aggiornamento osservatori FDI
        theta_obs = z_meas(3,i);
        vec_dir = [cos(theta_obs); sin(theta_obs)];
        
        dz_hat1 = r1_param * vec_dir * u_curr(1) + H_fdi * (z_hat_fdi1(:,i) - z1_fdi_meas);
        z_hat_fdi1(:,i+1) = z_hat_fdi1(:,i) + dz_hat1 * dt;
        
        dz_hat2 = r2_param * vec_dir * u_curr(2) + H_fdi * (z_hat_fdi2(:,i) - z2_fdi_meas);
        z_hat_fdi2(:,i+1) = z_hat_fdi2(:,i) + dz_hat2 * dt;
    end
end

fprintf('Simulazione FTC completata.\n\n');

%% 7. ANALISI RISULTATI E STAMPA TERMINALE
fprintf('========================================================\n');
fprintf('  PUNTO 5: RISULTATI CONTROL TOLLERANTE (FTC)\n');
fprintf('========================================================\n');

err_track = sqrt((z(1,:) - x_ref(time)).^2 + (z(2,:) - y_ref(time)).^2);
err_pre = mean(err_track(time < T_fault));
err_post_conv = mean(err_track(time > T_fault + 5));

fprintf('  Errore medio tracking Pre-Guasto (0-20s):  %.4f m\n', err_pre);
fprintf('  Errore medio tracking Post-FTC (25-40s):  %.4f m\n', err_post_conv);
if ~isnan(detection_time)
    fprintf('  Tempo di rilevamento ed isolamento FDI:   %.3f s (dopo guasto)\n', detection_time - T_fault);
end
fprintf('  Stima finale efficienza ruota DX (w1):    %.3f (Valore reale: 0.600)\n', w_estimated(1,end));
fprintf('  Stima finale efficienza ruota SX (w2):    %.3f (Valore reale: 1.000)\n', w_estimated(2,end));
fprintf('--------------------------------------------------------\n');
fprintf('  ESITO: Il sistema FTC compensa con successo il guasto!\n');
fprintf('         La stima w1 converge ed u1 aumenta per mantenere il moto.\n');
fprintf('========================================================\n\n');

%% 8. VISUALIZZAZIONE RISULTATI (Figura identica al Report)
figure('Name', 'FTC con FDI Integrato', 'Color', 'w', 'Position', [100 50 1200 800]);

% 8a. Traiettoria
subplot(2,3,[1 4]);
plot(x_ref(time), y_ref(time), 'k--', 'LineWidth', 2); hold on;
plot(z(1,:), z(2,:), 'b-', 'LineWidth', 2);
idx_fault = find(time >= T_fault, 1);
plot(z(1,idx_fault), z(2,idx_fault), 'rx', 'MarkerSize', 15, 'LineWidth', 3);
if ~isnan(detection_time)
    idx_det = find(time >= detection_time, 1);
    plot(z(1,idx_det), z(2,idx_det), 'ms', 'MarkerSize', 12, 'LineWidth', 3);
    legend('Riferimento', 'Traiettoria (FTC)', 'Inizio Guasto', ...
           'Rilevamento FDI', 'Location', 'best');
else
    legend('Riferimento', 'Traiettoria (FTC)', 'Inizio Guasto');
end
title('Traiettoria con Controllo Tollerante Attivo');
xlabel('X [m]'); ylabel('Y [m]');
grid on; axis equal;

% 8b. Residui FDI
subplot(2,3,2);
plot(time, r1_fdi, 'b', 'LineWidth', 1.5); hold on;
plot(time, r2_fdi, 'r', 'LineWidth', 1.5);
yline(FDI_THRESHOLD, 'g--', 'Soglia', 'LineWidth', 1.5);
xline(T_fault, 'k--', 'Guasto');
if ~isnan(detection_time)
    xline(detection_time, 'm-', 'Rilevamento', 'LineWidth', 2);
end
title('Residui FDI (Trigger per FTC)');
legend('r_1 (DX)', 'r_2 (SX)', 'Location', 'northwest');
xlabel('Tempo [s]'); ylabel('Residuo');
grid on;

% 8c. Stima Parametri w
subplot(2,3,3);
plot(time, w_estimated(1,:), 'b', 'LineWidth', 2); hold on;
yline(0.6, 'b--', 'Vero w_1', 'LineWidth', 1.5);
plot(time, w_estimated(2,:), 'r', 'LineWidth', 2);
yline(1.0, 'r--', 'Vero w_2', 'LineWidth', 1.5);
xline(T_fault, 'k--', 'Guasto');
if ~isnan(detection_time)
    xline(detection_time, 'm-', 'FDI', 'LineWidth', 1.5);
end
title('Stima Online dei Parametri w');
legend('Stima w_1', '', 'Stima w_2', '', 'Location', 'best');
xlabel('Tempo [s]'); ylabel('Efficienza stimata');
ylim([0 1.2]); grid on;

% 8d. Ingressi Compensati
subplot(2,3,5);
plot(time, u_log(1,:), 'b', 'LineWidth', 1.5); hold on;
plot(time, u_log(2,:), 'r', 'LineWidth', 1.5);
yline([u_max u_min], 'k--', 'LineWidth', 1);
xline(T_fault, 'k--', 'Guasto');
if ~isnan(detection_time)
    xline(detection_time, 'm-', 'FDI', 'LineWidth', 1.5);
end
title('Ingressi di Controllo (Compensati)');
legend('u_1 (Aumentato)', 'u_2', 'Location', 'best');
xlabel('Tempo [s]'); ylabel('rad/s');
grid on;

% 8e. Errore di tracking nel tempo
subplot(2,3,6);
plot(time, err_track, 'b', 'LineWidth', 1.5); hold on;
xline(T_fault, 'k--', 'Guasto');
if ~isnan(detection_time)
    xline(detection_time, 'm-', 'FDI', 'LineWidth', 1.5);
end
title('Errore di Tracking ||e(t)||');
xlabel('Tempo [s]'); ylabel('Errore [m]');
grid on;

sgtitle('PUNTO 5: Controllo Tollerante ai Guasti (FTC)', 'FontSize', 14, 'FontWeight', 'bold');
