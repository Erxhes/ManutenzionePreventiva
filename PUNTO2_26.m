%% ========================================================================
%  PROGETTO MPRAI - GRUPPO F2
%  Punto 2: Design e Implementazione Controllore per Tracking 
%  Modifica: Aumento saturazione per evitare windup e drift
%% ========================================================================

clear all; close all; clc;

%% 1. PARAMETRI DEL SISTEMA 
r1 = 0.1;      % Raggio ruota destra [m]
r2 = 0.1;      % Raggio ruota sinistra [m]
l = 0.4;       % Lunghezza asse ruote [m]

%% 2. FUNZIONI DEL MODELLO CINEATICO [Eq. 6-7 PDF]
% Matrici g1(z) e g2(z) per la dinamica: dot_z = g1*u1 + g2*u2
g1 = @(theta) [(1/2)*r1*cos(theta); 
               (1/2)*r1*sin(theta); 
               (1/l)*r1];

g2 = @(theta) [(1/2)*r2*cos(theta); 
               (1/2)*r2*sin(theta); 
               -(1/l)*r2];

% Dinamica del sistema completa (include i guasti w, qui w=1)
f_system = @(t, z, u, w) g1(z(3))*u(1)*w(1) + g2(z(3))*u(2)*w(2);

%% 3. PARAMETRI DI SIMULAZIONE
T_total = 40;      % Tempo totale simulazione [s]
dt = 0.01;         % Passo di integrazione [s]
time = 0:dt:T_total;
N = length(time);

%% 4. TRAIETTORIA DI RIFERIMENTO
% Cerchio ampio e veloce
R_circle = 5;              % Raggio [m]
omega_ref = 0.3;           % Velocità angolare traiettoria [rad/s]
x_center = 5;              
y_center = 5;

% Generazione traiettoria analitica
x_ref = @(t) x_center + R_circle * cos(omega_ref * t);
y_ref = @(t) y_center + R_circle * sin(omega_ref * t);

% Derivate prime (feedforward)
dx_ref = @(t) -R_circle * omega_ref * sin(omega_ref * t);
dy_ref = @(t) R_circle * omega_ref * cos(omega_ref * t);

% Orientamento desiderato (tangente alla curva)
theta_ref = @(t) atan2(dy_ref(t), dx_ref(t));
dtheta_ref = omega_ref; % Derivata dell'orientamento costante sul cerchio

%% 5. PARAMETRI DEL CONTROLLORE (TUNING OTTIMIZZATO)
% Guadagni (Feedback Linearization + PD)
kx = 1.0;          % Guadagno errore x (morbido)
ky = 1.0;          % Guadagno errore y (morbido)
k_theta = 2.0;     % Guadagno errore orientamento (reattivo)

% Limiti fisici degli attuatori (MODIFICA CHIAVE)
% Il nominale richiesto è 15 rad/s. Impostiamo 25 per avere margine.
u_max = 25;        
u_min = -25;       

%% 6. INIZIALIZZAZIONE
% Stato iniziale: Coerente con la traiettoria al tempo t=0
% t=0 -> x=10, y=5, vy=vel_positiva -> theta=pi/2
z0 = [x_center + R_circle; y_center; pi/2]; 

z = zeros(3, N);
z(:,1) = z0;
u = zeros(2, N);
z_ref_vec = zeros(3, N);
errors = zeros(3, N);

% Vettore Guasti (w=1 nominale)
w = [1; 1];

%% 7. LOOP DI SIMULAZIONE
fprintf('Avvio simulazione Tracking...\n');

for i = 1:N
    t = time(i);
    
    % a) Calcolo Riferimenti Istantanei
    curr_z_ref = [x_ref(t); y_ref(t); theta_ref(t)];
    curr_dz_ref = [dx_ref(t); dy_ref(t); dtheta_ref];
    z_ref_vec(:,i) = curr_z_ref;
    
    % b) Calcolo Errori (per plotting)
    errors(1:2,i) = z(1:2,i) - curr_z_ref(1:2);
    errors(3,i) = wrapToPi(z(3,i) - curr_z_ref(3));
    
    % c) Calcolo Legge di Controllo
    u(:,i) = controller_tracking(z(:,i), curr_z_ref, curr_dz_ref, ...
                                 kx, ky, k_theta, r1, r2, l, u_min, u_max);
    
    % d) Integrazione Numerica (Eulero in avanti)
    if i < N
        dz = f_system(t, z(:,i), u(:,i), w);
        z(:,i+1) = z(:,i) + dz * dt;
        % Normalizzazione angolo tra -pi e pi
        z(3,i+1) = wrapToPi(z(3,i+1)); 
    end
end

fprintf('Simulazione completata.\n');

%% 8. VISUALIZZAZIONE RISULTATI
figure('Name', 'Tracking Performance Corretto', 'Color', 'w', 'Position', [100 100 1200 800]);

% Plot A: Traiettoria XY
subplot(2,2,[1 3]);
plot(z_ref_vec(1,:), z_ref_vec(2,:), 'k--', 'LineWidth', 2); hold on;
plot(z(1,:), z(2,:), 'b-', 'LineWidth', 2);
% Disegna robot inizio/fine
plot_robot(z(:,1), r1, 'g'); 
plot_robot(z(:,end), r1, 'r');
legend('Riferimento', 'Robot', 'Start', 'End');
xlabel('X [m]'); ylabel('Y [m]');
title('Inseguimento Traiettoria (Piano XY)');
axis equal; grid on;

% Plot B: Errori
subplot(2,2,2);
plot(time, errors(1,:), 'r', 'LineWidth', 1); hold on;
plot(time, errors(2,:), 'b', 'LineWidth', 1);
plot(time, errors(3,:), 'g', 'LineWidth', 1);
legend('Err X', 'Err Y', 'Err Theta');
title('Andamento Errori');
xlabel('Tempo [s]'); ylabel('Errore [m] / [rad]');
grid on;

% Plot C: Ingressi di Controllo (Verifica Saturazione)
subplot(2,2,4);
plot(time, u(1,:), 'b', 'LineWidth', 1.5); hold on;
plot(time, u(2,:), 'r', 'LineWidth', 1.5);
yline([u_max u_min], 'k--');
yline([15 -15], 'g:', 'LineWidth', 2); % Il limite teorico nominale
legend('u_1 (Des)', 'u_2 (Sin)', 'Limiti Attuali', 'Richiesta Nominale');
title('Ingressi di Controllo (u)');
xlabel('Tempo [s]'); ylabel('rad/s');
ylim([u_min-5, u_max+5]);
grid on;

%% ========================================================================
%  FUNZIONI AUSILIARIE
%% ========================================================================

function u = controller_tracking(z, z_ref, dz_ref, kx, ky, k_theta, r1, r2, l, u_min, u_max)
    % Unpacking
    theta = z(3);
    
    % Errore nel frame globale
    ex = z(1) - z_ref(1);
    ey = z(2) - z_ref(2);
    etheta = wrapToPi(z(3) - z_ref(3));
    
    % Feedforward + Feedback (Output: Velocità Cartesiane Desiderate)
    % vx_des = dx_ref - k * errore
    vx_global = dz_ref(1) - kx * ex;
    vy_global = dz_ref(2) - ky * ey;
    
    % Rotazione nel frame del robot (v, omega)
    % v = v_x * cos(th) + v_y * sin(th)
    v_des = vx_global * cos(theta) + vy_global * sin(theta);
    omega_des = dz_ref(3) - k_theta * etheta;
    
    % Inversione Cinematica (da v,w a u1,u2)
    % u1 = (v + l*w/2) / r1
    % u2 = (v - l*w/2) / r2
    u1 = (v_des + l*omega_des/2) / r1;
    u2 = (v_des - l*omega_des/2) / r2;
    
    % Saturazione
    u = [max(min(u1, u_max), u_min); 
         max(min(u2, u_max), u_min)];
end

function plot_robot(z, r, col)
    % Semplice funzione per disegnare la posa del robot
    x = z(1); y = z(2); th = z(3);
    plot(x, y, 'o', 'MarkerSize', 10, 'MarkerFaceColor', col, 'Color', 'k');
    quiver(x, y, cos(th), sin(th), 1, 'Color', col, 'LineWidth', 2, 'MaxHeadSize', 0.5);
end