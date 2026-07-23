%% ========================================================================
%  PROGETTO MPRAI - GRUPPO F2
%  Punto 1: Implementazione Modello Cinematico Uniciclo
%% ========================================================================

clear all; close all; clc;

%% PARAMETRI DEL SISTEMA
% Parametri geometrici (da Tabella 1)
r1 = 0.1;      % Raggio ruota destra [m]
r2 = 0.1;      % Raggio ruota sinistra [m]
l = 0.4;       % Lunghezza asse ruote [m]

%% CALCOLO DELLE FUNZIONI g1(z) e g2(z)
% Dal modello (8): dz/dt = g1(z)*u1*w1 + g2(z)*u2*w2
% 
% Dalla mappatura degli input (7):
% v1 = (1/2)*r1*w1*u1 + (1/2)*r2*w2*u2
% v2 = (1/l)*r1*w1*u1 - (1/l)*r2*w2*u2
%
% Dal modello cinematico (6):
% dx/dt = cos(theta)*v1
% dy/dt = sin(theta)*v1
% dtheta/dt = v2

% Funzioni vettoriali g1(z) e g2(z)
g1 = @(theta) [(1/2)*r1*cos(theta); 
               (1/2)*r1*sin(theta); 
               (1/l)*r1];

g2 = @(theta) [(1/2)*r2*cos(theta); 
               (1/2)*r2*sin(theta); 
               -(1/l)*r2];

%% DINAMICA DEL SISTEMA
% Stato: z = [x; y; theta]
% Ingressi: u = [u1; u2] (velocità angolari desiderate)
% Guasti: w = [w1; w2] (moltiplicatori velocità)

% Funzione dinamica del sistema
f_system = @(t, z, u, w) g1(z(3))*u(1)*w(1) + g2(z(3))*u(2)*w(2);

%% SIMULAZIONE CON INGRESSI COSTANTI A TRATTI

% Parametri di simulazione
T_total = 30;      % Tempo totale simulazione [s]
dt = 0.01;         % Passo di integrazione [s]
time = 0:dt:T_total;
N = length(time);

% Condizioni iniziali
z0 = [0; 0; 0];    % [x0; y0; theta0]

% Inizializzazione variabili
z = zeros(3, N);
z(:,1) = z0;
u = zeros(2, N);

% DEFINIZIONE INGRESSI COSTANTI A TRATTI
% Fase 1 (0-5s): Moto rettilineo
% Fase 2 (5-10s): Curva a destra
% Fase 3 (10-15s): Moto rettilineo
% Fase 4 (15-20s): Curva a sinistra
% Fase 5 (20-25s): Moto rettilineo
% Fase 6 (25-30s): Rotazione sul posto

for i = 1:N
    t = time(i);
    
    if t < 5
        % Fase 1: Avanti diritto (u1 = u2 = 5 rad/s)
        u(:,i) = [5; 5];
    elseif t < 10
        % Fase 2: Curva a destra (u1 > u2)
        u(:,i) = [6; 4];
    elseif t < 15
        % Fase 3: Avanti diritto
        u(:,i) = [5; 5];
    elseif t < 20
        % Fase 4: Curva a sinistra (u2 > u1)
        u(:,i) = [4; 6];
    elseif t < 25
        % Fase 5: Avanti diritto
        u(:,i) = [5; 5];
    else
        % Fase 6: Rotazione sul posto (u1 = -u2)
        u(:,i) = [3; -3];
    end
end

% Condizioni nominali (nessun guasto)
w1 = 1;
w2 = 1;
w = [w1; w2];

%% INTEGRAZIONE NUMERICA (Metodo di Eulero)
for i = 1:N-1
    % Calcolo derivata
    dz = f_system(time(i), z(:,i), u(:,i), w);
    
    % Integrazione
    z(:,i+1) = z(:,i) + dz * dt;
end

%% VISUALIZZAZIONE RISULTATI

% Figura 1: Traiettoria nel piano XY
figure('Name', 'Punto 1 - Test Modello Cinematico', 'Position', [100 100 1200 800]);

subplot(2,3,[1 4])
plot(z(1,:), z(2,:), 'b-', 'LineWidth', 2);
hold on;
plot(z(1,1), z(2,1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot(z(1,end), z(2,end), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

% Aggiungi frecce per mostrare l'orientamento ogni secondo
step = round(1/dt);
for i = 1:step*2:N
    theta_i = z(3,i);
    arrow_len = 0.3;
    quiver(z(1,i), z(2,i), arrow_len*cos(theta_i), arrow_len*sin(theta_i), ...
           'r', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
end

grid on;
xlabel('x [m]', 'FontSize', 12);
ylabel('y [m]', 'FontSize', 12);
title('Traiettoria nel Piano XY', 'FontSize', 14, 'FontWeight', 'bold');
legend('Traiettoria', 'Inizio', 'Fine', 'Orientamento', 'Location', 'best');
axis equal;

% Figura 2: Posizione x nel tempo
subplot(2,3,2)
plot(time, z(1,:), 'b-', 'LineWidth', 2);
grid on;
xlabel('Tempo [s]', 'FontSize', 11);
ylabel('x [m]', 'FontSize', 11);
title('Posizione x(t)', 'FontSize', 12, 'FontWeight', 'bold');

% Figura 3: Posizione y nel tempo
subplot(2,3,3)
plot(time, z(2,:), 'r-', 'LineWidth', 2);
grid on;
xlabel('Tempo [s]', 'FontSize', 11);
ylabel('y [m]', 'FontSize', 11);
title('Posizione y(t)', 'FontSize', 12, 'FontWeight', 'bold');

% Figura 4: Orientamento theta nel tempo
subplot(2,3,5)
plot(time, rad2deg(z(3,:)), 'g-', 'LineWidth', 2);
grid on;
xlabel('Tempo [s]', 'FontSize', 11);
ylabel('\theta [°]', 'FontSize', 11);
title('Orientamento \theta(t)', 'FontSize', 12, 'FontWeight', 'bold');

% Figura 5: Ingressi di controllo
subplot(2,3,6)
plot(time, u(1,:), 'b-', 'LineWidth', 2);
hold on;
plot(time, u(2,:), 'r--', 'LineWidth', 2);
grid on;
xlabel('Tempo [s]', 'FontSize', 11);
ylabel('Velocità angolare [rad/s]', 'FontSize', 11);
title('Ingressi di Controllo', 'FontSize', 12, 'FontWeight', 'bold');
legend('u_1 (ruota dx)', 'u_2 (ruota sx)', 'Location', 'best');

sgtitle('PUNTO 1: Test Modello Cinematico - Ingressi Costanti a Tratti', ...
        'FontSize', 16, 'FontWeight', 'bold');

%% STAMPA INFORMAZIONI
fprintf('\n========================================\n');
fprintf('  PUNTO 1: MODELLO CINEMATICO UNICICLO\n');
fprintf('========================================\n\n');

fprintf('PARAMETRI DEL SISTEMA:\n');
fprintf('  - Raggio ruota destra (r1): %.2f m\n', r1);
fprintf('  - Raggio ruota sinistra (r2): %.2f m\n', r2);
fprintf('  - Lunghezza asse (l): %.2f m\n', l);

fprintf('\nVARIABILI DI STATO:\n');
fprintf('  - z = [x; y; theta]\n');
fprintf('  - z(0) = [%.2f; %.2f; %.2f]\n', z0(1), z0(2), z0(3));
fprintf('  - z(T) = [%.2f; %.2f; %.2f rad = %.2f°]\n', ...
        z(1,end), z(2,end), z(3,end), rad2deg(z(3,end)));

fprintf('\nINGRESSI DI CONTROLLO:\n');
fprintf('  - u = [u1; u2] (velocità angolari desiderate)\n');
fprintf('  - Test con 6 fasi di ingressi costanti a tratti\n');

fprintf('\nCONDIZIONI DI FUNZIONAMENTO:\n');
fprintf('  - w1 = %.2f (nominale, nessun guasto ruota dx)\n', w1);
fprintf('  - w2 = %.2f (nominale, nessun guasto ruota sx)\n', w2);

fprintf('\nDISTANZA PERCORSA:\n');
fprintf('  - Distanza totale: %.2f m\n', sum(sqrt(diff(z(1,:)).^2 + diff(z(2,:)).^2)));

fprintf('\n========================================\n');
fprintf('Test completato con successo!\n');
fprintf('========================================\n\n');

%% SALVATAGGIO DATI
save('unicycle_test_data.mat', 'z', 'u', 'time', 'r1', 'r2', 'l', 'w');
fprintf('Dati salvati in: unicycle_test_data.mat\n\n');