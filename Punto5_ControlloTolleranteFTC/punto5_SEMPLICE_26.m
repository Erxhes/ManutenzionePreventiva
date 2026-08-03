%% ========================================================================
%  PROGETTO MPRAI - GRUPPO F2
%  PUNTO 5 (versione semplificata): Controllo Tollerante ai Guasti (FTC)
%
%  La traccia richiede:
%  "Creare una azione di feedforward nel loop di controllo al fine di
%   ottenere un controllo tollerante ai guasti di tipo attivo."
%
%  Approccio implementato:
%  1. Si integra il banco di osservatori FDI (Punto 4) per rilevare il guasto
%  2. Quando il guasto e' confermato, si stima w con legge al gradiente:
%       dw_hat/dt = gamma * (g_i(z)' * epsilon) * u_i
%     dove epsilon = dz_meas - g1*u1*w_hat1 - g2*u2*w_hat2
%  3. Si applica la compensazione feedforward:
%       u_ftc = u_nom / w_hat
%% ========================================================================

clear all; close all; clc;

%% 1. PARAMETRI DEL SISTEMA (dalla traccia, Tabella 1)
r1 = 0.1; r2 = 0.1; l = 0.4;

% Vettori cinematici g1(z), g2(z) dell'uniciclo (Eq. 8 della traccia)
g1 = @(th) [(r1/2)*cos(th); (r1/2)*sin(th);  r1/l];
g2 = @(th) [(r2/2)*cos(th); (r2/2)*sin(th); -r2/l];

%% 2. PARAMETRI SIMULAZIONE
T_tot   = 60;    % Durata totale simulazione [s]
T_fault = 20;    % Istante di insorgenza del guasto [s]
dt      = 0.01;  % Passo di integrazione [s]
time    = 0:dt:T_tot;
N       = length(time);

% Scenario guasto: FD1 (rallentamento ruota destra, w1=0.6 in (0,1))
w_real = [0.6; 1.0];  % Valori reali (sconosciuti al controllore)

%% 3. TRAIETTORIA DI RIFERIMENTO (circolare, R=5m, omega=0.3 rad/s)
R_ref = 5;   w_ref = 0.3;
x_c   = 5;   y_c   = 5;
xr  = @(t)  x_c + R_ref*cos(w_ref*t);
yr  = @(t)  y_c + R_ref*sin(w_ref*t);
dxr = @(t) -R_ref*w_ref*sin(w_ref*t);
dyr = @(t)  R_ref*w_ref*cos(w_ref*t);
thr = @(t)  atan2(dyr(t), dxr(t));

%% 4. PARAMETRI CONTROLLORE DI TRACKING (Feedback Linearization, Punto 2)
kx  = 1.0;   % Guadagno errore x
ky  = 1.0;   % Guadagno errore y
kth = 2.0;   % Guadagno errore orientamento
u_max = 30;  % Limite motori [rad/s] (con margine per compensazione FTC)

%% 5. PARAMETRI FDI (dal Punto 4)
H_fdi         = diag([-5, -5]);   % Guadagno osservatori (matrice Hurwitz)
FDI_THRESHOLD = 0.05;              % Soglia rilevamento residuo
FDI_PERSIST   = round(0.1/dt);    % Campioni per conferma guasto (0.1s)

% Trasformazioni di disaccoppiamento Phi1, Phi2 (dal Punto 4, Eq. 21-22)
Phi1 = @(z) [z(1) + (l/2)*sin(z(3));   z(2) - (l/2)*cos(z(3))];
Phi2 = @(z) [z(1) - (l/2)*sin(z(3));   z(2) + (l/2)*cos(z(3))];

%% 6. PARAMETRI STIMA ADATTIVA DI w (legge del gradiente)
% Legge di aggiornamento (gradiente della norma dell'errore di predizione):
%   w_hat_i(k+1) = w_hat_i(k) + gamma * (gi(z)' * epsilon) * u_i * dt
gamma  = 0.5;          % Learning rate
w_hat  = [1.0; 1.0];  % Stima iniziale: nessun guasto

%% 7. INIZIALIZZAZIONE VETTORI
z         = zeros(3, N);
z(:,1)    = [x_c + R_ref; y_c; pi/2];  % Robot sulla traiettoria circolare

z1_hat_vec = zeros(2, N);   z1_hat_vec(:,1) = Phi1(z(:,1));  % Stato stimato osservatore 1
z2_hat_vec = zeros(2, N);   z2_hat_vec(:,1) = Phi2(z(:,1));  % Stato stimato osservatore 2

r1_res    = zeros(1, N);   % Residuo FDI ruota destra
r2_res    = zeros(1, N);   % Residuo FDI ruota sinistra
err_track = zeros(1, N);   % Errore di tracking ||e(t)||
w_hat_log = ones(2, N);    % Log stima w
u_log     = zeros(2, N);   % Log ingressi applicati

cnt_fault  = [0; 0];         % Contatori persistenza guasto
conf_fault = [false; false];  % Guasto confermato?
ftc_active = false;
t_detect   = NaN;

fprintf('Avvio FTC (versione semplificata)...\n');

%% 8. LOOP DI SIMULAZIONE
for i = 1:N
    t = time(i);

    % Guasto attivo dopo T_fault
    w_now = [1.0; 1.0];
    if t >= T_fault,  w_now = w_real;  end

    % ---- A. CONTROLLORE DI TRACKING - Feedback Linearization (Punto 2) ----
    ex  = z(1,i) - xr(t);
    ey  = z(2,i) - yr(t);
    eth = wrapToPi(z(3,i) - thr(t));
    err_track(i) = sqrt(ex^2 + ey^2);

    cth = cos(z(3,i));  sth = sin(z(3,i));
    vx_d  = dxr(t) - kx*ex;
    vy_d  = dyr(t) - ky*ey;
    v_des  = cth*vx_d + sth*vy_d;
    om_des = w_ref - kth*eth;

    u1_nom = (v_des + l*om_des/2) / r1;
    u2_nom = (v_des - l*om_des/2) / r2;

    % ---- B. COMPENSAZIONE FEEDFORWARD FTC: u_ftc = u_nom / w_hat ----
    if ftc_active
        u1 = u1_nom / max(w_hat(1), 0.01);
        u2 = u2_nom / max(w_hat(2), 0.01);
    else
        u1 = u1_nom;
        u2 = u2_nom;
    end

    % Saturazione motori a +/-25 rad/s
    u1 = max(min(u1, u_max), -u_max);
    u2 = max(min(u2, u_max), -u_max);
    u_curr = [u1; u2];
    u_log(:,i) = u_curr;

    % ---- C. OSSERVATORI FDI & RESIDUI (Punto 4) ----
    z1_meas = Phi1(z(:,i));
    z2_meas = Phi2(z(:,i));

    r1_res(i) = norm(z1_meas - z1_hat_vec(:,i));
    r2_res(i) = norm(z2_meas - z2_hat_vec(:,i));

    % Logica conferma guasto con persistenza (0.1s)
    if r1_res(i) > FDI_THRESHOLD
        cnt_fault(1) = cnt_fault(1) + 1;
        if cnt_fault(1) >= FDI_PERSIST && ~conf_fault(1)
            conf_fault(1) = true;  ftc_active = true;  t_detect = t;
            fprintf('  [t=%.2fs] FDI: Guasto CONFERMATO su ruota 1. FTC attivo.\n', t);
        end
    else
        cnt_fault(1) = 0;
    end

    if r2_res(i) > FDI_THRESHOLD
        cnt_fault(2) = cnt_fault(2) + 1;
        if cnt_fault(2) >= FDI_PERSIST && ~conf_fault(2)
            conf_fault(2) = true;  ftc_active = true;
            if isnan(t_detect), t_detect = t; end
            fprintf('  [t=%.2fs] FDI: Guasto CONFERMATO su ruota 2. FTC attivo.\n', t);
        end
    else
        cnt_fault(2) = 0;
    end

    % ---- D. STIMA ADATTIVA DI w - legge del gradiente (attiva dopo FDI) ----
    if i > 2 && ftc_active
        dz_meas = (z(:,i) - z(:,i-1)) / dt;
        dz_meas(3) = wrapToPi(z(3,i) - z(3,i-1)) / dt; % Evita spike sul salto +/-pi
        dz_exp  = g1(z(3,i-1))*u_log(1,i-1)*w_hat(1) + ...
                  g2(z(3,i-1))*u_log(2,i-1)*w_hat(2);
        epsilon = dz_meas - dz_exp;

        if conf_fault(1)
            w_hat(1) = w_hat(1) + gamma*(g1(z(3,i-1))'*epsilon)*u_log(1,i-1)*dt;
            w_hat(1) = max(min(w_hat(1), 1.0), 0.01);
        end
        if conf_fault(2)
            w_hat(2) = w_hat(2) + gamma*(g2(z(3,i-1))'*epsilon)*u_log(2,i-1)*dt;
            w_hat(2) = max(min(w_hat(2), 1.0), 0.01);
        end
    end
    w_hat_log(:,i) = w_hat;

    % ---- E. INTEGRAZIONE DINAMICA REALE & OSSERVATORI ----
    if i < N
        dz = g1(z(3,i))*u1*w_now(1) + g2(z(3,i))*u2*w_now(2);
        z(:,i+1) = z(:,i) + dz*dt;
        z(3,i+1) = wrapToPi(z(3,i+1));

        % Aggiornamento osservatori FDI (feedback stabile)
        dir_vec = [cos(z(3,i)); sin(z(3,i))];
        dz1h = r1 * dir_vec * u_curr(1) + H_fdi * (z1_hat_vec(:,i) - z1_meas);
        dz2h = r2 * dir_vec * u_curr(2) + H_fdi * (z2_hat_vec(:,i) - z2_meas);

        z1_hat_vec(:,i+1) = z1_hat_vec(:,i) + dz1h*dt;
        z2_hat_vec(:,i+1) = z2_hat_vec(:,i) + dz2h*dt;
    end
end

fprintf('Simulazione completata.\n\n');

%% 9. STAMPA RISULTATI
err_pre  = mean(err_track(time < T_fault));
err_post = mean(err_track(time > T_fault + 5));

fprintf('========================================================\n');
fprintf('  PUNTO 5 (SEMPLIFICATO): RISULTATI FTC\n');
fprintf('========================================================\n');
fprintf('  Errore medio tracking Pre-Guasto  (0-%ds):   %.4f m\n', T_fault, err_pre);
fprintf('  Errore medio tracking Post-FTC   (%d-%ds): %.4f m\n', T_fault+5, T_tot, err_post);
if ~isnan(t_detect)
    fprintf('  Tempo di rilevamento FDI:             %.3f s (dopo guasto)\n', t_detect - T_fault);
end
fprintf('  Stima finale w1:  %.3f  (Valore reale: %.3f)\n', w_hat_log(1,end), w_real(1));
fprintf('  Stima finale w2:  %.3f  (Valore reale: %.3f)\n', w_hat_log(2,end), w_real(2));
fprintf('========================================================\n\n');

%% 10. GRAFICI
figure('Name','PUNTO 5 - FTC Semplificato','Color','k','Position',[50 50 1400 700]);

% Traiettoria XY
subplot(2,3,1);
th_v = linspace(0, 2*pi, 300);
plot(x_c+R_ref*cos(th_v), y_c+R_ref*sin(th_v), 'w--','LineWidth',1.5); hold on;
plot(z(1,:), z(2,:), 'b-', 'LineWidth',1.5);
if ~isnan(t_detect)
    idx_f = round(T_fault/dt)+1;  idx_d = round(t_detect/dt)+1;
    plot(z(1,idx_f), z(2,idx_f), 'rx','MarkerSize',10,'LineWidth',2);
    plot(z(1,idx_d), z(2,idx_d), 'ms','MarkerSize',8, 'LineWidth',2);
end
set(gca,'Color','k','XColor','w','YColor','w'); grid on; axis equal;
title('Traiettoria con FTC Attivo','Color','w');
xlabel('X [m]','Color','w'); ylabel('Y [m]','Color','w');
legend({'Riferimento','Traiettoria FTC','Inizio Guasto','Rilevamento FDI'},...
       'TextColor','w','Color','k','Location','best');

% Residui FDI
subplot(2,3,2);
plot(time, r1_res, 'b-','LineWidth',1.5); hold on;
plot(time, r2_res, 'r-','LineWidth',1.5);
yline(FDI_THRESHOLD,'g--','Soglia','LineWidth',1.2);
if ~isnan(t_detect), xline(t_detect,'m-','FDI','LineWidth',1.5); end
set(gca,'Color','k','XColor','w','YColor','w'); grid on;
title('Residui FDI','Color','w');
xlabel('Tempo [s]','Color','w'); ylabel('Residuo','Color','w');
legend({'r_1 (DX)','r_2 (SX)'},'TextColor','w','Color','k');

% Stima w
subplot(2,3,3);
plot(time, w_hat_log(1,:),'b-','LineWidth',1.5); hold on;
plot(time, w_hat_log(2,:),'r-','LineWidth',1.5);
yline(w_real(1),'--','LineWidth',1,'Color',[0.3 0.8 1]);
yline(w_real(2),'--','LineWidth',1,'Color',[1 0.5 0.5]);
if ~isnan(t_detect), xline(t_detect,'m-','FDI','LineWidth',1.5); end
set(gca,'Color','k','XColor','w','YColor','w'); grid on;
title('Stima Online w (Gradiente)','Color','w');
xlabel('Tempo [s]','Color','w'); ylabel('Efficienza stimata','Color','w');
legend({'Stima w_1','Stima w_2'},'TextColor','w','Color','k');
ylim([0 1.3]);

% Errore tracking
subplot(2,3,4);
plot(time, err_track,'b-','LineWidth',1.5); hold on;
if ~isnan(t_detect), xline(t_detect,'m-','FDI','LineWidth',1.5); end
set(gca,'Color','k','XColor','w','YColor','w'); grid on;
title('Errore di Tracking ||e(t)||','Color','w');
xlabel('Tempo [s]','Color','w'); ylabel('Errore [m]','Color','w');

% Ingressi
subplot(2,3,5);
plot(time, u_log(1,:),'b-','LineWidth',1.5); hold on;
plot(time, u_log(2,:),'r-','LineWidth',1.5);
yline( u_max,'g--','','LineWidth',1); yline(-u_max,'g--','','LineWidth',1);
if ~isnan(t_detect), xline(t_detect,'m-','FDI','LineWidth',1.5); end
set(gca,'Color','k','XColor','w','YColor','w'); grid on;
title('Ingressi di Controllo (Compensati)','Color','w');
xlabel('Tempo [s]','Color','w'); ylabel('rad/s','Color','w');
legend({'u_1 (Compensato)','u_2'},'TextColor','w','Color','k');

% Testo riepilogo
subplot(2,3,6);
set(gca,'Color','k','Visible','off');
txt = sprintf(['RIEPILOGO\n\n'...
    'Pre-guasto (0-%ds):\n  Errore = %.2f cm\n\n'...
    'Post-FTC (%d-%ds):\n  Errore = %.1f cm\n\n'...
    'FDI: dt_rilevamento = %.2fs\n\n'...
    'Stima:\n  w1 = %.3f (reale %.1f)\n  w2 = %.3f (reale %.1f)'],...
    T_fault, err_pre*100, T_fault+5, T_tot, err_post*100,...
    t_detect-T_fault, w_hat_log(1,end), w_real(1), w_hat_log(2,end), w_real(2));
text(0.05, 0.95, txt,'Units','normalized','VerticalAlignment','top',...
    'FontSize',10,'Color','w','FontName','Courier');

sgtitle('PUNTO 5 (Semplificato): FTC con FDI Integrato',...
    'Color','w','FontSize',13,'FontWeight','bold');
