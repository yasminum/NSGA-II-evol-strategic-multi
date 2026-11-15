%% simulacion_minima.m  —  Simulacion MINIMA del brazo RRR (sin dependencias).
% ------------------------------------------------------------------
% Script AUTOCONTENIDO (~3 segundos) que simula el brazo RRR de 3-GDL
% (Conejo Benitez 2021) con un control CTC-PID simple, mostrando la
% animacion 3D del brazo y la trayectoria del efector.
%
% NO requiere la cadena de controladores adaptativos; solo el modelo
% dinamico del brazo (que esta en la carpeta) y un CTC-PID simple.
% Diseñado para ejecutarse URGENTEMENTE y mostrar el brazo moviendose.
%
% Uso:
%   >> simulacion_minima
% ------------------------------------------------------------------
function simulacion_minima()
  clc;
  fprintf('=== Simulacion Minima: Brazo RRR 3-GDL con CTC-PID ===\n\n');

  % 1) Parametros del brazo (Conejo 2021)
  p = arm_params();
  fprintf('Geometria: hB=%.3f  l1=%.3f  l2=%.3f [m]\n', p.d1, p.a2, p.a3);
  fprintf('Masas: m1=%.3f  m2=%.3f  m3=%.3f [kg]\n', p.m1, p.m2, p.m3);
  fprintf('Par max: [%.1f %.1f %.1f] N*m\n', p.tau_max);

  % 2) Trayectoria de referencia (5 s, 3 waypoints)
  tr = default_trajectory();

  % 3) Controlador CTC-PID con ganancias offline cacheadas
  base_path = fileparts(mfilename('fullpath'));
  [theta0, ~] = ensure_offline(false, base_path);
  fprintf('Ganancias CTC: Kp=[%.0f %.0f %.0f]  Ki=[%.0f %.0f %.0f]  Kd=[%.0f %.0f %.0f]\n\n', ...
          theta0(1:3), theta0(4:6), theta0(7:9));

  % 4) Simulacion (planta nominal, sin perturbaciones)
  DT = 0.005; T = 5.0;
  fprintf('Simulando %.1f s (paso %.0f ms) ...\n', T, DT*1000);
  ctrl = CTC_PID(p, tr, DT, theta0(1:3), theta0(4:6), theta0(7:9));
  logs = simulate(p, ctrl, tr, T, DT, 10001, struct('q',0,'v',0), [], 5);
  m = metrics_from_logs(logs, DT);
  fprintf('ISE = %.3e | IAE = %.3e | U2 = %.3e\n\n', m.ISE, m.IAE, m.U2);

  % 5) Animacion 3D
  fprintf('Animando el brazo ...\n');
  fig = figure('Name', 'Brazo RRR 3-GDL', 'Position', [100 100 900 700], 'Color', 'w');
  ax = axes('Parent', fig); hold(ax, 'on'); grid(ax, 'on'); view(ax, 3);
  axis(ax, 'equal'); axis(ax, [-0.3 0.3 -0.3 0.3 0 0.45]);
  xlabel(ax, 'X [m]'); ylabel(ax, 'Y [m]'); zlabel(ax, 'Z [m]');
  title(ax, 'Brazo Robotico RRR 3-GDL (Conejo 2021) — CTC-PID');

  % Trayectoria deseada del efector (linea de referencia)
  Pdes = cellfun(@(q) forward_kinematics(p, q.'), ...
                  num2cell(logs.qd, 2), 'UniformOutput', false);
  ee_des = cell2mat(cellfun(@(P) P{4}.', Pdes, 'UniformOutput', false));
  plot3(ax, ee_des(:,1), ee_des(:,2), ee_des(:,3), '--', 'Color', [0.5 0.5 0.5], ...
        'LineWidth', 1.5, 'DisplayName', 'Tray. deseada');

  % Trayectoria REAL del efector (linea de seguimiento)
  P_real = cellfun(@(q) forward_kinematics(p, q.'), ...
                    num2cell(logs.q, 2), 'UniformOutput', false);
  ee_real = cell2mat(cellfun(@(P) P{4}.', P_real, 'UniformOutput', false));
  plot3(ax, ee_real(:,1), ee_real(:,2), ee_real(:,3), '-', 'Color', [0 0.0 0.7], ...
        'LineWidth', 1.2, 'DisplayName', 'Tray. real');

  % Eslabones del brazo (linea animada)
  L_arm = plot3(ax, 0, 0, 0, 'o-', 'MarkerFaceColor', [0.10 0.45 0.85], ...
                'MarkerSize', 10, 'LineWidth', 4, 'Color', [0.2 0.2 0.2]);

  legend(ax, 'Location', 'northeast');

  % Animar
  n = size(logs.q, 1);
  step = max(1, floor(n / 80));   % ~80 frames
  for k = 1:step:n
    q_k = logs.q(k, :).';
    Pk  = forward_kinematics(p, q_k);
    X = [Pk{1}(1) Pk{2}(1) Pk{3}(1) Pk{4}(1)];
    Y = [Pk{1}(2) Pk{2}(2) Pk{3}(2) Pk{4}(2)];
    Z = [Pk{1}(3) Pk{2}(3) Pk{3}(3) Pk{4}(3)];
    set(L_arm, 'XData', X, 'YData', Y, 'ZData', Z);
    title(ax, sprintf('t = %.2f s | q = [%.2f, %.2f, %.2f] rad', ...
                       logs.t(k), q_k(1), q_k(2), q_k(3)));
    drawnow;
  end

  % 6) Figuras adicionales: seguimiento articular y torque
  figure('Name', 'Seguimiento y esfuerzo', 'Position', [150 100 900 700], 'Color', 'w');

  subplot(2, 2, 1); hold on; grid on;
  plot(logs.t, rad2deg(logs.qd(:,1)), '--k', 'DisplayName', 'q_{d,1}');
  plot(logs.t, rad2deg(logs.q(:,1)),  '-', 'Color', [0.10 0.45 0.85], 'LineWidth', 1.5, 'DisplayName', 'q_1');
  xlabel('t [s]'); ylabel('q_1 [deg]'); title('Articulacion 1 (base)'); legend('Location','best');

  subplot(2, 2, 2); hold on; grid on;
  plot(logs.t, rad2deg(logs.qd(:,2)), '--k', 'DisplayName', 'q_{d,2}');
  plot(logs.t, rad2deg(logs.q(:,2)),  '-', 'Color', [0.10 0.45 0.85], 'LineWidth', 1.5, 'DisplayName', 'q_2');
  xlabel('t [s]'); ylabel('q_2 [deg]'); title('Articulacion 2 (hombro)'); legend('Location','best');

  subplot(2, 2, 3); hold on; grid on;
  plot(logs.t, rad2deg(logs.qd(:,3)), '--k', 'DisplayName', 'q_{d,3}');
  plot(logs.t, rad2deg(logs.q(:,3)),  '-', 'Color', [0.10 0.45 0.85], 'LineWidth', 1.5, 'DisplayName', 'q_3');
  xlabel('t [s]'); ylabel('q_3 [deg]'); title('Articulacion 3 (codo)'); legend('Location','best');

  subplot(2, 2, 4); hold on; grid on;
  plot(logs.t, logs.tau(:,1), '-', 'DisplayName', '\tau_1');
  plot(logs.t, logs.tau(:,2), '-', 'DisplayName', '\tau_2');
  plot(logs.t, logs.tau(:,3), '-', 'DisplayName', '\tau_3');
  xlabel('t [s]'); ylabel('\tau [N m]'); title('Pares motor'); legend('Location','best');

  sgtitle('Simulacion Minima: Brazo RRR 3-GDL con CTC-PID (planta nominal)');

  % 7) Guardar
  figs_dir = fullfile(base_path, 'results', 'figs');
  if ~exist(figs_dir, 'dir'), mkdir(figs_dir); end
  saveas(figure(1), fullfile(figs_dir, 'fig_simulacion_minima_3d.png'));
  saveas(figure(2), fullfile(figs_dir, 'fig_simulacion_minima_seguimiento.png'));
  fprintf('\nFiguras guardadas en: %s\n', figs_dir);
  fprintf('=== Simulacion completada ===\n');
end
