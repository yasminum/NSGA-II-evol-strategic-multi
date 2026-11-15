%% simulacion_rapida.m  —  Simulacion RAPIDA del brazo RRR 3-GDL.
% ------------------------------------------------------------------
% SCRIPT AUTONOMO: corre en ~5 s, dibuja el brazo en 3D y la
% comparacion CTC fijo vs. Hibrido (CTC + MPC + NN) en el escenario
% S2 (incertidumbre parametrica). Diseñado para mostrar al profesor
% de forma inmediata.
%
% Uso (despues de extraer el ZIP y hacer 'cd' a la carpeta):
%   >> simulacion_rapida
%
% Salidas:
%   - Figure 1: animacion 3D del brazo (posicion del efector final).
%   - Figure 2: comparacion de error de seguimiento (CTC vs Hibrido).
%   - Impresion en consola de las metricas.
% ------------------------------------------------------------------
function simulacion_rapida()
  clc;
  fprintf('=========================================================\n');
  fprintf('  SIMULACION RAPIDA — Brazo RRR 3-GDL + Control Hibrido\n');
  fprintf('  (CTC + MPC + NN) | Escenario S2: Incertidumbre +10%%\n');
  fprintf('=========================================================\n\n');

  % ----- 1) Configuracion rapida -----
  p      = arm_params();
  tr     = default_trajectory();
  DT     = 0.005;       % 5 ms ciclo de control
  T      = 2.5;         % 2.5 s de simulacion (rapido)
  N_sub  = 3;           % 3 sub-pasos RK4 (mas rapido, suficiente)
  base_path = fileparts(mfilename('fullpath'));
  [theta0, ~] = ensure_offline(false, base_path);

  fprintf('Brazo RRR (Conejo 2021):\n');
  fprintf('  hB=%.3f m  l1=%.3f m  l2=%.3f m\n', p.d1, p.a2, p.a3);
  fprintf('  m1=%.3f kg  m2=%.3f kg  m3=%.3f kg\n', p.m1, p.m2, p.m3);
  fprintf('  tau_max = [%.1f %.1f %.1f] N*m (AX-18A/AX-12A/AX-12A)\n', p.tau_max);
  fprintf('  Tiempo simulacion: %.1f s | Ciclo control: %.0f ms\n\n', T, DT*1000);

  % ----- 2) Escenario S2 -----
  SC = scenarios_table();
  sc = SC.S2;
  p_plant = arm_with_uncertainty(p, sc.m2f, sc.m3f, sc.payload, sc.vf, sc.coulomb);
  fprintf('Escenario S2: m2 +%.0f%%  m3 +%.0f%%  payload=%.0f g  friccion x%.1f\n\n', ...
          (sc.m2f-1)*100, (sc.m3f-1)*100, sc.payload*1000, sc.vf);

  % ----- 3) Simulacion 1: CTC FIJO (baseline) -----
  fprintf('[1/3] CTC fijo ...\n');
  ctrl_ctc = CTC_PID(p, tr, DT, theta0(1:3), theta0(4:6), theta0(7:9));
  logs_ctc = simulate(p_plant, ctrl_ctc, tr, T, DT, 10001, ...
                       struct('q',0,'v',0), [], N_sub);
  m_ctc = metrics_from_logs(logs_ctc, DT);
  fprintf('      ISE = %.3e | IAE = %.3e | U2 = %.3e\n', m_ctc.ISE, m_ctc.IAE, m_ctc.U2);

  % ----- 4) Simulacion 2: HIBRIDO (CTC + MPC + NN) -----
  fprintf('[2/3] Hibrido (CTC + MPC + NN residual) ...\n');
  ctrl_hyb = HybridRobust(p, tr, DT, theta0(1:3), theta0(4:6), theta0(7:9), ...
                            5, 3, 0.1, 16, 0.1);
  logs_hyb = simulate(p_plant, ctrl_hyb, tr, T, DT, 10001, ...
                       struct('q',0,'v',0), [], N_sub);
  m_hyb = metrics_from_logs(logs_hyb, DT);
  fprintf('      ISE = %.3e | IAE = %.3e | U2 = %.3e\n', m_hyb.ISE, m_hyb.IAE, m_hyb.U2);

  % ----- 5) Comparacion -----
  mejora = m_ctc.ISE / max(m_hyb.ISE, 1e-12);
  fprintf('\n[3/3] Resultado:\n');
  fprintf('  CTC fijo:        ISE = %.3e\n', m_ctc.ISE);
  fprintf('  Hibrido:          ISE = %.3e\n', m_hyb.ISE);
  fprintf('  Mejora del Hibrido: %.2fx\n\n', mejora);

  % ----- 6) Figura 1: Animacion 3D del brazo (Hibrido) -----
  fprintf('Generando animacion 3D del brazo ...\n');
  fig1 = figure('Name', 'Brazo RRR 3-GDL (Hibrido)', ...
                'Position', [50 50 900 700], 'Color', 'w');
  ax = axes('Parent', fig1); hold(ax, 'on'); grid(ax, 'on'); view(ax, 3);
  axis(ax, 'equal'); axis(ax, [-0.3 0.3 -0.3 0.3 0 0.4]);
  xlabel(ax, 'X [m]'); ylabel(ax, 'Y [m]'); zlabel(ax, 'Z [m]');
  title(ax, 'Brazo Robotico RRR 3-GDL — Control Hibrido (S2)');
  L1 = plot3(ax, 0, 0, 0, 'o-', 'MarkerFaceColor', [0.85 0.10 0.55], ...
              'MarkerSize', 8, 'LineWidth', 3, 'Color', [0.3 0.3 0.3]);
  L2 = plot3(ax, 0, 0, 0, '--', 'Color', [0.85 0.10 0.55], 'LineWidth', 1.5);
  % Traza del efector final (linea de la trayectoria deseada)
  Pdes = cellfun(@(q) forward_kinematics(p, q.'), ...
                  num2cell(logs_hyb.qd, 2), 'UniformOutput', false);
  ee_des = cell2mat(cellfun(@(P) P{4}.', Pdes, 'UniformOutput', false));
  plot3(ax, ee_des(:,1), ee_des(:,2), ee_des(:,3), '--', 'Color', [0.2 0.2 0.2 0.5], ...
        'LineWidth', 1.5, 'DisplayName', 'Tray. deseada');

  % Animar
  n_frames = size(logs_hyb.q, 1);
  step_frame = max(1, floor(n_frames / 60));   % 60 frames maximo
  for k = 1:step_frame:n_frames
    q_k = logs_hyb.q(k, :).';
    Pk  = forward_kinematics(p, q_k);
    X = [Pk{1}(1) Pk{2}(1) Pk{3}(1) Pk{4}(1)];
    Y = [Pk{1}(2) Pk{2}(2) Pk{3}(2) Pk{4}(2)];
    Z = [Pk{1}(3) Pk{2}(3) Pk{3}(3) Pk{4}(3)];
    set(L1, 'XData', X, 'YData', Y, 'ZData', Z);
    set(L2, 'XData', Pk{4}(1), 'YData', Pk{4}(2), 'ZData', Pk{4}(3));
    title(ax, sprintf('Brazo RRR 3-GDL — t = %.2f s (Hibrido, S2)', logs_hyb.t(k)));
    drawnow;
  end
  title(ax, sprintf('Brazo RRR 3-GDL — Final (t = %.2f s) | ISE = %.2e', T, m_hyb.ISE));

  % ----- 7) Figura 2: Comparacion de error -----
  fig2 = figure('Name', 'Comparacion CTC vs Hibrido', ...
                'Position', [950 50 700 700], 'Color', 'w');
  subplot(3, 1, 1);
  hold on; grid on;
  plot(logs_ctc.t, rad2deg(logs_ctc.qd(:,2)), '--k', 'LineWidth', 1.2, 'DisplayName', 'q_{d,2}');
  plot(logs_ctc.t, rad2deg(logs_ctc.q(:,2)), '-', 'Color', [0.6 0.6 0.6], 'LineWidth', 1.5, 'DisplayName', 'CTC fijo');
  plot(logs_hyb.t, rad2deg(logs_hyb.q(:,2)), '-', 'Color', [0.85 0.10 0.55], 'LineWidth', 1.6, 'DisplayName', 'Hibrido');
  xlabel('t [s]'); ylabel('q_2 [deg]');
  title('Seguimiento articular 2 (escenario S2)'); legend('Location', 'best');

  subplot(3, 1, 2);
  hold on; grid on;
  plot(logs_ctc.t, sqrt(sum(logs_ctc.e.^2, 2)), '-', 'Color', [0.6 0.6 0.6], 'LineWidth', 1.5, 'DisplayName', 'CTC fijo');
  plot(logs_hyb.t, sqrt(sum(logs_hyb.e.^2, 2)), '-', 'Color', [0.85 0.10 0.55], 'LineWidth', 1.6, 'DisplayName', 'Hibrido');
  xlabel('t [s]'); ylabel('||e(t)|| [rad]');
  title('Norma del error de seguimiento'); set(gca, 'YScale', 'log');
  legend('Location', 'northeast');

  subplot(3, 1, 3);
  hold on; grid on;
  plot(logs_ctc.t, logs_ctc.tau(:,2), '-', 'Color', [0.6 0.6 0.6], 'LineWidth', 1.5, 'DisplayName', 'CTC fijo');
  plot(logs_hyb.t, logs_hyb.tau(:,2), '-', 'Color', [0.85 0.10 0.55], 'LineWidth', 1.6, 'DisplayName', 'Hibrido');
  yline(p.tau_max(2), '--r', sprintf('\\tau_{max} = %.1f', p.tau_max(2)));
  yline(-p.tau_max(2), '--r');
  xlabel('t [s]'); ylabel('\tau_2 [N m]');
  title('Esfuerzo de control (articulacion 2, con saturacion)'); legend('Location', 'best');

  sgtitle('Simulacion Rapida: CTC fijo vs. Hibrido (CTC + MPC + NN) en S2');

  % Guardar figura
  figs_dir = fullfile(base_path, 'results', 'figs');
  if ~exist(figs_dir, 'dir'), mkdir(figs_dir); end
  saveas(fig2, fullfile(figs_dir, 'fig_simulacion_rapida.png'));

  fprintf('\n=========================================================\n');
  fprintf('  Simulacion completada en ~5 s\n');
  fprintf('  Figuras guardadas en: results/figs/\n');
  fprintf('=========================================================\n');
end
