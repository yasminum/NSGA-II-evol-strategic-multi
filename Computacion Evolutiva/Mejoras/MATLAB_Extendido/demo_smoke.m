%% demo_smoke.m  —  Demostración rápida para el profesor (~20 s).
% ------------------------------------------------------------------
% Verifica en menos de 1 minuto que toda la cadena funciona en
% MATLAB u Octave:
%
%   1) Construye el brazo y valida la dinámica (M, g, F).
%   2) Muestra la geometría y la trayectoria de referencia.
%   3) Corre UNA simulación corta con CTC fijo (θ0 cacheado) en S2.
%   4) Corre UNA simulación corta con PR-DE adaptativo en S2.
%   5) Compara el ISE de las dos alternativas (la adaptación debe
%      reducirlo notablemente) y grafica el error.
%
% Uso:
%   >> demo_smoke
%
% Salidas:
%   - Impresión en consola de los parámetros del brazo y de los
%     resultados ISE de CTC vs. PR-DE.
%   - results/figs/fig_demo_smoke.png con la comparación de error.
% ------------------------------------------------------------------
function demo_smoke()
  fprintf('=== demo_smoke: Brazo RRR 3 GDL + control adaptativo ===\n\n');

  % 1) Parámetros del brazo
  p = arm_params();
  fprintf('Geometria: d1=%.3f m, a2=%.3f m, a3=%.3f m\n', p.d1, p.a2, p.a3);
  fprintf('Masas   : m1=%.3f, m2=%.3f, m3=%.3f kg\n', p.m1, p.m2, p.m3);
  fprintf('Friccion: Fv=[%.4f %.4f %.4f]  tau_max=[%.2f %.2f %.2f] N*m\n', ...
          p.Fv, p.tau_max);

  % Validación de la dinámica en una configuración típica
  q = [0.5; 0.6; 0.5];
  M = mass_matrix(p, q);
  g = gravity_vector(p, q);
  fprintf('\nM(q) en q = [0.5, 0.6, 0.5] (simetrica, def. positiva):\n');
  disp(M);
  fprintf('g(q) = [%.4f %.4f %.4f] N*m\n', g);

  % 2) Trayectoria de referencia
  tr = default_trajectory();
  fprintf('\nTrayectoria: 3 waypoints, T = 5 s\n');
  disp(tr.wp);

  % 3) Cargar ganancias offline cacheadas (o generarlas si no existen)
  [theta0, iseo] = ensure_offline(false, fileparts(mfilename('fullpath')));
  fprintf('\ntheta0 (offline): Kp=[%.1f %.1f %.1f]  Ki=[%.1f %.1f %.1f]  '...
          'Kd=[%.1f %.1f %.1f]  (ISE_offline=%.3e)\n', ...
          theta0(1), theta0(2), theta0(3), theta0(4), theta0(5), theta0(6), ...
          theta0(7), theta0(8), theta0(9), iseo);

  % 4) Escenario S2 (incertidumbre paramétrica)
  SC = scenarios_table();
  sc = SC.S2;
  p_plant = arm_with_uncertainty(p, sc.m2f, sc.m3f, sc.payload, ...
                                  sc.vf, sc.coulomb);
  DT = 0.005; T = 2.0;    % simulación corta (2 s)

  % ---- corrida con CTC fijo ----
  fprintf('\nCorriendo CTC fijo en S2 (T = 2 s)...\n');
  ctrl_ctc = CTC_PID(p, tr, DT, theta0(1:3), theta0(4:6), theta0(7:9));
  logs_ctc = simulate(p_plant, ctrl_ctc, tr, T, DT, 10001, ...
                      struct('q',0,'v',0), [], 5);
  m_ctc = metrics_from_logs(logs_ctc, DT);
  fprintf('  ISE = %.4e, IAE = %.4e, U2 = %.4e\n', m_ctc.ISE, m_ctc.IAE, m_ctc.U2);

  % ---- corrida con PR-DE adaptativo ----
  fprintf('Corriendo PR-DE adaptativo en S2 (T = 2 s)...\n');
  [lo, hi] = online_bounds(theta0);
  ctrl_ad = AdaptiveCTC(p, tr, DT, theta0(1:3), theta0(4:6), theta0(7:9), ...
                        lo, hi, 'DE', 2, 10, 1, 20, 20, 11);
  logs_ad = simulate(p_plant, ctrl_ad, tr, T, DT, 10001, ...
                      struct('q',0,'v',0), [], 5);
  m_ad = metrics_from_logs(logs_ad, DT);
  fprintf('  ISE = %.4e, IAE = %.4e, U2 = %.4e  (n_retunes = %d, '...
          'ms/ciclo = %.2f)\n', m_ad.ISE, m_ad.IAE, m_ad.U2, ...
          ctrl_ad.n_retunes, m_ad.ms_retune);

  % 5) Comparación
  fprintf('\n=== Comparacion ===\n');
  fprintf('  CTC fijo   ISE = %.4e\n', m_ctc.ISE);
  fprintf('  PR-DE adapt ISE = %.4e   (mejora x %.1f)\n', m_ad.ISE, ...
          m_ctc.ISE / m_ad.ISE);

  % 6) Figura comparativa
  figs_dir = fullfile(fileparts(mfilename('fullpath')), 'results', 'figs');
  if ~exist(figs_dir, 'dir'), mkdir(figs_dir); end
  figure('Position', [50 50 720 480]);
  subplot(2,1,1); hold on;
  plot(logs_ctc.t, rad2deg(logs_ctc.qd(:,2)), '--', 'Color', [0.4 0.4 0.4], ...
       'LineWidth', 1.2, 'DisplayName', 'q_{d,2}');
  plot(logs_ctc.t, rad2deg(logs_ctc.q(:,2)), '-', 'Color', [0.33 0.33 0.33], ...
       'LineWidth', 1.4, 'DisplayName', 'CTC fija');
  plot(logs_ad.t, rad2deg(logs_ad.q(:,2)), '-', 'Color', [0 0.45 0.70], ...
       'LineWidth', 1.4, 'DisplayName', 'PR-DE');
  xlabel('t [s]'); ylabel('q_2 [deg]'); title('Seguimiento articular 2 (S2)');
  legend('Location', 'best'); grid on;

  subplot(2,1,2); hold on;
  plot(logs_ctc.t, sqrt(sum(logs_ctc.e.^2, 2)), '-', ...
       'Color', [0.33 0.33 0.33], 'LineWidth', 1.4, 'DisplayName', 'CTC fija');
  plot(logs_ad.t, sqrt(sum(logs_ad.e.^2, 2)), '-', ...
       'Color', [0 0.45 0.70], 'LineWidth', 1.4, 'DisplayName', 'PR-DE');
  xlabel('t [s]'); ylabel('||e(t)|| [rad]');
  title('Norma del error de seguimiento (S2)'); set(gca, 'YScale', 'log');
  legend('Location', 'northeast'); grid on;

  sgtitle('demo_smoke: CTC fija vs. PR-DE adaptativo (escenario S2)');
  saveas(gcf, fullfile(figs_dir, 'fig_demo_smoke.png'));
  fprintf('\nFigura guardada en: %s\n', fullfile(figs_dir, 'fig_demo_smoke.png'));
end
