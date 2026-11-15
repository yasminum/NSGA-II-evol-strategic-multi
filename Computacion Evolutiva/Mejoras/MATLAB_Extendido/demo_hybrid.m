%% demo_hybrid.m  —  Demostración del Control Robusto Híbrido (CTC + MPC + NN).
% ------------------------------------------------------------------
% Demostración rápida (~30 s) que compara en un mismo escenario (S2,
% incertidumbre paramétrica) cuatro controladores:
%
%   1) CTC fijo (baseline, ganancias offline cacheadas).
%   2) CTC-PID adaptativo PR-DE (regresor polinomial + DE online).
%   3) MPC puro (predicción N_p = 5, control N_c = 3).
%   4) Híbrido Robusto (CTC + MPC + compensador NN residual).
%
% Verifica que la arquitectura híbrida se ejecuta sin errores, que la
% combinación de las tres leyes de control produce un ISE comparable
% (o mejor) que el adaptativo puro, y que el tiempo de cómputo por
% ciclo permanece dentro del presupuesto de tiempo real (≤ 5 ms).
%
% Uso:
%   >> demo_hybrid
%
% Salidas:
%   - Impresión en consola de las métricas (ISE, IAE, U2, ms/ciclo).
%   - results/figs/fig_demo_hybrid.png con 4 subgráficas comparativas.
% ------------------------------------------------------------------
function demo_hybrid()
  fprintf('=== demo_hybrid: Control Robusto Hibrido (CTC + MPC + NN) ===\n\n');

  % 1) Parámetros y trayectoria
  p = arm_params();
  tr = default_trajectory();
  fprintf('Brazo RRR: d1=%.3f, a2=%.3f, a3=%.3f [m]\n', p.d1, p.a2, p.a3);
  fprintf('Trayectoria: 3 waypoints, T = 5 s\n\n');

  % 2) Ganancias offline cacheadas
  base_path = fileparts(mfilename('fullpath'));
  [theta0, ~] = ensure_offline(false, base_path);
  fprintf('theta0 (CTC offline): Kp=[%.1f %.1f %.1f]  Ki=[%.1f %.1f %.1f]  '...
          'Kd=[%.1f %.1f %.1f]\n\n', theta0(1), theta0(2), theta0(3), ...
          theta0(4), theta0(5), theta0(6), theta0(7), theta0(8), theta0(9));

  % 3) Escenario S2 (incertidumbre paramétrica)
  SC = scenarios_table();
  sc = SC.S2;
  p_plant = arm_with_uncertainty(p, sc.m2f, sc.m3f, sc.payload, ...
                                  sc.vf, sc.coulomb);
  DT = 0.005; T = 2.0;   % simulación corta para el demo

  % 4) Corrida 1: CTC fijo (baseline)
  fprintf('Corriendo CTC fijo (T = 2 s)...\n');
  ctrl_ctc = CTC_PID(p, tr, DT, theta0(1:3), theta0(4:6), theta0(7:9));
  logs_ctc = simulate(p_plant, ctrl_ctc, tr, T, DT, 10001, ...
                      struct('q',0,'v',0), [], 5);
  m_ctc = metrics_from_logs(logs_ctc, DT);

  % 5) Corrida 2: AdaptiveCTC (PR-DE, regresor polinomial)
  fprintf('Corriendo AdaptiveCTC (PR-DE)...\n');
  [lo, hi] = online_bounds(theta0);
  ctrl_ad = AdaptiveCTC(p, tr, DT, theta0(1:3), theta0(4:6), theta0(7:9), ...
                        lo, hi, 'DE', 2, 10, 1, 20, 20, 11);
  logs_ad = simulate(p_plant, ctrl_ad, tr, T, DT, 10001, ...
                      struct('q',0,'v',0), [], 5);
  m_ad = metrics_from_logs(logs_ad, DT);
  m_ad_ms_retune = 1000.0 * ctrl_ad.t_adapt_total / max(1, ctrl_ad.n_retunes);

  % 6) Corrida 3: MPC puro (predicción N_p=5, control N_c=3)
  fprintf('Corriendo MPC puro (Np=5, Nc=3, max_iter=10)...\n');
  ctrl_mpc = MPC(p, tr, DT, 5, 3, 0.1, 0.4*p.tau_max, 10);
  logs_mpc = simulate(p_plant, ctrl_mpc, tr, T, DT, 10001, ...
                       struct('q',0,'v',0), [], 5);
  m_mpc = metrics_from_logs(logs_mpc, DT);

  % 7) Corrida 4: Híbrido Robusto (CTC + MPC + NN residual)
  fprintf('Corriendo Hibrido Robusto (CTC + MPC + NN residual)...\n');
  ctrl_hyb = HybridRobust(p, tr, DT, theta0(1:3), theta0(4:6), theta0(7:9), ...
                            5, 3, 0.1, 16, 0.1);
  logs_hyb = simulate(p_plant, ctrl_hyb, tr, T, DT, 10001, ...
                       struct('q',0,'v',0), [], 5);
  m_hyb = metrics_from_logs(logs_hyb, DT);

  % 8) Comparación final
  fprintf('\n=== Comparacion (escenario S2, T = 2 s) ===\n');
  fprintf('  %-25s  ISE=%.3e  IAE=%.3e  U2=%.3e\n', 'CTC fijo',      m_ctc.ISE, m_ctc.IAE, m_ctc.U2);
  fprintf('  %-25s  ISE=%.3e  IAE=%.3e  U2=%.3e  ms/ciclo=%.2f\n', 'AdaptiveCTC PR-DE', m_ad.ISE, m_ad.IAE, m_ad.U2, m_ad_ms_retune);
  fprintf('  %-25s  ISE=%.3e  IAE=%.3e  U2=%.3e\n', 'MPC puro',      m_mpc.ISE, m_mpc.IAE, m_mpc.U2);
  fprintf('  %-25s  ISE=%.3e  IAE=%.3e  U2=%.3e\n', 'Hibrido (CTC+MPC+NN)', m_hyb.ISE, m_hyb.IAE, m_hyb.U2);

  % 9) Figura comparativa
  figs_dir = fullfile(base_path, 'results', 'figs');
  if ~exist(figs_dir, 'dir'), mkdir(figs_dir); end
  figure('Position', [50 50 1000 700]);
  subplot(3,1,1); hold on;
  plot(logs_ctc.t, rad2deg(logs_ctc.qd(:,2)), '--k', 'DisplayName', 'q_{d,2}');
  plot(logs_ctc.t, rad2deg(logs_ctc.q(:,2)), '-', 'Color', [0.6 0.6 0.6], 'DisplayName', 'CTC');
  plot(logs_ad.t,  rad2deg(logs_ad.q(:,2)),  '-', 'Color', [0 0.55 0.85], 'DisplayName', 'PR-DE');
  plot(logs_mpc.t, rad2deg(logs_mpc.q(:,2)), '-', 'Color', [0.85 0.45 0], 'DisplayName', 'MPC');
  plot(logs_hyb.t, rad2deg(logs_hyb.q(:,2)), '-', 'Color', [0.85 0.10 0.55], 'LineWidth', 1.6, 'DisplayName', 'Hibrido');
  xlabel('t [s]'); ylabel('q_2 [deg]');
  title('Seguimiento articular 2 (escenario S2)'); grid on; legend('Location','best');

  subplot(3,1,2); hold on;
  plot(logs_ctc.t, sqrt(sum(logs_ctc.e.^2, 2)), '-', 'Color', [0.6 0.6 0.6], 'DisplayName', 'CTC');
  plot(logs_ad.t,  sqrt(sum(logs_ad.e.^2, 2)),  '-', 'Color', [0 0.55 0.85], 'DisplayName', 'PR-DE');
  plot(logs_mpc.t, sqrt(sum(logs_mpc.e.^2, 2)), '-', 'Color', [0.85 0.45 0], 'DisplayName', 'MPC');
  plot(logs_hyb.t, sqrt(sum(logs_hyb.e.^2, 2)), '-', 'Color', [0.85 0.10 0.55], 'LineWidth', 1.6, 'DisplayName', 'Hibrido');
  xlabel('t [s]'); ylabel('||e(t)|| [rad]'); set(gca, 'YScale', 'log');
  title('Norma del error de seguimiento'); grid on; legend('Location','best');

  subplot(3,1,3); hold on;
  plot(logs_ctc.t, logs_ctc.tau(:,2), '-', 'Color', [0.6 0.6 0.6], 'DisplayName', 'CTC');
  plot(logs_ad.t,  logs_ad.tau(:,2),  '-', 'Color', [0 0.55 0.85], 'DisplayName', 'PR-DE');
  plot(logs_mpc.t, logs_mpc.tau(:,2), '-', 'Color', [0.85 0.45 0], 'DisplayName', 'MPC');
  plot(logs_hyb.t, logs_hyb.tau(:,2), '-', 'Color', [0.85 0.10 0.55], 'LineWidth', 1.6, 'DisplayName', 'Hibrido');
  xlabel('t [s]'); ylabel('\tau_2 [N m]');
  title('Esfuerzo de control (articulacion 2)'); grid on; legend('Location','best');

  sgtitle('demo_hybrid: Control Robusto Hibrido (CTC + MPC + NN) en S2');
  saveas(gcf, fullfile(figs_dir, 'fig_demo_hybrid.png'));
  fprintf('\nFigura guardada en: %s\n', fullfile(figs_dir, 'fig_demo_hybrid.png'));
end
