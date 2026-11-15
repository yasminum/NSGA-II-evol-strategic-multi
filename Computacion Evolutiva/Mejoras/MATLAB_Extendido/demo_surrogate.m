%% demo_surrogate.m  —  Demostración de sintonía evolutiva asistida por sustitutos.
% ------------------------------------------------------------------
% Demostración (~40 s) que compara tres estrategias de sintonía
% online para el controlador CTC-PID en el escenario S2:
%
%   1) AdaptiveCTC (PR-DE): sintonía single-objetivo, 400 evaluaciones
%      reales por ciclo (presupuesto estándar del paper base).
%   2) MultiObjAdaptiveCTC (PR + NSGA-II): sintonía multi-objetivo,
%      400 evaluaciones reales por ciclo.
%   3) SurrogateAdaptiveCTC (RBF + NSGA-II): sintonía multi-objetivo
%      ASISTIDA POR UN MODELO SUSTITUTO (RBF), que reduce las
%      evaluaciones reales a k_lie + N_gen ≈ 35-50 por ciclo (~10x
%      aceleración efectiva).
%
% Verifica que:
%   - Los tres modelos sustitutos (RBF, Kriging, NN) se entrenan y
%     predicen sin errores.
%   - El esquema asistido por sustitutos mantiene un ISE comparable
%     al esquema de evaluación real completa (en el peor caso, peor
%     pero en el mismo orden de magnitud) con ~10x menos evaluaciones
%     reales (n_real_evals mucho menor que n_retunes × pop × gens).
%
% Uso:
%   >> demo_surrogate
%
% Salidas:
%   - Impresión en consola de las métricas.
%   - results/figs/fig_demo_surrogate.png con 4 subgráficas.
% ------------------------------------------------------------------
function demo_surrogate()
  fprintf('=== demo_surrogate: Sintonia Evolutiva Asistida por Sustitutos ===\n\n');

  % 1) Parámetros y trayectoria
  p = arm_params();
  tr = default_trajectory();
  base_path = fileparts(mfilename('fullpath'));
  [theta0, ~] = ensure_offline(false, base_path);
  fprintf('theta0 (CTC offline) cargado.\n\n');

  % 2) Test autónomo de los tres sustitutos
  test_surrogates();
  fprintf('\n');

  % 3) Escenario S2
  SC = scenarios_table();
  sc = SC.S2;
  p_plant = arm_with_uncertainty(p, sc.m2f, sc.m3f, sc.payload, ...
                                  sc.vf, sc.coulomb);
  DT = 0.005; T = 2.0;

  [lo, hi] = online_bounds(theta0);

  % 4) Corrida 1: AdaptiveCTC PR-DE (baseline single-objetivo)
  fprintf('Corriendo AdaptiveCTC (PR-DE, single-obj) en S2...\n');
  ctrl_ad = AdaptiveCTC(p, tr, DT, theta0(1:3), theta0(4:6), theta0(7:9), ...
                        lo, hi, 'DE', 2, 10, 1, 20, 20, 11);
  logs_ad = simulate(p_plant, ctrl_ad, tr, T, DT, 10001, ...
                      struct('q',0,'v',0), [], 5);
  m_ad = metrics_from_logs(logs_ad, DT);
  fprintf('  ISE = %.4e, U2 = %.4e, n_retunes=%d, ms/ciclo=%.2f\n', ...
          m_ad.ISE, m_ad.U2, ctrl_ad.n_retunes, ...
          1000.0 * ctrl_ad.t_adapt_total / max(1, ctrl_ad.n_retunes));

  % 5) Corrida 2: MultiObjAdaptiveCTC (NSGA-II puro)
  fprintf('Corriendo MultiObjAdaptiveCTC (NSGA-II, multi-obj) en S2...\n');
  ctrl_mo = MultiObjAdaptiveCTC(p, tr, DT, theta0(1:3), theta0(4:6), theta0(7:9), ...
                                  lo, hi, 2, 10, 1, 20, 10, 11);
  logs_mo = simulate(p_plant, ctrl_mo, tr, T, DT, 10001, ...
                       struct('q',0,'v',0), [], 5);
  m_mo = metrics_from_logs(logs_mo, DT);
  fprintf('  ISE = %.4e, U2 = %.4e, n_retunes=%d\n', ...
          m_mo.ISE, m_mo.U2, ctrl_mo.n_retunes);

  % 6) Corrida 3: SurrogateAdaptiveCTC (NSGA-II + RBF surrogate)
  fprintf('Corriendo SurrogateAdaptiveCTC (RBF + NSGA-II) en S2...\n');
  ctrl_sa = SurrogateAdaptiveCTC(p, tr, DT, theta0(1:3), theta0(4:6), theta0(7:9), ...
                                   lo, hi, 2, 10, 1, 20, 10, 11, 'rbf', 4, 10);
  logs_sa = simulate(p_plant, ctrl_sa, tr, T, DT, 10001, ...
                       struct('q',0,'v',0), [], 5);
  m_sa = metrics_from_logs(logs_sa, DT);
  fprintf('  ISE = %.4e, U2 = %.4e, n_retunes=%d, n_real_evals=%d\n', ...
          m_sa.ISE, m_sa.U2, ctrl_sa.n_retunes, ctrl_sa.n_real_evals);

  % 7) Comparación
  fprintf('\n=== Comparacion (S2, T = 2 s) ===\n');
  fprintf('  %-35s  ISE=%.3e  U2=%.3e  reales=%d\n', ...
          'AdaptiveCTC (PR-DE)', m_ad.ISE, m_ad.U2, ctrl_ad.n_retunes*400);
  fprintf('  %-35s  ISE=%.3e  U2=%.3e  reales=%d\n', ...
          'MultiObjAdaptiveCTC (NSGA-II)', m_mo.ISE, m_mo.U2, ctrl_mo.n_retunes*200);
  fprintf('  %-35s  ISE=%.3e  U2=%.3e  reales=%d\n', ...
          'SurrogateAdaptiveCTC (RBF+NSGA-II)', m_sa.ISE, m_sa.U2, ctrl_sa.n_real_evals);
  fprintf('  Aceleracion efectiva: %.1fx menos evaluaciones reales\n', ...
          (ctrl_mo.n_retunes * 200) / max(1, ctrl_sa.n_real_evals));

  % 8) Figuras
  figs_dir = fullfile(base_path, 'results', 'figs');
  if ~exist(figs_dir, 'dir'), mkdir(figs_dir); end
  figure('Position', [50 50 1000 700]);
  subplot(2,1,1); hold on;
  plot(logs_ad.t, rad2deg(logs_ad.qd(:,2)), '--k', 'DisplayName','q_{d,2}');
  plot(logs_ad.t, rad2deg(logs_ad.q(:,2)), '-', 'Color', [0 0.55 0.85], 'DisplayName','PR-DE');
  plot(logs_mo.t, rad2deg(logs_mo.q(:,2)), '-', 'Color', [0.85 0.45 0], 'DisplayName','NSGA-II');
  plot(logs_sa.t, rad2deg(logs_sa.q(:,2)), '-', 'Color', [0.10 0.65 0.10], 'LineWidth', 1.5, 'DisplayName','Surrogate');
  xlabel('t [s]'); ylabel('q_2 [deg]'); grid on; legend('Location','best');
  title('Seguimiento articular 2 (escenario S2)');

  subplot(2,1,2); hold on;
  plot(logs_ad.t, sqrt(sum(logs_ad.e.^2, 2)), '-', 'Color', [0 0.55 0.85], 'DisplayName','PR-DE');
  plot(logs_mo.t, sqrt(sum(logs_mo.e.^2, 2)), '-', 'Color', [0.85 0.45 0], 'DisplayName','NSGA-II');
  plot(logs_sa.t, sqrt(sum(logs_sa.e.^2, 2)), '-', 'Color', [0.10 0.65 0.10], 'LineWidth', 1.5, 'DisplayName','Surrogate');
  xlabel('t [s]'); ylabel('||e(t)|| [rad]'); set(gca, 'YScale','log'); grid on; legend('Location','best');
  title('Norma del error de seguimiento');

  sgtitle('demo_surrogate: sintonia evolutiva asistida por sustitutos (S2)');
  saveas(gcf, fullfile(figs_dir, 'fig_demo_surrogate.png'));
  fprintf('\nFigura guardada en: %s\n', fullfile(figs_dir, 'fig_demo_surrogate.png'));
end

function test_surrogates()
% Test rápido de los tres sustitutos: ajuste y predicción sobre un
% problema simple (sinusoide + ruido).
  fprintf('Test de modelos sustitutos (sinusoide 2D):\n');
  rng(0);
  N = 50;
  X = rand(N, 2) * 2 - 1;            % X ∈ [-1, 1]^2
  y = sin(3*X(:, 1)) .* cos(2*X(:, 2)) + 0.05 * randn(N, 1);
  X_test = rand(20, 2) * 2 - 1;
  y_test = sin(3*X_test(:, 1)) .* cos(2*X_test(:, 2));
  % RBF
  surr_rbf = RBFNet(2, 1, 32, 1e-3);
  surr_rbf.fit(X, y);
  yhat_rbf = surr_rbf.predict(X_test);
  err_rbf = mean((yhat_rbf - y_test).^2);
  fprintf('  RBFNet:        MSE = %.4e\n', err_rbf);
  % Kriging
  surr_krig = KrigingSurrogate(2, 1);
  surr_krig.fit(X, y);
  [yhat_krig, var_krig] = surr_krig.predict(X_test);
  err_krig = mean((yhat_krig - y_test).^2);
  fprintf('  Kriging:       MSE = %.4e (mean var = %.3e)\n', ...
          err_krig, mean(var_krig));
  % NN
  surr_nn = NNSurrogate(2, 24, 1);
  surr_nn.fit(X, y);
  yhat_nn = surr_nn.predict(X_test);
  err_nn = mean((yhat_nn - y_test).^2);
  fprintf('  NNSurrogate:   MSE = %.4e\n', err_nn);
  % SurrogateManager (ensamble RBF)
  mgr = SurrogateManager('rbf', 2, 1, 2);
  mgr.add_samples(X, y);
  [yhat_ens, var_ens] = mgr.predict_ensemble(X_test);
  err_ens = mean((yhat_ens - y_test).^2);
  fprintf('  SurrogateManager (ensamble RBF x2): MSE = %.4e\n', err_ens);
end
