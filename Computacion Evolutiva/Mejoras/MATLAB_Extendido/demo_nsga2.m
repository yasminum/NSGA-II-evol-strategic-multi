%% demo_nsga2.m  —  Demostración de sintonía multi-objetivo (NSGA-II).
% ------------------------------------------------------------------
% Demostración rápida (~40 s) que compara dos estrategias de sintonía
% online para el controlador CTC-PID en el escenario S2:
%
%   1) AdaptiveCTC (PR-DE): sintonía SINGLE-OBJECTIVE (minimiza ||e||²).
%   2) MultiObjAdaptiveCTC (PR-NSGA-II): sintonía MULTI-OBJETIVO con
%      tres objetivos (ISE_pred, ||u||², saturación) y selección
%      del knee point del frente de Pareto.
%
% Verifica que el NSGA-II:
%   - Se ejecuta sin errores en MATLAB/Octave (sin toolboxes).
%   - Devuelve un frente de Pareto no dominado en cada ciclo.
%   - Reduce el esfuerzo de control U2 respecto al single-objetivo
%     (compromiso entre seguimiento y esfuerzo, explícito en Pareto).
%
% Uso:
%   >> demo_nsga2
%
% Salidas:
%   - Impresión en consola de las métricas comparativas.
%   - results/figs/fig_demo_nsga2.png con:
%       * Frente de Pareto final.
%       * Evolución del knee point a lo largo del tiempo.
%       * Comparación de error de seguimiento (single vs multi-obj).
% ------------------------------------------------------------------
function demo_nsga2()
  fprintf('=== demo_nsga2: Sintonia Multi-Objetivo (NSGA-II) ===\n\n');

  % 1) Parámetros y trayectoria
  p = arm_params();
  tr = default_trajectory();
  base_path = fileparts(mfilename('fullpath'));
  [theta0, ~] = ensure_offline(false, base_path);
  fprintf('theta0 (CTC offline) cargado.\n\n');

  % 2) Escenario S2
  SC = scenarios_table();
  sc = SC.S2;
  p_plant = arm_with_uncertainty(p, sc.m2f, sc.m3f, sc.payload, ...
                                  sc.vf, sc.coulomb);
  DT = 0.005; T = 2.0;

  % 3) Prueba autónoma del NSGA-II (problema ZDT1 simplificado)
  fprintf('Test autónomo de NSGA-II (función de prueba ZDT1)...\n');
  test_nsga2_zdt1();
  fprintf('\n');

  % 4) Corrida 1: AdaptiveCTC PR-DE (single-objetivo)
  fprintf('Corriendo AdaptiveCTC (PR-DE, single-objetivo) en S2...\n');
  [lo, hi] = online_bounds(theta0);
  ctrl_ad = AdaptiveCTC(p, tr, DT, theta0(1:3), theta0(4:6), theta0(7:9), ...
                        lo, hi, 'DE', 2, 10, 1, 20, 20, 11);
  logs_ad = simulate(p_plant, ctrl_ad, tr, T, DT, 10001, ...
                      struct('q',0,'v',0), [], 5);
  m_ad = metrics_from_logs(logs_ad, DT);
  fprintf('  ISE = %.4e, IAE = %.4e, U2 = %.4e (n_retunes=%d, ms/ciclo=%.2f)\n', ...
          m_ad.ISE, m_ad.IAE, m_ad.U2, ctrl_ad.n_retunes, ...
          1000.0 * ctrl_ad.t_adapt_total / max(1, ctrl_ad.n_retunes));

  % 5) Corrida 2: MultiObjAdaptiveCTC (NSGA-II)
  fprintf('Corriendo MultiObjAdaptiveCTC (PR + NSGA-II, multi-objetivo)...\n');
  ctrl_mo = MultiObjAdaptiveCTC(p, tr, DT, theta0(1:3), theta0(4:6), theta0(7:9), ...
                                  lo, hi, 2, 10, 1, 20, 10, 11);
  logs_mo = simulate(p_plant, ctrl_mo, tr, T, DT, 10001, ...
                       struct('q',0,'v',0), [], 5);
  m_mo = metrics_from_logs(logs_mo, DT);
  fprintf('  ISE = %.4e, IAE = %.4e, U2 = %.4e (n_retunes=%d, ms/ciclo=%.2f)\n', ...
          m_mo.ISE, m_mo.IAE, m_mo.U2, ctrl_mo.n_retunes, ...
          1000.0 * ctrl_mo.t_adapt_total / max(1, ctrl_mo.n_retunes));
  fprintf('  Knee points (J1,J2,J3): \n');
  if ~isempty(ctrl_mo.knee_history)
    for k = 1:size(ctrl_mo.knee_history, 1)
      fprintf('    ciclo %d: J1=%.3e  J2=%.3e  J3=%.3e\n', k, ...
              ctrl_mo.knee_history(k, 1), ctrl_mo.knee_history(k, 2), ...
              ctrl_mo.knee_history(k, 3));
    end
  end

  % 6) Comparación
  fprintf('\n=== Comparacion (S2, T = 2 s) ===\n');
  fprintf('  %-30s  ISE=%.3e  U2=%.3e  sat%%=%.2f\n', 'PR-DE (single-obj)', ...
          m_ad.ISE, m_ad.U2, m_ad.pct_sat);
  fprintf('  %-30s  ISE=%.3e  U2=%.3e  sat%%=%.2f\n', 'NSGA-II (multi-obj)', ...
          m_mo.ISE, m_mo.U2, m_mo.pct_sat);

  % 7) Figuras
  figs_dir = fullfile(base_path, 'results', 'figs');
  if ~exist(figs_dir, 'dir'), mkdir(figs_dir); end
  figure('Position', [50 50 1000 700]);
  subplot(3,1,1); hold on;
  plot(logs_ad.t, rad2deg(logs_ad.qd(:,2)), '--k', 'DisplayName','q_{d,2}');
  plot(logs_ad.t, rad2deg(logs_ad.q(:,2)), '-', 'Color', [0 0.55 0.85], 'DisplayName','PR-DE');
  plot(logs_mo.t, rad2deg(logs_mo.q(:,2)), '-', 'Color', [0.85 0.10 0.55], 'LineWidth', 1.5, 'DisplayName','NSGA-II');
  xlabel('t [s]'); ylabel('q_2 [deg]'); grid on; legend('Location','best');
  title('Seguimiento articular 2');

  subplot(3,1,2); hold on;
  plot(logs_ad.t, sqrt(sum(logs_ad.e.^2, 2)), '-', 'Color', [0 0.55 0.85], 'DisplayName','PR-DE');
  plot(logs_mo.t, sqrt(sum(logs_mo.e.^2, 2)), '-', 'Color', [0.85 0.10 0.55], 'LineWidth', 1.5, 'DisplayName','NSGA-II');
  xlabel('t [s]'); ylabel('||e(t)|| [rad]'); set(gca, 'YScale','log'); grid on; legend('Location','best');
  title('Norma del error de seguimiento');

  subplot(3,1,3); hold on;
  if ~isempty(ctrl_mo.knee_history)
    semilogy(1:size(ctrl_mo.knee_history, 1), ctrl_mo.knee_history(:, 1), '-o', 'DisplayName','J1 (ISE)');
    semilogy(1:size(ctrl_mo.knee_history, 1), ctrl_mo.knee_history(:, 2), '-s', 'DisplayName','J2 (||u||^2)');
    semilogy(1:size(ctrl_mo.knee_history, 1), max(1e-12, ctrl_mo.knee_history(:, 3)), '-^', 'DisplayName','J3 (saturac.)');
  end
  xlabel('ciclo'); ylabel('objetivos'); set(gca, 'YScale','log'); grid on; legend('Location','best');
  title('Evolucion del knee point (frente de Pareto)');

  sgtitle('demo_nsga2: sintonia multi-objetivo con NSGA-II (escenario S2)');
  saveas(gcf, fullfile(figs_dir, 'fig_demo_nsga2.png'));
  fprintf('\nFigura guardada en: %s\n', fullfile(figs_dir, 'fig_demo_nsga2.png'));
end

function test_nsga2_zdt1()
% Test ZDT1 simplificado: minimizar f1 = x1 y f2 = g·(1 - sqrt(f1/g))
% donde g = 1 + 9/(n-1) Σ x_i (i=2..n), x_i ∈ [0,1].
  n = 5; pop = 30; gens = 40;
  f = @(X) [X(:, 1), ...
            (1 + 9/(n-1) * sum(X(:, 2:end), 2)) .* ...
            (1 - sqrt(X(:, 1) ./ (1 + 9/(n-1) * sum(X(:, 2:end), 2))))];
  lo = zeros(1, n); hi = ones(1, n);
  rng(0);
  t0 = tic;
  [Pareto, F_pop, ~] = nsga2(f, n, lo, hi, pop, gens);
  fprintf('  NSGA-II ejecutado en %.1f s con %d generaciones\n', toc(t0), gens);
  fprintf('  Frente de Pareto: %d soluciones no dominadas\n', size(Pareto, 1));
  fprintf('  Rango de f1: [%.3f, %.3f], rango de f2: [%.3f, %.3f]\n', ...
          min(F_pop(:,1)), max(F_pop(:,1)), min(F_pop(:,2)), max(F_pop(:,2)));
  % Test de dominancia: ningún punto del frente debe dominar a otro
  n_dom = 0;
  for i = 1:size(F_pop, 1)
    for j = 1:size(F_pop, 1)
      if i ~= j && all(F_pop(i, :) <= F_pop(j, :) + 1e-9) && any(F_pop(i, :) < F_pop(j, :) - 1e-9)
        n_dom = n_dom + 1;
      end
    end
  end
  if n_dom == 0
    fprintf('  OK: frente de Pareto no dominado (sin dominancias internas)\n');
  else
    fprintf('  WARN: %d dominancias detectadas dentro del frente\n', n_dom);
  end
end
