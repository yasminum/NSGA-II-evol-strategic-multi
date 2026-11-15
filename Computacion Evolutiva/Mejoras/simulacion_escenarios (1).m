%% simulacion_escenarios.m  —  Simulacion rapida de los 4 escenarios S1-S4.
% ------------------------------------------------------------------
% Script autonomo (~30 s) que corre el brazo RRR con CTC-PID en los
% 4 escenarios del proyecto:
%
%   S1 Nominal                 — planta sin perturbaciones
%   S2 +Incertidumbre           — m2 +10%, friccion x1.5, payload 60 g
%   S3 +Incertidumbre+Perturb.   — S2 + perturbacion senoidal externa
%   S4 +Incertidumbre+Perturb.+Ruido — S3 + ruido de medicion ±1.5%
%
% Para cada escenario, ejecuta CTC-PID y un control HIBRIDO (CTC+MPC+NN)
% y compara los resultados. Genera una tabla resumen con los ISE.
%
% Uso:
%   >> simulacion_escenarios
% ------------------------------------------------------------------
function simulacion_escenarios()
  clc;
  fprintf('=========================================================\n');
  fprintf('  SIMULACION DE LOS 4 ESCENARIOS (S1, S2, S3, S4)\n');
  fprintf('  Brazo RRR 3-GDL | CTC-PID vs. Hibrido (CTC+MPC+NN)\n');
  fprintf('=========================================================\n\n');

  % Configuracion
  p = arm_params();
  tr = default_trajectory();
  DT = 0.005; T = 2.5;   % simulacion corta por escenario (rapido)
  N_sub = 3;
  base_path = fileparts(mfilename('fullpath'));
  [theta0, ~] = ensure_offline(false, base_path);
  SC = scenarios_table();

  escenarios = {'S1', 'S2', 'S3', 'S4'};
  metodos = {'CTC', 'Hibrido'};

  % Tabla de resultados
  resultados = struct();
  fprintf('Configuracion: T=%.1f s, dt=%.0f ms, subpasos RK4=%d\n', T, DT*1000, N_sub);
  fprintf('Brazo: l1=%.3f, l2=%.3f [m], m1=%.3f, m2=%.3f [kg], tau_max=[%.1f %.1f %.1f]\n\n', ...
          p.a2, p.a3, p.m1, p.m2, p.tau_max);

  fprintf('%-12s %-20s %-12s %-12s %-12s %-12s\n', ...
          'Escenario', 'Metodo', 'ISE', 'IAE', 'U2', 'sat%');
  fprintf(repmat('-', 1, 80)); fprintf('\n');

  for s = 1:numel(escenarios)
    sc = SC.(escenarios{s});
    p_plant = arm_with_uncertainty(p, sc.m2f, sc.m3f, sc.payload, ...
                                    sc.vf, sc.coulomb);
    if sc.dist
      dist = make_disturbance();
    else
      dist = [];
    end
    if isempty(sc.noise)
      noise = struct('q', 0, 'v', 0);
    else
      noise = sc.noise;
    end

    for m_idx = 1:numel(metodos)
      metodo = metodos{m_idx};
      switch metodo
        case 'CTC'
          ctrl = CTC_PID(p, tr, DT, theta0(1:3), theta0(4:6), theta0(7:9));
        case 'Hibrido'
          ctrl = HybridRobust(p, tr, DT, theta0(1:3), theta0(4:6), theta0(7:9), ...
                                5, 3, 0.1, 16, 0.1);
      end
      logs = simulate(p_plant, ctrl, tr, T, DT, 10001 + s*7, noise, dist, N_sub);
      m = metrics_from_logs(logs, DT);
      resultados.(escenarios{s}).(strrep(metodo, ' ', '_')) = m;
      resultados.(escenarios{s}).(strrep(metodo, ' ', '_')).logs = logs;
      fprintf('%-12s %-20s %-12.3e %-12.3e %-12.3e %-12.2f\n', ...
              escenarios{s}, metodo, m.ISE, m.IAE, m.U2, m.pct_sat);
    end
    fprintf(repmat('-', 1, 80)); fprintf('\n');
  end

  % Resumen: mejora del Hibrido sobre el CTC
  fprintf('\nResumen de mejora (Hibrido vs CTC):\n');
  for s = 1:numel(escenarios)
    sc_key = escenarios{s};
    isectc = resultados.(sc_key).CTC.ISE;
    isehyb = resultados.(sc_key).Hibrido.ISE;
    if isehyb < 1e-12
      mejora = inf;
    else
      mejora = isectc / isehyb;
    end
    fprintf('  %-12s: CTC=%.3e  Hibrido=%.3e  mejora=%.2fx\n', ...
            sc_key, isectc, isehyb, mejora);
  end

  % Figuras: comparacion
  figs_dir = fullfile(base_path, 'results', 'figs');
  if ~exist(figs_dir, 'dir'), mkdir(figs_dir); end

  fig = figure('Name', 'Comparacion escenarios S1-S4', ...
                'Position', [50 50 1200 700], 'Color', 'w');
  for s = 1:numel(escenarios)
    sc_key = escenarios{s};
    subplot(2, 2, s); hold on; grid on;
    logs_ctc = resultados.(sc_key).CTC.logs;
    logs_hyb = resultados.(sc_key).Hibrido.logs;
    plot(logs_ctc.t, sqrt(sum(logs_ctc.e.^2, 2)), '-', 'Color', [0.6 0.6 0.6], ...
          'LineWidth', 1.4, 'DisplayName', 'CTC fijo');
    plot(logs_hyb.t, sqrt(sum(logs_hyb.e.^2, 2)), '-', 'Color', [0.85 0.10 0.55], ...
          'LineWidth', 1.6, 'DisplayName', 'Hibrido');
    xlabel('t [s]'); ylabel('||e(t)|| [rad]');
    title(sprintf('Escenario %s: error de seguimiento', sc_key));
    set(gca, 'YScale', 'log'); legend('Location', 'northeast');
  end
  sgtitle('Simulacion rapida: CTC fijo vs. Hibrido en escenarios S1-S4');
  saveas(fig, fullfile(figs_dir, 'fig_simulacion_escenarios.png'));
  fprintf('\nFigura guardada en: %s\n', fullfile(figs_dir, 'fig_simulacion_escenarios.png'));
  fprintf('\n=== Simulacion completada ===\n');
end
