%% experiment_hybrid.m  —  Experimento completo del proyecto extendido.
% ------------------------------------------------------------------
% Compara 7 controladores sobre los 4 escenarios del proyecto base
% (S1-S4) con un subconjunto reducido de corridas (default: 5 por
% combinación → 140 corridas; con 'runs', 30 → 840 corridas).
%
% Métodos comparados:
%   1) CTC                       — baseline de torque calculado con PID.
%   2) PR-DE                     — AdaptiveCTC con regresor polinomial + DE.
%   3) PR-CMA                    — AdaptiveCTC con regresor polinomial + CMA-ES.
%   4) NN-DE                     — NNAdaptiveCTC (red neuronal) + DE.
%   5) MPC                       — Control predictivo puro.
%   6) HYBRID                    — CTC + MPC + NN residual.
%   7) SA-NSGA2                  — SurrogateAdaptiveCTC (RBF + NSGA-II).
%
% El protocolo estadístico (semillas 1..runs, números aleatorios
% comunes entre alternativas) preserva la validez de las comparaciones
% cuando el ruido de medición está presente (S4).
%
% Uso:
%   >> experiment_hybrid                       % versión corta: 5 runs
%   >> experiment_hybrid('runs', 30)            % versión completa
%   >> experiment_hybrid('smoke')                % prueba mínima
%   >> experiment_hybrid('methods', {'CTC','HYBRID'})   % subconjunto
%
% Salidas:
%   results/hybrid/results.csv      — métricas por corrida.
%   results/hybrid/summary.csv       — medias ± std por (método, escenario).
%   results/hybrid/figs/*.png       — 6 figuras comparativas.
%   results/hybrid/stats.mat        — versión binaria para análisis posterior.
% ------------------------------------------------------------------
function experiment_hybrid(varargin)
  % Parseo de argumentos
  runs = 5; smoke = false; methods = {};
  base_path = fileparts(mfilename('fullpath'));
  for k = 1:2:numel(varargin)
    switch lower(varargin{k})
      case 'runs',    runs = varargin{k+1};
      case 'smoke',  smoke = true; runs = 1;
      case 'methods', methods = varargin{k+1};
      case 'base_path', base_path = varargin{k+1};
    end
  end
  if isempty(methods)
    methods = {'CTC', 'PR-DE', 'PR-CMA', 'NN-DE', 'MPC', 'HYBRID', 'SA-NSGA2'};
  end
  scenarios = {'S1', 'S2', 'S3', 'S4'};

  fprintf('=== experiment_hybrid ===\n');
  fprintf('Controladores : %s\n', strjoin(methods, ', '));
  fprintf('Escenarios    : %s\n', strjoin(scenarios, ', '));
  fprintf('Corridas/comb : %d\n', runs);
  fprintf('Total corridas: %d\n', numel(methods) * numel(scenarios) * runs);

  % Cargar ganancias offline
  [theta0, ~] = ensure_offline(false, base_path);
  fprintf('\ntheta0 (CTC offline) cargado.\n\n');

  out_dir = fullfile(base_path, 'results', 'hybrid');
  figs_dir = fullfile(out_dir, 'figs');
  if ~exist(out_dir, 'dir'),  mkdir(out_dir); end
  if ~exist(figs_dir, 'dir'), mkdir(figs_dir); end

  % Lista de tareas
  tasks = {};
  for s = 1:numel(scenarios)
    for m = 1:numel(methods)
      for r = 1:runs
        tasks{end+1} = struct('method', methods{m}, ...
                              'scen', scenarios{s}, ...
                              'seed', r, 'theta0', theta0, 'T', 2.0); %#ok<AGROW>
      end
    end
  end
  n_tasks = numel(tasks);
  fprintf('Iniciando %d tareas...\n\n', n_tasks);
  t_start = tic;

  % Cabecera del CSV
  csv_header = {'scenario', 'method', 'seed', ...
                'ISE', 'IAE', 'ISE_v', 'U2', 'u_max', 'e_max', 'pct_sat', ...
                'T_adapt', 'ms_retune'};
  results_table = cell(n_tasks, numel(csv_header));

  for i = 1:n_tasks
    t_task = tic;
    m = run_hybrid_case(tasks{i}, base_path);
    for col = 1:numel(csv_header)
      results_table{i, col} = m.(csv_header{col});
    end
    elapsed = toc(t_task);
    fprintf('  [%3d/%3d] %-12s | %-8s | seed=%2d | ISE=%.3e | %.1fs\n', ...
            i, n_tasks, tasks{i}.method, tasks{i}.scen, tasks{i}.seed, ...
            m.ISE, elapsed);
  end

  % Guardar CSV
  csv_path = fullfile(out_dir, 'results.csv');
  fid = fopen(csv_path, 'w');
  fprintf(fid, '%s\n', strjoin(csv_header, ','));
  for i = 1:n_tasks
    row = results_table(i, :);
    line = sprintf('%s,%s,%d', row{1}, row{2}, row{3});
    for c = 4:numel(csv_header)
      line = [line, ',', sprintf('%.6e', row{c})]; %#ok<AGROW>
    end
    fprintf(fid, '%s\n', line);
  end
  fclose(fid);
  fprintf('\nResultados guardados en: %s\n', csv_path);

  % Resumen: medias por (método, escenario)
  summary = compute_summary(results_table, csv_header, methods, scenarios);
  summary_path = fullfile(out_dir, 'summary.csv');
  fid = fopen(summary_path, 'w');
  fprintf(fid, 'method,scenario,ISE_mean,ISE_std,U2_mean,U2_std,pct_sat_mean\n');
  for s = 1:numel(scenarios)
    for m = 1:numel(methods)
      key = {methods{m}, scenarios{s}};
      idx = cellfun(@(c) strcmp(c{1}, key{1}) && strcmp(c{2}, key{2}), ...
                    num2cell(results_table(:, 1:2)));
      if any(idx)
        ise_vals = [results_table{idx, 4}];
        u2_vals  = [results_table{idx, 8}];
        sat_vals = [results_table{idx, 10}];
        fprintf(fid, '%s,%s,%.6e,%.6e,%.6e,%.6e,%.6e\n', ...
                methods{m}, scenarios{s}, ...
                mean(ise_vals), std(ise_vals), ...
                mean(u2_vals), std(u2_vals), ...
                mean(sat_vals));
      end
    end
  end
  fclose(fid);
  fprintf('Resumen guardado en: %s\n\n', summary_path);

  % Guardar .mat
  save(fullfile(out_dir, 'stats.mat'), 'results_table', 'csv_header', ...
       'methods', 'scenarios', 'runs');

  % Generar figuras
  make_hybrid_figures(results_table, csv_header, methods, scenarios, figs_dir);

  fprintf('=== experiment_hybrid terminado en %.1f min ===\n', toc(t_start)/60);
end

function m = run_hybrid_case(task, base_path)
% Construye planta + controlador y dispara una corrida.
  method = task.method;
  scen   = task.scen;
  seed   = task.seed;
  theta0 = task.theta0;
  T      = task.T;
  DT     = 0.005;
  S_B    = 10;
  SCENARIOS = scenarios_table();
  sc = SCENARIOS.(scen);
  p_nom   = arm_params();
  p_plant = arm_with_uncertainty(p_nom, sc.m2f, sc.m3f, sc.payload, ...
                                  sc.vf, sc.coulomb);
  traj = default_trajectory();
  if sc.dist
    dist = make_disturbance();
  else
    dist = [];
  end
  [lo, hi] = online_bounds(theta0);

  switch method
    case 'CTC'
      ctrl = CTC_PID(p_nom, traj, DT, theta0(1:3), theta0(4:6), theta0(7:9));
    case 'PR-DE'
      ctrl = AdaptiveCTC(p_nom, traj, DT, theta0(1:3), theta0(4:6), theta0(7:9), ...
                         lo, hi, 'DE', 2, S_B, 1, 20, 20, seed);
    case 'PR-CMA'
      ctrl = AdaptiveCTC(p_nom, traj, DT, theta0(1:3), theta0(4:6), theta0(7:9), ...
                         lo, hi, 'CMA', 2, S_B, 1, 20, 20, seed);
    case 'NN-DE'
      ctrl = NNAdaptiveCTC(p_nom, traj, DT, theta0(1:3), theta0(4:6), theta0(7:9), ...
                            lo, hi, 'DE', S_B, 1, 20, 20, seed);
    case 'MPC'
      ctrl = MPC(p_nom, traj, DT, 5, 3, 0.1, 0.4*p_nom.tau_max, 10);
    case 'HYBRID'
      ctrl = HybridRobust(p_nom, traj, DT, theta0(1:3), theta0(4:6), theta0(7:9), ...
                            5, 3, 0.1, 16, 0.1);
    case 'SA-NSGA2'
      ctrl = SurrogateAdaptiveCTC(p_nom, traj, DT, theta0(1:3), theta0(4:6), theta0(7:9), ...
                                    lo, hi, 2, S_B, 1, 20, 10, seed, 'rbf', 4, 10);
    otherwise
      error('Método desconocido: %s', method);
  end

  sim_seed = 10000 + seed;
  if isempty(sc.noise)
    noise = struct('q', 0, 'v', 0);
  else
    noise = sc.noise;
  end
  logs = simulate(p_plant, ctrl, traj, T, DT, sim_seed, noise, dist, 5);
  m = metrics_from_logs(logs, DT);
  m.T_adapt = 0;
  if isprop(ctrl, 't_adapt_total')
    m.T_adapt = ctrl.t_adapt_total;
  end
  if isprop(ctrl, 'n_retunes') && ctrl.n_retunes > 0
    m.ms_retune = 1000.0 * ctrl.t_adapt_total / ctrl.n_retunes;
  else
    m.ms_retune = 0.0;
  end
  m.scenario  = scen;
  m.method   = method;
  m.seed     = seed;
end

function make_hybrid_figures(results_table, csv_header, methods, scenarios, figs_dir)
% Genera 4 figuras resumen:
%   1) ISE por método y escenario (barras).
%   2) Esfuerzo U2 por método y escenario (barras).
%   3) Tiempo de cómputo (T_adapt) por método.
%   4) Tiempo por ciclo (ms_retune) por método.
  fig = figure('Position', [50 50 1200 800], 'Visible', 'off');
  cmap = lines(numel(methods));

  % Fig 1: ISE por (método, escenario) — barras agrupadas
  subplot(2,2,1);
  ise_means = zeros(numel(methods), numel(scenarios));
  ise_stds  = zeros(numel(methods), numel(scenarios));
  for s = 1:numel(scenarios)
    for m = 1:numel(methods)
      idx = find(strcmp(results_table(:, 1), scenarios{s}) & ...
                 strcmp(results_table(:, 2), methods{m}));
      if ~isempty(idx)
        vals = [results_table{idx, 4}];
        ise_means(m, s) = mean(vals);
        ise_stds(m, s)  = std(vals);
      end
    end
  end
  b = bar(ise_means, 'grouped');
  set(gca, 'XTickLabel', methods, 'XTickLabelRotation', 30);
  legend(scenarios, 'Location', 'best');
  title('ISE medio por método y escenario'); ylabel('ISE'); grid on;
  set(b, 'FaceColor', 'flat');
  for k = 1:numel(b), b(k).FaceColor = cmap(k, :); end

  % Fig 2: U2 por (método, escenario) — barras agrupadas
  subplot(2,2,2);
  u2_means = zeros(numel(methods), numel(scenarios));
  for s = 1:numel(scenarios)
    for m = 1:numel(methods)
      idx = find(strcmp(results_table(:, 1), scenarios{s}) & ...
                 strcmp(results_table(:, 2), methods{m}));
      if ~isempty(idx)
        vals = [results_table{idx, 8}];
        u2_means(m, s) = mean(vals);
      end
    end
  end
  b = bar(u2_means, 'grouped');
  set(gca, 'XTickLabel', methods, 'XTickLabelRotation', 30);
  legend(scenarios, 'Location', 'best');
  title('Esfuerzo U_2 medio por método y escenario'); ylabel('U_2'); grid on;
  set(b, 'FaceColor', 'flat');
  for k = 1:numel(b), b(k).FaceColor = cmap(k, :); end

  % Fig 3: tiempo de cómputo total (T_adapt)
  subplot(2,2,3);
  t_means = zeros(numel(methods), 1);
  for m = 1:numel(methods)
    idx = find(strcmp(results_table(:, 2), methods{m}));
    if ~isempty(idx)
      vals = [results_table{idx, 11}];
      t_means(m, 1) = mean(vals);
    end
  end
  bar(t_means, 'FaceColor', 'flat', 'CData', cmap(1:numel(methods), :));
  set(gca, 'XTickLabel', methods, 'XTickLabelRotation', 30);
  title('Tiempo total de adaptación por método'); ylabel('T_{adapt} [s]'); grid on;

  % Fig 4: tiempo por ciclo (ms_retune)
  subplot(2,2,4);
  ms_means = zeros(numel(methods), 1);
  for m = 1:numel(methods)
    idx = find(strcmp(results_table(:, 2), methods{m}));
    if ~isempty(idx)
      vals = [results_table{idx, 12}];
      ms_means(m, 1) = mean(vals);
    end
  end
  bar(ms_means, 'FaceColor', 'flat', 'CData', cmap(1:numel(methods), :));
  set(gca, 'XTickLabel', methods, 'XTickLabelRotation', 30);
  title('Tiempo medio por retune por método'); ylabel('ms/retune'); grid on;

  sgtitle('experiment_hybrid: comparativa de 7 controladores (4 escenarios)');
  saveas(fig, fullfile(figs_dir, 'fig_experiment_hybrid.png'));
  fprintf('Figura resumen guardada en: %s\n', ...
          fullfile(figs_dir, 'fig_experiment_hybrid.png'));
  close(fig);
end
