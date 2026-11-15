%% experiment.m  —  Protocolo experimental completo (entrada principal).
% ------------------------------------------------------------------
% Diseño: 6 alternativas de control × 4 escenarios × 30 corridas
% independientes (semillas 1..30; mismos números aleatorios de medición
% por semilla para todas las alternativas — common random numbers).
%
% Alternativas:
%   CTC      CTC-PID con ganancias fijas θ0 (sintonizadas offline con ODE)
%   SLR-DE   control adaptativo con regresor LINEAL (grado 1) + ODE
%   PR-DE    control adaptativo con REGRESOR POLINOMIAL grado 2 + ODE
%            (propuesta principal)
%   PR-GA / PR-PSO / PR-CMA  mismo regresor polinomial, optimizador
%            distinto con presupuesto equivalente (400 evaluac./ciclo)
%
% Escenarios:
%   S1 Nominal | S2 +Incertidumbre paramétrica (carga útil 60 g, fricción
%   ×1.5, Coulomb no modelado, m2 +10 %) | S3 +Perturbación externa
%   (senoidal + escalón) | S4 +Ruido de medición (±1 % a escala completa).
%
% Métricas: ISE (principal), IAE, ISE_v, ∫‖τ‖²dt, τ_max, % saturación y
% tiempo de cómputo de la adaptación (viabilidad en tiempo real: ms por
% ciclo de 5 ms).
%
% Análisis estadístico (protocolo de Rodríguez-Molina et al., 2024):
%   test de Friedman por escenario + rangos medios + W de Kendall +
%   comparaciones por pares Wilcoxon vs. la mejor alternativa con
%   corrección Holm.
%
% Uso (en MATLAB u Octave):
%   >> experiment              % 30 corridas, 6×4 alternativas
%   >> experiment('smoke')     % prueba rápida (1 corrida, S2, T = 2 s)
%   >> experiment('runs', 5)   % sólo 5 corridas para depurar
%
% Salidas en results/:
%   results.csv          métricas de cada corrida
%   summary.csv          medias/desvíos por escenario × método
%   stats.json/.mat      análisis estadístico Friedman + Wilcoxon-Holm
%   budget.csv           presupuesto computacional por método
%   traces/{S}_{method}.mat   trazas de la semilla 11
%   figs/                9 figuras PNG (fig_brazo3d, fig_trayectoria, etc.)
%
% Notas:
%   * La ejecución COMPLETA (6×4×30 = 720 sims) toma ~25-50 min en
%     MATLAB u Octave en un portátil moderno (sin paralelización).
%   * El experimento es REANUDABLE: si se interrumpe, los resultados
%     parciales quedan en results/partial.jsonl y se reanudan.
% ------------------------------------------------------------------
function experiment(varargin)
  args = parse_args(varargin{:});

  base_path = fileparts(mfilename('fullpath'));
  if isempty(base_path), base_path = pwd; end
  results_dir = fullfile(base_path, 'results');
  traces_dir  = fullfile(results_dir, 'traces');
  figs_dir    = fullfile(results_dir, 'figs');
  if ~exist(results_dir, 'dir'), mkdir(results_dir); end
  if ~exist(traces_dir,  'dir'), mkdir(traces_dir);  end
  if ~exist(figs_dir,    'dir'), mkdir(figs_dir);    end

  METHODS = {'CTC', 'SLR-DE', 'PR-DE', 'PR-GA', 'PR-PSO', 'PR-CMA'};
  SCENARIOS = {'S1', 'S2', 'S3', 'S4'};

  if args.smoke
    methods   = setdiff(METHODS, 'CTC');
    if isempty(methods), methods = {'PR-DE'}; end
    scen_keys = {'S2'};
    runs      = 1;
    T_sim     = 2.0;
  else
    methods   = METHODS;
    scen_keys = SCENARIOS;
    runs      = args.runs;
    T_sim     = 6.0;
  end

  fprintf('=== Experimento CE brazo 3 GDL ===\n');
  fprintf('  métodos:    %s\n', strjoin(methods, ', '));
  fprintf('  escenarios: %s\n', strjoin(scen_keys, ', '));
  fprintf('  corridas:   %d por caso\n', runs);

  % ---- ganancias offline θ0 -----------------------------------
  [theta0, iseo] = ensure_offline(args.offline_force, base_path);
  fprintf('  theta0 (offline): Kp=[%.1f %.1f %.1f] Ki=[%.1f %.1f %.1f] '...
          'Kd=[%.1f %.1f %.1f]  ISE_offline=%.3e\n', ...
          theta0(1), theta0(2), theta0(3), ...
          theta0(4), theta0(5), theta0(6), ...
          theta0(7), theta0(8), theta0(9), iseo);

  % ---- construir lista de tareas -------------------------------
  all_tasks = {};
  for s_idx = 1:numel(scen_keys)
    s = scen_keys{s_idx};
    for m_idx = 1:numel(methods)
      m = methods{m_idx};
      for sd = 1:runs
        all_tasks{end+1} = struct('method', m, 'scen', s, ...
                                  'seed', sd, 'theta0', theta0, ...
                                  'T', T_sim); %#ok<AGROW>
      end
    end
  end

  % ---- reanudación: cargar resultados parciales ----------------
  partial_path = fullfile(results_dir, 'partial.jsonl');
  done = containers.Map();
  results = {};
  if exist(partial_path, 'file') && ~args.smoke
    lines = read_cell_lines(partial_path);
    for i = 1:numel(lines)
      if isempty(lines{i}), continue; end
      try
        r = jsondecode(lines{i});
        key = sprintf('%s|%s|%d', r.scenario, r.method, r.seed);
        results{end+1} = r; %#ok<AGROW>
        done(key) = true;
      catch
      end
    end
    fprintf('Reanudando: %d corridas ya completadas\n', numel(results));
  end
  tasks_to_run = {};
  for i = 1:numel(all_tasks)
    t = all_tasks{i};
    key = sprintf('%s|%s|%d', t.scen, t.method, t.seed);
    if ~isKey(done, key)
      tasks_to_run{end+1} = t; %#ok<AGROW>
    end
  end
  fprintf('Ejecutando %d corridas...\n', numel(tasks_to_run));

  % ---- bucle de ejecución --------------------------------------
  t0 = tic;
  fout = [];
  if ~args.smoke
    fout = fopen(partial_path, 'a');
  end
  for i = 1:numel(tasks_to_run)
    t = tasks_to_run{i};
    r = run_case(t, base_path);
    results{end+1} = r; %#ok<AGROW>
    if ~isempty(fout)
      fprintf(fout, '%s\n', jsonencode(r));
      fflush(fout);
    end
    if mod(i, 10) == 0 || i == numel(tasks_to_run)
      el = toc(t0);
      eta = el / i * (numel(tasks_to_run) - i);
      fprintf('  [%4d/%d]  %6.1f s   ETA %6.1f s\n', i, numel(tasks_to_run), ...
              el, eta);
    end
    if ~isempty(args.max_seconds) && toc(t0) > args.max_seconds ...
        && i < numel(tasks_to_run)
      fprintf('  Limite de %.0f s alcanzado tras %d corridas; se reanudara.\n', ...
              args.max_seconds, i);
      if ~isempty(fout), fclose(fout); end
      return;
    end
  end
  if ~isempty(fout), fclose(fout); end

  if args.smoke && exist(partial_path, 'file'), delete(partial_path); end

  % ---- CSV de resultados ---------------------------------------
  cols = {'scenario','method','seed','ISE','IAE','ISE_v','U2', ...
          'u_max','e_max','pct_sat','T_adapt','ms_retune'};
  fid = fopen(fullfile(results_dir, 'results.csv'), 'w');
  fprintf(fid, '%s\n', strjoin(cols, ','));
  for i = 1:numel(results)
    r = results{i};
    fprintf(fid, '%s,%s,%d,%.10e,%.10e,%.10e,%.10e,%.6e,%.6e,%.4f,%.4f,%.4f\n', ...
            r.scenario, r.method, r.seed, r.ISE, r.IAE, r.ISE_v, r.U2, ...
            r.u_max, r.e_max, r.pct_sat, r.T_adapt, r.ms_retune);
  end
  fclose(fid);

  % ---- resumen por escenario × método ---------------------------
  fid = fopen(fullfile(results_dir, 'summary.csv'), 'w');
  fprintf(fid, 'scenario,method,ISE_mean,ISE_std,ISE_min,ISE_max,U2_mean,T_adapt_mean,ms_retune_mean\n');
  for s_idx = 1:numel(scen_keys)
    s = scen_keys{s_idx};
    for m_idx = 1:numel(methods)
      m = methods{m_idx};
      ise = [];
      u2 = []; tad = []; msr = [];
      for i = 1:numel(results)
        r = results{i};
        if strcmp(r.scenario, s) && strcmp(r.method, m)
          ise(end+1) = r.ISE; %#ok<AGROW>
          u2(end+1)  = r.U2;  %#ok<AGROW>
          tad(end+1) = r.T_adapt; %#ok<AGROW>
          msr(end+1) = r.ms_retune; %#ok<AGROW>
        end
      end
      if isempty(ise), continue; end
      fprintf(fid, '%s,%s,%.6e,%.3e,%.6e,%.6e,%.4e,%.3f,%.3f\n', ...
              s, m, mean(ise), std(ise), min(ise), max(ise), ...
              mean(u2), mean(tad), mean(msr));
    end
  end
  fclose(fid);

  % ---- estadística Friedman + Wilcoxon-Holm --------------------
  stats_all = struct();
  fprintf('\n%s\n', repmat('=', 1, 78));
  for s_idx = 1:numel(scen_keys)
    s = scen_keys{s_idx};
    mat = zeros(runs, numel(methods));
    for m_idx = 1:numel(methods)
      m = methods{m_idx};
      vals = [];
      for i = 1:numel(results)
        r = results{i};
        if strcmp(r.scenario, s) && strcmp(r.method, m)
          vals(end+1) = r.ISE; %#ok<AGROW>
        end
      end
      mat(:, m_idx) = vals(:);
    end
    if runs < 2, continue; end
    st = friedman_analysis(mat, methods, 0.05);
    stats_all.(s) = st;
    SC = scenarios_table();
    fprintf('\nEscenario %s — %s\n', s, SC.(s).nombre);
    fprintf('  Friedman chi2(%d) = %.2f, p = %.3e, W de Kendall = %.3f\n', ...
            numel(methods)-1, st.chi2, st.p, st.W);
    for m_idx = 1:numel(methods)
      mark = '';
      if strcmp(methods{m_idx}, st.control), mark = ' <--- control'; end
      fprintf('    %-14s rango medio = %5.2f%s\n', methods{m_idx}, ...
              st.mean_ranks(m_idx), mark);
    end
    fprintf('%s\n', holm_table(st.pairwise));
  end
  fprintf('\n%s\n', repmat('=', 1, 78));

  save(fullfile(results_dir, 'stats.mat'), '-struct', 'stats_all');
  fid = fopen(fullfile(results_dir, 'stats.json'), 'w');
  fprintf(fid, '%s\n', jsonencode(stats_all));
  fclose(fid);

  % ---- presupuesto computacional -------------------------------
  fid = fopen(fullfile(results_dir, 'budget.csv'), 'w');
  fprintf(fid, 'method,evals_por_ciclo,ms_por_ciclo,T_adapt_s\n');
  for m_idx = 1:numel(methods)
    m = methods{m_idx};
    msr = []; tad = [];
    for i = 1:numel(results)
      r = results{i};
      if strcmp(r.method, m)
        msr(end+1) = r.ms_retune; %#ok<AGROW>
        tad(end+1) = r.T_adapt;   %#ok<AGROW>
      end
    end
    if isempty(msr), continue; end
    fprintf(fid, '%s,%d,%.3f,%.3f\n', m, 20*20, mean(msr), mean(tad));
  end
  fclose(fid);

  % ---- figuras (sólo si hay ≥ 2 corridas) ----------------------
  if runs >= 2 && ~args.skip_figs
    make_figures(scen_keys, methods, runs, base_path);
  end

  fprintf('\nListo en %.1f s. Resultados en: %s\n', toc(t0), results_dir);
end

% -------------------------------------------------------------------------
function args = parse_args(varargin)
  args.smoke = false;
  args.runs = 30;
  args.max_seconds = [];
  args.offline_force = false;
  args.skip_figs = false;
  i = 1;
  while i <= nargin
    switch lower(varargin{i})
      case 'smoke',       args.smoke = true;
      case 'runs',        args.runs = varargin{i+1}; i = i + 1;
      case 'max_seconds', args.max_seconds = varargin{i+1}; i = i + 1;
      case 'offline_force', args.offline_force = true;
      case 'skip_figs',   args.skip_figs = true;
      otherwise, warning('Argumento desconocido: %s', varargin{i});
    end
    i = i + 1;
  end
end

function lines = read_cell_lines(path)
  fid = fopen(path, 'r');
  if fid < 0, lines = {}; return; end
  raw = fread(fid, '*char')';
  fclose(fid);
  lines = strsplit(raw, {'\n', '\r\n'});
end
