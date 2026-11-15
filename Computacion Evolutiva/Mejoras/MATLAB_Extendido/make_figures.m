%% make_figures.m  —  Genera las 9 figuras del informe en results/figs/.
% ------------------------------------------------------------------
% Genera las figuras del informe (paralelas a las del código Python):
%   fig_brazo3d      geometría del brazo + trayectoria del efector
%   fig_trayectoria  seguimiento articular (S3, PR-DE)
%   fig_error        norma del error por método (S3)
%   fig_ganancias    evolución online de las ganancias (S3, PR-DE)
%   fig_convergencia ISE acumulado por método (S3)
%   fig_boxplot      ISE por método y escenario
%   fig_ranks        mapa de calor de rangos medios de Friedman
%   fig_torque       pares de control (S3)
%
% Requiere MATLAB R2014b+ u Octave (la mayoría de gráficos funcionan
% también en Octave; los 3D requieren Octave ≥ 4.0 con OpenGL).
% ------------------------------------------------------------------
function make_figures(scen_keys, methods, runs, base_path)
  if nargin < 4, base_path = '.'; end
  figs_dir   = fullfile(base_path, 'results', 'figs');
  traces_dir = fullfile(base_path, 'results', 'traces');

  LABELS = containers.Map( ...
    {'CTC', 'SLR-DE', 'PR-DE', 'PR-GA', 'PR-PSO', 'PR-CMA'}, ...
    {'CTC fija', 'SLR-DE', 'PR-DE (prop.)', 'PR-GA', 'PR-PSO', 'PR-CMA'});
  COLORS = containers.Map( ...
    {'CTC', 'SLR-DE', 'PR-DE', 'PR-GA', 'PR-PSO', 'PR-CMA'}, ...
    {[0.33 0.33 0.33], [0.90 0.62 0.00], [0.00 0.45 0.70], ...
     [0.00 0.62 0.45], [0.80 0.47 0.59], [0.84 0.37 0.00]});
  DT = 0.005;
  TRACE_SEED = 11;
  SC = scenarios_table();

  % ---- fig_brazo3d ----------------------------------------------
  tr_path = fullfile(traces_dir, 'S3_PR-DE.mat');
  if exist(tr_path, 'file')
    S = load(tr_path);
    tr = S.logs; gh = S.gh;
    p = arm_params();
    pts = zeros(numel(tr.t), 3);
    for k = 1:numel(tr.t)
      P = forward_kinematics(p, tr.q(k,:).');
      pts(k,:) = P{4}.';
    end
    figure('Position', [50 50 640 520]);
    ax = axes; hold(ax, 'on');
    step_idx = max(1, round(linspace(1, numel(tr.t), 7)));
    for k = step_idx
      P = forward_kinematics(p, tr.q(k,:).');
      xs = [P{1}(1), P{2}(1), P{3}(1), P{4}(1)];
      ys = [P{1}(2), P{2}(2), P{3}(2), P{4}(2)];
      zs = [P{1}(3), P{2}(3), P{3}(3), P{4}(3)];
      plot3(xs, ys, zs, '-o', 'Color', [0 0.45 0.70 0.45], ...
            'LineWidth', 1.8, 'MarkerSize', 3);
    end
    plot3(pts(:,1), pts(:,2), pts(:,3), '-', 'Color', [0.84 0.37 0.00], ...
          'LineWidth', 1.4);
    plot3(pts(1,1),   pts(1,2),   pts(1,3),   's', 'Color', [0 0.6 0], 'MarkerSize', 6);
    plot3(pts(end,1), pts(end,2), pts(end,3), '^', 'Color', [0.8 0 0], 'MarkerSize', 6);
    xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
    title({'Brazo RRR de 3 GDL (Conejo Benitez, 2021)', ...
           'Posiciones sucesivas y trayectoria cartesiana del efector'});
    grid on; view(30, 25);
    saveas(gcf, fullfile(figs_dir, 'fig_brazo3d.png')); close(gcf);

    % ---- fig_trayectoria --------------------------------------
    figure('Position', [50 50 720 640]);
    names = {'Art. 1 (base, AX-18A)', 'Art. 2 (AX-12A)', 'Art. 3 (AX-12A)'};
    for j = 1:3
      subplot(3,1,j); hold on;
      plot(tr.t, rad2deg(tr.qd(:,j)), '--', 'Color', [0.33 0.33 0.33], ...
           'LineWidth', 1.2); plot(tr.t, rad2deg(tr.q(:,j)), ...
           'Color', [0 0.45 0.70], 'LineWidth', 1.4);
      ylabel(sprintf('q_{%d} [deg]', j));
      title(names{j}, 'FontSize', 9, 'Interpreter', 'none');
      vline(2.5, ':', [0.53 0.53 0.53]);
      vline(5.0, ':', [0.53 0.53 0.53]);
      vline(4.2, '--', [0.84 0.37 0.00]);
      if j == 1, legend('q_d', 'PR-DE', 'Location', 'southeast'); end
    end
    xlabel('t [s]');
    sgtitle('Seguimiento articular con control adaptativo PR-DE (S3)', ...
            'FontSize', 10);
    saveas(gcf, fullfile(figs_dir, 'fig_trayectoria.png')); close(gcf);
  end

  % ---- fig_error ---------------------------------------------
  figure('Position', [50 50 720 360]); hold on;
  for mi = 1:numel(methods)
    m = methods{mi};
    fp = fullfile(traces_dir, sprintf('S3_%s.mat', m));
    if ~exist(fp, 'file'), continue; end
    S = load(fp); d = S.logs;
    plot(d.t, sqrt(sum(d.e.^2, 2)), 'Color', COLORS(m), ...
         'LineWidth', 1.3, 'DisplayName', LABELS(m));
    hold on;
  end
  vline(4.2, '--', [0.84 0.37 0.00]);
  text(4.25, 0.92*gca.YLim(2), 'Escalon de perturbacion', ...
       'Color', [0.84 0.37 0.00], 'FontSize', 8);
  xlabel('t [s]'); ylabel('||e(t)|| [rad]');
  title('Norma del error de seguimiento (S3)');
  legend('Location', 'southoutside', 'Orientation', 'horizontal', ...
         'NumColumns', 3, 'FontSize', 8);
  grid on;
  saveas(gcf, fullfile(figs_dir, 'fig_error.png')); close(gcf);

  % ---- fig_ganancias -----------------------------------------
  if exist(tr_path, 'file') && size(gh, 1) > 0
    figure('Position', [50 50 720 680]);
    tags = {'K_p', 'K_i', 'K_d'};
    S0 = load(fullfile(base_path, 'results', 'offline_gains.mat'));
    th0 = S0.theta0;
    for j = 1:3
      subplot(3,1,j); hold on;
      for i = 1:3
        plot(gh(:,1), gh(:, 1+(j-1)*3+i), 'Color', [0 0.45 0.70], ...
             'LineWidth', 1.1, 'Alpha', 0.45 + 0.25*i);
        hline(th0((j-1)*3+i), ':', [0.53 0.53 0.53]);
      end
      ylabel(sprintf('%s (art 1-3)', tags{j}));
    end
    xlabel('t [s]');
    sgtitle('Evolucion online de las ganancias CTC-PID con PR-DE (S3). '...
            'Lineas punteadas: valores fijos theta_0', 'FontSize', 10);
    saveas(gcf, fullfile(figs_dir, 'fig_ganancias.png')); close(gcf);
  end

  % ---- fig_convergencia --------------------------------------
  figure('Position', [50 50 720 360]); hold on;
  for mi = 1:numel(methods)
    m = methods{mi};
    fp = fullfile(traces_dir, sprintf('S3_%s.mat', m));
    if ~exist(fp, 'file'), continue; end
    S = load(fp); d = S.logs;
    plot(d.t, DT * cumsum(sum(d.e.^2, 2)), 'Color', COLORS(m), ...
         'LineWidth', 1.4, 'DisplayName', LABELS(m));
    hold on;
  end
  set(gca, 'YScale', 'log');
  xlabel('t [s]'); ylabel('ISE acumulado [rad^2 s]');
  title(sprintf('Convergencia del ISE acumulado (S3, semilla %d)', TRACE_SEED));
  legend('Location', 'northwest', 'NumColumns', 2, 'FontSize', 8);
  grid on;
  saveas(gcf, fullfile(figs_dir, 'fig_convergencia.png')); close(gcf);

  % ---- fig_boxplot -------------------------------------------
  results_csv = fullfile(base_path, 'results', 'results.csv');
  if exist(results_csv, 'file')
    R = read_csv_struct(results_csv);
    figure('Position', [50 50 1150 380]);
    for s_idx = 1:numel(scen_keys)
      s = scen_keys{s_idx};
      subplot(1, numel(scen_keys), s_idx); hold on;
      data = {}; labels = {}; cols = {};
      for mi = 1:numel(methods)
        m = methods{mi};
        vals = [];
        for i = 1:numel(R)
          if strcmp(R{i}.scenario, s) && strcmp(R{i}.method, m)
            vals(end+1) = str2double(R{i}.ISE); %#ok<AGROW>
          end
        end
        if isempty(vals), continue; end
        data{end+1} = vals; %#ok<AGROW>
        labels{end+1} = LABELS(m); %#ok<AGROW>
        cols{end+1} = COLORS(m); %#ok<AGROW>
      end
      if isempty(data), continue; end
      bh = boxplot(cell2mat(data')', 'Orientation', 'horizontal');
      % bh es una matriz de handles: fila 5 = cajas (boxes).
      % Si boxplot no devolvió handles válidos, se ignora el coloreo.
      if ~isempty(bh) && isnumeric(bh) && size(bh,1) >= 5
        for b = 1:numel(data)
          set(bh(5, b), 'Color', cols{b}, 'FaceColor', cols{b}, ...
              'FaceAlpha', 0.55);
        end
      end
      set(gca, 'YTickLabel', labels);
      title(sprintf('%s: %s', s, SC.(s).nombre), 'FontSize', 9);
      ylabel('');
      if s_idx == 1, ylabel('ISE [rad^2 s]'); end
    end
    sgtitle(sprintf('Distribucion del ISE por alternativa y escenario (%d corridas)', runs), ...
            'FontSize', 10);
    saveas(gcf, fullfile(figs_dir, 'fig_boxplot.png')); close(gcf);
  end

  % ---- fig_ranks ---------------------------------------------
  stats_path = fullfile(base_path, 'results', 'stats.mat');
  if exist(stats_path, 'file')
    ST = load(stats_path);
    scs = scen_keys;
    M = zeros(numel(scs), numel(methods));
    for i = 1:numel(scs)
      s = scs{i};
      M(i,:) = ST.(s).mean_ranks;
    end
    figure('Position', [50 50 760 340]);
    imagesc(M); colorbar; colormap(flipud(parula));
    set(gca, 'XTick', 1:numel(methods), 'XTickLabel', ...
        cellfun(@(m) LABELS(m), methods, 'UniformOutput', false));
    set(gca, 'YTick', 1:numel(scs), 'YTickLabel', ...
        cellfun(@(s) sprintf('%s: %s', s, SC.(s).nombre), scs, ...
                'UniformOutput', false));
    for i = 1:size(M,1)
      for j = 1:size(M,2)
        text(j, i, sprintf('%.2f', M(i,j)), 'HorizontalAlignment', 'center');
      end
    end
    title('Rangos medios por escenario (test de Friedman)', 'FontSize', 10);
    saveas(gcf, fullfile(figs_dir, 'fig_ranks.png')); close(gcf);
  end

  % ---- fig_torque --------------------------------------------
  fp_ctc = fullfile(traces_dir, 'S3_CTC.mat');
  figure('Position', [50 50 720 640]);
  for j = 1:3
    subplot(3,1,j); hold on;
    if exist(tr_path, 'file')
      plot(tr.t, tr.tau(:,j), 'Color', [0 0.45 0.70], 'LineWidth', 1.2, ...
           'DisplayName', 'PR-DE (adaptativo)');
    end
    if exist(fp_ctc, 'file')
      dc = load(fp_ctc); dc = dc.logs;
      plot(dc.t, dc.tau(:,j), '--', 'Color', [0.33 0.33 0.33], ...
           'LineWidth', 1.0, 'DisplayName', 'CTC fija');
    end
    p = arm_params();
    hline( p.tau_max(j), ':', [0.84 0.37 0.00]);
    hline(-p.tau_max(j), ':', [0.84 0.37 0.00]);
    ylabel(sprintf('\\tau_{%d} [N*m]', j));
    title(sprintf('Articulacion %d', j), 'FontSize', 9);
    if j == 1, legend('Location', 'northeast'); end
  end
  xlabel('t [s]');
  sgtitle('Pares de control (S3). Lineas punteadas: limites Dynamixel', ...
          'FontSize', 10);
  saveas(gcf, fullfile(figs_dir, 'fig_torque.png')); close(gcf);

  fprintf('Figuras generadas en %s\n', figs_dir);
end

% -------------------------------------------------------------------------
function R = read_csv_struct(path)
  fid = fopen(path, 'r');
  if fid < 0, R = {}; return; end
  hdr = strsplit(fgetl(fid), ',');
  R = {};
  while ~feof(fid)
    line = fgetl(fid);
    if ~ischar(line) || isempty(strtrim(line)), continue; end
    f = strsplit(strtrim(line), ',');
    s = struct();
    for k = 1:numel(hdr)
      s.(hdr{k}) = f{k};
    end
    R{end+1} = s; %#ok<AGROW>
  end
  fclose(fid);
end

function vline(x, ls, c)
% Línea vertical en los ejes actuales. Usa xline si está disponible
% (MATLAB R2018b+ / Octave 5+); si no, usa plot con ylim.
  ax = gca;
  if exist('xline', 'file') == 2 || exist('xline', 'builtin') == 5
    xline(x, ls, 'Color', c);
  else
    yl = ax.YLim;
    hold(ax, 'on');
    plot(ax, [x x], yl, ls, 'Color', c);
  end
end

function hline(y, ls, c)
% Línea horizontal en los ejes actuales. Usa yline si está disponible
% (MATLAB R2018b+ / Octave 5+); si no, usa plot con xlim.
  ax = gca;
  if exist('yline', 'file') == 2 || exist('yline', 'builtin') == 5
    yline(y, ls, 'Color', c);
  else
    xl = ax.XLim;
    hold(ax, 'on');
    plot(ax, xl, [y y], ls, 'Color', c);
  end
end
