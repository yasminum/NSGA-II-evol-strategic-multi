%% nsga2.m  —  Algoritmo Evolutivo Multi-Objetivo NSGA-II.
% ------------------------------------------------------------------
% Implementación independiente (sin toolboxes) del NSGA-II clásico
% (Deb et al., 2002) para optimización multi-objetivo con variables
% reales:
%
%   - Ordenamiento por no-dominancia por frentes (fast non-dominated sort).
%   - Distancia de hacinamiento (crowding distance) para mantener diversidad.
%   - Selección por torneo (compara primero por frente, luego por crowding).
%   - Cruce SBX (Simulated Binary Crossover) y mutación polinomial.
%   - Elitismo estricto: la nueva población se forma a partir de los
%     (padres + hijos) ordenados por frente y crowding.
%
% Uso:
%   [Pareto, F_pop, all_pop] = nsga2(f_objs, n_var, lo, hi, pop, gens, ...
%                                    varargin)
%
%   f_objs  function handle:  X (P, n_var) -> Y (P, n_obj).
%           Puede tomar una segunda salida var (P, n_obj) para acelerar
%           la selección cuando el sustituto entrega varianza.
%   n_var   número de variables (en este proyecto: 9 ganancias PID).
%   lo, hi  cotas inferiores y superiores (1×n_var).
%   pop     tamaño de población.
%   gens    número de generaciones.
%
% Salidas:
%   Pareto    cromosomas del primer frente no dominado (P_p × n_var).
%   F_pop     valores f(Pareto)         (P_p × n_obj).
%   all_pop   población final completa (pop × n_var) para diagnóstico.
%
% Opciones:
%   'pc'      prob. de cruce SBX (0.9)
%   'pm'      prob. de mutación polinomial (1/n_var)
%   'eta_c'   índice de distribución SBX (15)
%   'eta_m'   índice de distribución de mutación (20)
%   'seed'    semilla (0)
%   'warm'    cromosoma caliente inicial (1×n_var)
%   'init_pop' población inicial opcional (pop × n_var)
% ------------------------------------------------------------------
function [Pareto, F_pop, all_pop] = nsga2(f_objs, n_var, lo, hi, pop, gens, varargin)
  % defaults
  opt = struct('pc', 0.9, 'pm', 1/n_var, 'eta_c', 15, 'eta_m', 20, ...
               'seed', 0, 'warm', [], 'init_pop', [], 'verbose', false);
  for k = 1:2:numel(varargin)
    if isfield(opt, varargin{k}), opt.(varargin{k}) = varargin{k+1}; end
  end
  rng(opt.seed);
  lo = lo(:).'; hi = hi(:).';
  if isempty(opt.init_pop)
    P = rand(pop, n_var) .* repmat(hi - lo, pop, 1) + repmat(lo, pop, 1);
    if ~isempty(opt.warm)
      P(1, :) = max(lo, min(hi, opt.warm));
    end
  else
    P = opt.init_pop;
  end

  F = f_objs(P);

  for g = 1:gens
    % Ordenamiento no-dominado y crowding
    [fronts, rank] = fast_nds(F);
    crowd = crowding_distance(F, fronts);
    % Generar hijos (Q) por torneo + SBX + mutación
    Q = generate_offspring(P, F, rank, crowd, lo, hi, opt);
    Fq = f_objs(Q);
    % Combinar padres + hijos, elitismo
    R_P = [P; Q];
    R_F = [F; Fq];
    [fronts_r, rank_r] = fast_nds(R_F);
    new_idx = [];
    i = 1;
    while i <= numel(fronts_r) && numel(new_idx) + numel(fronts_r{i}) <= pop
      new_idx = [new_idx, fronts_r{i}(:).']; %#ok<AGROW>
      i = i + 1;
    end
    if numel(new_idx) < pop && i <= numel(fronts_r)
      last = fronts_r{i}(:).';
      crowd_last = crowding_distance(R_F(last, :), {1:numel(last)});
      [~, sort_c] = sort(crowd_last, 'descend');
      new_idx = [new_idx, last(sort_c(1:(pop - numel(new_idx))))]; %#ok<AGROW>
    end
    P = R_P(new_idx, :);
    F = R_F(new_idx, :);
    if opt.verbose && mod(g, 10) == 0
      fprintf('NSGA-II gen %d, frente 1 tamaño %d\n', g, numel(fronts_r{1}));
    end
  end

  [fronts_final, ~] = fast_nds(F);
  Pareto_idx = fronts_final{1};
  Pareto = P(Pareto_idx, :);
  F_pop  = F(Pareto_idx, :);
  all_pop = P;
end

% ----------------------- fast non-dominated sort ----------------------- %
function [fronts, rank] = fast_nds(F)
  N = size(F, 1);
  S = cell(N, 1);
  n_dom = zeros(N, 1);
  rank = zeros(N, 1);
  for p = 1:N
    for q = 1:N
      if p == q, continue; end
      if dominates(F(p, :), F(q, :))
        S{p} = [S{p}, q];
      elseif dominates(F(q, :), F(p, :))
        n_dom(p) = n_dom(p) + 1;
      end
    end
    if n_dom(p) == 0
      rank(p) = 1;
    end
  end
  fronts = {};
  cur = find(rank == 1).';
  while ~isempty(cur)
    fronts{end+1} = cur; %#ok<AGROW>
    next = [];
    for p = cur(:).'
      for q = S{p}(:).'
        n_dom(q) = n_dom(q) - 1;
        if n_dom(q) == 0
          rank(q) = rank(p) + 1;
          next = [next, q]; %#ok<AGROW>
        end
      end
    end
    cur = next;
  end
end

function d = dominates(a, b)
% a dominates b si a ≤ b en todos los objetivos y a < b en al menos uno.
  d = all(a <= b + 1e-12) && any(a < b - 1e-12);
end

% ----------------------- crowding distance ----------------------- %
function crowd = crowding_distance(F, fronts)
  N = size(F, 1);
  crowd = zeros(N, 1);
  for f = 1:numel(fronts)
    I = fronts{f};
    if numel(I) <= 2
      crowd(I) = inf;
      continue;
    end
    for m = 1:size(F, 2)
      [~, s] = sort(F(I, m), 'ascend');
      crowd(I(s(1))) = inf;
      crowd(I(s(end))) = inf;
      f_min = F(I(s(1)), m);
      f_max = F(I(s(end)), m);
      rng_ = max(f_max - f_min, 1e-12);
      for j = 2:(numel(I) - 1)
        crowd(I(s(j))) = crowd(I(s(j))) ...
          + (F(I(s(j+1)), m) - F(I(s(j-1)), m)) / rng_;
      end
    end
  end
end

% ----------------------- generacion de hijos ----------------------- %
function Q = generate_offspring(P, ~, rank, crowd, lo, hi, opt)
  pop = size(P, 1);
  n_var = size(P, 2);
  % Torneo binario: gana el de menor rank, o mayor crowd en empate.
  parents = zeros(pop, 1);
  for i = 1:pop
    a = randi(pop);
    b = randi(pop);
    if rank(a) < rank(b) || (rank(a) == rank(b) && crowd(a) > crowd(b))
      parents(i) = a;
    else
      parents(i) = b;
    end
  end
  Q = zeros(pop, n_var);
  for i = 1:2:pop
    p1 = P(parents(i), :);
    if i < pop
      p2 = P(parents(i+1), :);
    else
      p2 = P(parents(1), :);
    end
    if rand() < opt.pc
      [c1, c2] = sbx(p1, p2, lo, hi, opt.eta_c);
    else
      c1 = p1; c2 = p2;
    end
    c1 = poly_mutate(c1, lo, hi, opt.pm, opt.eta_m);
    c2 = poly_mutate(c2, lo, hi, opt.pm, opt.eta_m);
    Q(i, :)   = c1;
    if i < pop
      Q(i+1, :) = c2;
    end
  end
end

function [c1, c2] = sbx(p1, p2, lo, hi, eta_c)
  n = numel(p1);
  c1 = p1; c2 = p2;
  for k = 1:n
    if rand() < 0.5
      u = rand();
      if u <= 0.5
        beta_q = (2*u)^(1/(eta_c+1));
      else
        beta_q = (1/(2*(1-u)))^(1/(eta_c+1));
      end
      x1 = 0.5*((1+beta_q)*p1(k) + (1-beta_q)*p2(k));
      x2 = 0.5*((1-beta_q)*p1(k) + (1+beta_q)*p2(k));
      c1(k) = min(max(x1, lo(k)), hi(k));
      c2(k) = min(max(x2, lo(k)), hi(k));
    end
  end
end

function y = poly_mutate(x, lo, hi, pm, eta_m)
  y = x;
  for k = 1:numel(x)
    if rand() < pm
      delta = rand();
      if delta < 0.5
        delta_q = (2*delta)^(1/(eta_m+1)) - 1;
      else
        delta_q = 1 - (2*(1-delta))^(1/(eta_m+1));
      end
      y(k) = x(k) + delta_q * (hi(k) - lo(k));
      y(k) = min(max(y(k), lo(k)), hi(k));
    end
  end
end
