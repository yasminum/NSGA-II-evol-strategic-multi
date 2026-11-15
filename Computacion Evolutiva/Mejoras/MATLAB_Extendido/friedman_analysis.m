%% friedman_analysis.m  —  Friedman + rangos + Wilcoxon-Holm.
% ------------------------------------------------------------------
% Protocolo estadístico de Rodríguez-Molina et al. (2024, sec. 4.3):
%
%   1. test de Friedman por escenario (k métodos, N corridas)
%   2. rangos medios por método y W de Kendall = χ² / (N·(k-1))
%   3. test de Wilcoxon por pares (control vs. cada uno de los demás)
%   4. corrección Holm step-down sobre los p-valores de Wilcoxon
%
% Argumentos:
%   mat      matriz N×k (corridas × métodos) con el ISE de cada caso
%   methods  cell array de nombres (k,)
%   alpha    nivel de significancia (0.05 por defecto)
%
% Devuelve struct con campos:
%   chi2, p, W, mean_ranks, control, pairwise
% ------------------------------------------------------------------
function st = friedman_analysis(mat, methods, alpha)
  if nargin < 3 || isempty(alpha), alpha = 0.05; end
  [N, k] = size(mat);

  % Friedman chi-cuadrado: requiere la Statistics Toolbox en MATLAB.
  % En Octave usamos una implementación propia basada en rangos
  % equivalentes (Friedman χ² = (12 / (N·k·(k+1))) · Σ (R_j - N(k+1)/2)²)
  ranks = zeros(N, k);
  for i = 1:N
    ranks(i,:) = tiedrank(mat(i,:));
  end
  Rj = sum(ranks, 1);              % (1,k)
  Rj_bar = N * (k + 1) / 2;
  chi2 = (12.0 / (N * k * (k + 1))) * sum((Rj - Rj_bar).^2);
  p = 1.0 - chi2cdf(chi2, k - 1);     % p-valor asintótico
  mean_ranks = mean(ranks, 1);
  W = chi2 / (N * (k - 1));
  [~, ctrl_idx] = min(mean_ranks);
  control = methods{ctrl_idx};

  % comparaciones por pares: control vs. cada uno (Wilcoxon signed-rank)
  others = setdiff(1:k, ctrl_idx);
  raw = struct('idx', {}, 'p', {});
  for ii = 1:numel(others)
    j = others(ii);
    try
      [~, pj] = signrank(mat(:, ctrl_idx), mat(:, j));
      if isempty(pj) || isnan(pj), pj = 1.0; end
    catch
      pj = 1.0;
    end
    raw(end+1) = struct('idx', j, 'p', pj); %#ok<AGROW>
  end
  % ordenar por p-valor ascendente
  pv = [raw.p];
  ord_idx = [raw.idx];
  [pv, ord] = sort(pv);
  ord_idx = ord_idx(ord);

  % corrección Holm step-down: p_adj(i) = min(1, max_{j>=i} (m-j) * p(j))
  m_cmp = numel(pv);
  adj = zeros(m_cmp, 1);
  for i = 1:m_cmp
    adj(i) = (m_cmp - i + 1) * pv(i);
  end
  % step-down: el ajustado en i es el máx de los (m-j+1)·p(j) para j>=i
  adj = flip(cummax(flip(adj)));
  adj = min(adj, 1.0);

  pairs = struct('vs', {}, 'p_raw', {}, 'p_holm', {}, 'reject', {});
  for i = 1:m_cmp
    j = ord_idx(i);
    reject = adj(i) <= alpha;
    pairs(end+1) = struct('vs', methods{j}, 'p_raw', pv(i), ...
                          'p_holm', adj(i), 'reject', reject); %#ok<AGROW>
  end

  st = struct('chi2', chi2, 'p', p, 'W', W, ...
              'mean_ranks', mean_ranks, 'control', control, ...
              'pairwise', pairs);
end

function r = tiedrank(x)
% Rangos con promedio en empates (equivalente a scipy.stats.rankdata
% con método 'average'). Para datos continuos como ISE los empates son
% excepcionales, pero se mantiene la fidelidad con el código Python.
  n = numel(x);
  [xs, idx] = sort(x);
  r = zeros(size(x));
  i = 1;
  while i <= n
    j = i;
    while j < n && xs(j+1) == xs(i)
      j = j + 1;
    end
    rank_avg = (i + j) / 2;       % rango medio del grupo de empate
    for k = i:j
      r(idx(k)) = rank_avg;
    end
    i = j + 1;
  end
end

function cdf = chi2cdf(x, k)
% CDF de chi-cuadrado con k grados de libertad: chi2cdf(x,k) = P(k/2, x/2)
% donde P(a,z) es la gamma incompleta regularizada inferior.
% `gammainc` está integrado en MATLAB y en Octave (sin toolbox).
  if x <= 0, cdf = 0; return; end
  cdf = gammainc(x / 2, k / 2);
end
