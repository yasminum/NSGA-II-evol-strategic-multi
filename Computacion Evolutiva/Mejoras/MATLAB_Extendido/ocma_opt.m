%% ocma_opt.m  —  Online CMA-ES (reinicio suave cada ciclo).
% ------------------------------------------------------------------
% (μ, λ)-CMA-ES estándar (Hansen & Ostermeier, 2001) con:
%   λ = pop = 20, μ = λ/2 = 10
%   pesos logarítmicos normalizados  w_i ∝ log(μ + 0.5) − log(i)
%   mueff = 1 / Σ w_i²
%
% Reinicio suave: en cada ciclo de control se re-inicializa
%   m0     = mejor individuo previo (memoria = 1 individuo)
%   σ0     = 0.25   (25 % del rango)
%   C0     = I
%
% Adaptación de ruta (cumulation):
%   ps ← (1−cs)·ps + sqrt(cs·(2−cs)·mueff)·C^{-1/2}·yw
%   pc ← (1−cc)·pc + hsig·sqrt(cc·(2−cc)·mueff)·yw
%   C  ← (1−c1−cmu)·C + c1·(pc·pc' + (1−hsig)·cc·(2−cc)·C)
%                       + cmu·Σ w_i·Y_i·Y_i'
%   σ  ← σ · exp((cs/ds)·(||ps||/chiN − 1))
%   m  ← m + σ·yw   (con clip al [0,1])
%
% Sintaxis:
%   [x_best, f_best] = ocma_opt(fbatch, dim, prev_best, pop, gens, sigma0)
% ------------------------------------------------------------------
function [x_best, f_best] = ocma_opt(fbatch, dim, prev_best, pop, gens, sigma0)
  if nargin < 2 || isempty(dim),       dim = 9;        end
  if nargin < 3 || isempty(prev_best), prev_best = []; end
  if nargin < 4 || isempty(pop),       pop = 20;       end
  if nargin < 5 || isempty(gens),       gens = 20;      end
  if nargin < 6 || isempty(sigma0),    sigma0 = 0.25;  end

  n   = dim;
  lam = pop;
  mu  = floor(lam / 2);
  w_  = log(mu + 0.5) - log(1:mu).';            % (mu,1)
  w_  = w_ / sum(w_);
  mueff = 1.0 / sum(w_.^2);

  sigma = sigma0;
  if ~isempty(prev_best)
    m = prev_best(:);
  else
    m = 0.5 * ones(n, 1);
  end
  C  = eye(n);
  pc = zeros(n, 1);
  ps = zeros(n, 1);

  cs = (mueff + 2.0) / (n + mueff + 3.0);
  ds = 1.0 + 2.0 * max(0, sqrt((mueff - 1.0) / (n + 1.0)) - 1.0) + cs;
  cc = (4.0 + mueff / n) / (n + 4.0 + 2.0 * mueff / n);
  c1 = 2.0 / ((n + 1.3)^2 + mueff);
  cmu = min(1.0 - c1, 2.0 * (mueff - 2.0 + 1.0/mueff) / ((n + 2.0)^2 + mueff));
  chiN = sqrt(n) * (1.0 - 1.0/(4.0*n) + 1.0/(21.0*n*n));

  best_x = []; best_f = inf;

  for g = 1:gens
    [B, D2] = eig(C); D2 = diag(D2);
    D = sqrt(max(D2, 1e-12));                  % (n,1)
    BD = B .* D.';                             % B * diag(D)
    Z = randn(lam, n);                          % (lam,n)
    Y = Z * BD.';                              % (lam,n) pasos
    X = max(0, min(1, repmat(m.', lam, 1) + sigma * Y));
    Fs = fbatch(X);

    [Fs_sorted, order] = sort(Fs);
    if Fs_sorted(1) < best_f
      best_f = Fs_sorted(1);
      best_x = X(order(1),:);
    end

    Ys = Y(order(1:mu), :);                    % (mu,n)
    yw = (w_.' * Ys).';                          % (n,1)
    inv_sqrt_C = (B ./ D) * B.';                % C^{-1/2}
    ps = (1 - cs) * ps + sqrt(cs * (2 - cs) * mueff) * (inv_sqrt_C * yw);
    hsig_cond = norm(ps) / sqrt(1 - (1 - cs)^(2*g)) / chiN;
    hsig = hsig_cond < (1.4 + 2.0/(n + 1.0));
    if hsig
      pc = (1 - cc) * pc + sqrt(cc * (2 - cc) * mueff) * yw;
    else
      pc = (1 - cc) * pc;
    end
    C  = (1 - c1 - cmu) * C ...
         + c1 * (pc * pc.' + (1 - hsig) * cc * (2 - cc) * C) ...
         + cmu * (Ys.' * (w_ .* Ys));          % Σ w_i Y_i Y_i'
    sigma = sigma * exp((cs / ds) * (norm(ps)/chiN - 1));
    sigma = min(max(sigma, 1e-4), 0.5);
    m = min(max(m + sigma * yw, 0), 1);
  end

  if isempty(best_x)
    x_best = m(:).'; f_best = inf;
  else
    x_best = best_x; f_best = best_f;
  end
end
