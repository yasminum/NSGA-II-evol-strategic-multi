%% @KrigingSurrogate  —  Kriging / Regresión por Procesos Gaussianos (GPR).
% ------------------------------------------------------------------
% Modelo sustituto de costo con kernel exponencial cuadrado y
% verosimilitud marginal máxima (REML aproximado por búsqueda de
% sección áurea).
%
% Soporta salidas multi-dimensionales (n_out > 1) manteniendo un GP
% independiente por cada salida. La varianza predictiva se devuelve
% como la media de las varianzas individuales (aproximación).
%
%   f(x) ~ N(m(x), σ²(x))
%
% Donde la varianza predictiva σ²(x) se usa para bandera de "región no
% explorada" en la estrategia de muestreo adaptativo (SurrogateManager).
%
% Implementación minimalista (sin dependencias externas), con
% hiperparámetros θ (log-escala del kernel) y σ_n² (ruido de observación)
% optimizados por maximización de la verosimilitud marginal.
%
% API compatible: .fit(X, y), .predict(X) -> [mean, var], .predict_one(x).
% ------------------------------------------------------------------
classdef KrigingSurrogate < handle
  properties
    n_in
    n_out
    X          % datos de entrenamiento (N × n_in)
    y          % objetivos (N × n_out)
    theta      % vector 1×n_in de long-escalas inversas
    sigma_n    % std de ruido
    Ls         % cell de factorizaciones de Cholesky por salida
    alphas     % cell de soluciones intermedias por salida
    mus        % medias por salida (vector 1×n_out)
    is_trained
  end
  methods
    function obj = KrigingSurrogate(n_in, n_out)
      if nargin < 1, n_in = 9; end
      if nargin < 2, n_out = 1; end
      obj.n_in = n_in;
      obj.n_out = n_out;
      obj.theta = ones(1, n_in);
      obj.sigma_n = 1e-3;
      obj.is_trained = false;
    end

    function fit(obj, X, y)
      N = size(X, 1);
      if N < 2
        obj.is_trained = false;
        return;
      end
      if size(y, 2) > obj.n_out
        obj.n_out = size(y, 2);
      end
      obj.X = X;
      obj.y = y;
      % Optimización de theta por búsqueda de sección áurea sobre log θ.
      log_theta_opt = golden_section(@(lt) obj.neg_log_marginal(exp(lt), X, y(:, 1)), ...
                                     log(1e-2), log(1e2), 30);
      obj.theta = exp(log_theta_opt) * ones(1, obj.n_in);
      % Para cada salida, construir el GP final con theta óptimo.
      K = obj.kernel(X, X, obj.theta);
      K = K + (obj.sigma_n^2 + 1e-8) * eye(N);
      L = chol(K, 'lower');
      obj.Ls = cell(1, obj.n_out);
      obj.alphas = cell(1, obj.n_out);
      obj.mus = zeros(1, obj.n_out);
      for k = 1:obj.n_out
        obj.mus(k) = mean(y(:, k));
        a = L \ (y(:, k) - obj.mus(k));
        obj.alphas{k} = L' \ a;
        obj.Ls{k} = L;   % mismo L para todas las salidas (mismo theta)
      end
      obj.is_trained = true;
    end

    function [mu, s2] = predict(obj, X)
      if ~obj.is_trained
        mu = zeros(size(X, 1), obj.n_out);
        s2 = 1e6 * ones(size(X, 1), 1);
        return;
      end
      Kst = obj.kernel(X, obj.X, obj.theta);   % (Ntest × Ntrain)
      v = obj.Ls{1} \ Kst.';
      mu = zeros(size(X, 1), obj.n_out);
      for k = 1:obj.n_out
        mu(:, k) = obj.mus(k) + Kst * obj.alphas{k};
      end
      s2 = max(0, 1 - sum(v.^2, 1).').';
    end

    function [yhat, var] = predict_one(obj, x)
      [mu, s2] = obj.predict(x(:).');
      yhat = mu(1, :);
      var  = s2(1);
    end

    function [yhat, var] = predict_var(obj, X)
    % API compatible con SurrogateManager: devuelve (P×n_out) media y
    % (P×1) varianza promedio.
      [yhat, s2] = obj.predict(X);
      var = s2;
    end

    function nll = neg_log_marginal(obj, theta, X, y)
      K = obj.kernel(X, X, theta * ones(1, size(X, 2)));
      K = K + (obj.sigma_n^2 + 1e-8) * eye(size(X, 1));
      L = chol(K, 'lower');
      mu = mean(y);
      a = L \ (y - mu);
      nll = 0.5 * (a' * (L' \ a) + 2*sum(log(diag(L))) + size(X,1)*log(2*pi));
    end

    function K = kernel(obj, A, B, theta)
    % Kernel exponencial cuadrado (ARD por dimensión).
      if size(A, 2) ~= numel(theta)
        theta = repmat(theta(1), 1, size(A, 2));
      end
      D2 = zeros(size(A, 1), size(B, 1));
      for d = 1:size(A, 2)
        ad = A(:, d);
        bd = B(:, d);
        D2 = D2 + theta(d) * ((sum(ad.^2, 2)) ...
                              + sum(bd.^2, 2).' - 2 * (ad * bd.'));
      end
      K = exp(-D2);
    end
  end
end

function x = golden_section(f, a, b, n_iter)
% Búsqueda de sección áurea para minimizar f en [a, b].
  phi = (sqrt(5) - 1) / 2;
  c = b - phi * (b - a);
  d = a + phi * (b - a);
  fc = f(c); fd = f(d);
  for k = 1:n_iter
    if fc < fd
      b = d; d = c; fd = fc;
      c = b - phi * (b - a);
      fc = f(c);
    else
      a = c; c = d; fc = fd;
      d = a + phi * (b - a);
      fd = f(d);
    end
  end
  x = 0.5 * (a + b);
end
