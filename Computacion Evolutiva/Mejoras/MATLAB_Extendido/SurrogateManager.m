%% @SurrogateManager  —  Gestor de modelos sustitutos con muestreo adaptativo.
% ------------------------------------------------------------------
% El gestor mantiene una base de datos (X, y) de cromosomas evaluados
% sobre la planta y un MODELO SUSTITUTO ajustado a esa base. En cada
% ciclo de optimización evolutiva multi-objetivo:
%
%   1. Las N_eval_reales (p. ej. 20) primeras muestras de cada generación
%      se evalúan con el sustituto (barato).
%   2. Las k_lie (p. ej. 4) más prometedoras o más inciertas (LIE:
%      "Likelihood of Improvement Exploration") se evalúan con la
%      planta real (caro) y se añaden a la base de datos.
%   3. El sustituto se reajusta con la base actualizada.
%
% Esto reduce el número de simulaciones de ~400 por ciclo a ~k_lie + N_gen
% por ciclo, manteniendo la calidad de la sintonía (objetivo del
% "Surrogate-Assisted Evolutionary Tuning" del título del proyecto).
%
% Soporta tres clases de sustituto:
%   'rbf'      → RBFNet (nativo multi-salida)
%   'kriging'  → KrigingSurrogate (un GP por salida)
%   'nn'       → NNSurrogate (MLP multi-salida)
%
% Mantiene dos objetivos:
%   - Predecir el costo J(θ) con el menor error posible,
%   - Estimar la incertidumbre σ²(θ) para guiar la exploración.
% ------------------------------------------------------------------
classdef SurrogateManager < handle
  properties
    kind          % 'rbf' | 'kriging' | 'nn'
    surrogates    % cell de 1×n_surrogates (ensamble)
    n_surrogates  % número de sustitutos en el ensamble
    X_db          % base de datos de cromosomas (N × n_in)
    y_db          % base de datos de costos (N × n_obj)
    n_in
    n_obj
    is_ready
  end
  methods
    function obj = SurrogateManager(kind, n_in, n_obj, n_surrogates)
      if nargin < 1 || isempty(kind), kind = 'rbf'; end
      if nargin < 2 || isempty(n_in), n_in = 9; end
      if nargin < 3 || isempty(n_obj), n_obj = 1; end
      if nargin < 4 || isempty(n_surrogates), n_surrogates = 1; end
      obj.kind = kind;
      obj.n_in = n_in;
      obj.n_obj = n_obj;
      obj.n_surrogates = n_surrogates;
      obj.surrogates = cell(1, n_surrogates);
      for k = 1:n_surrogates
        switch lower(kind)
          case 'rbf',     obj.surrogates{k} = RBFNet(n_in, n_obj, 32, 1e-3);
          case 'kriging', obj.surrogates{k} = KrigingSurrogate(n_in, n_obj);
          case 'nn',      obj.surrogates{k} = NNSurrogate(n_in, 24, n_obj);
          otherwise,     obj.surrogates{k} = RBFNet(n_in, n_obj, 32, 1e-3);
        end
      end
      obj.X_db = zeros(0, n_in);
      obj.y_db = zeros(0, n_obj);
      obj.is_ready = false;
    end

    function add_samples(obj, X, Y)
    % Añade pares (X, Y) a la base de datos y reajusta los sustitutos.
      obj.X_db = [obj.X_db; X];
      obj.y_db = [obj.y_db; Y];
      if size(obj.X_db, 1) >= 2
        for k = 1:obj.n_surrogates
          if ismethod(obj.surrogates{k}, 'fit') || isprop(obj.surrogates{k}, 'fit')
            obj.surrogates{k}.fit(obj.X_db, obj.y_db);
          end
        end
        obj.is_ready = true;
      end
    end

    function [yhat, v_out] = predict_ensemble(obj, X)
    % Devuelve la media (y variabilidad) del ensamble.
      M = size(X, 1);
      preds = zeros(M, obj.n_surrogates, obj.n_obj);
      v_out = zeros(M, 1);
      for k = 1:obj.n_surrogates
        if ismethod(obj.surrogates{k}, 'predict_var') || isprop(obj.surrogates{k}, 'predict_var')
          [yhat_k, vk] = obj.surrogates{k}.predict_var(X);
        else
          yhat_k = obj.surrogates{k}.predict(X);
          vk = var(yhat_k(:)) * ones(M, 1);
        end
        % Asegurar dimensiones consistentes (M × n_obj).
        if size(yhat_k, 2) == 1 && obj.n_obj > 1
          % Replicar la predicción unidimensional para los demás objetivos.
          yhat_k = repmat(yhat_k, 1, obj.n_obj);
        end
        if size(yhat_k, 2) < obj.n_obj
          % Replicar para igualar n_obj.
          yhat_k = repmat(yhat_k, 1, obj.n_obj);
        end
        preds(:, k, :) = yhat_k;
        if k == 1
          v_out(:, 1) = mean(vk, 2);
        else
          v_out(:, 1) = v_out(:, 1) + mean(vk, 2);
        end
      end
      % Media del ensamble + varianza epistémica inter-sustituto.
      yhat = zeros(M, obj.n_obj);
      for j = 1:obj.n_obj
        yhat(:, j) = mean(preds(:, :, j), 2);
      end
      if obj.n_surrogates > 1
        v_out = v_out / obj.n_surrogates + ...
                var(squeeze(mean(preds, 3)), 0, 2);
      end
    end

    function [X_lie, idx_lie] = select_lie(obj, X_cand, yhat, var, k_lie)
    % Selección LIE: combina baja predicción (excelente) con alta
    % incertidumbre (no explorado). Normaliza cada criterio y escoge
    % los k_lie de mayor "potencial" (lower-confidence-bound).
      if nargin < 4 || isempty(k_lie), k_lie = 4; end
      if isempty(yhat), yhat = zeros(size(X_cand, 1), obj.n_obj); end
      if isempty(var),  var  = zeros(size(X_cand, 1), 1); end
      % LCB sobre el primer objetivo (ISE_pred) — clásico "expected
      % improvement" simplificado.
      kappa = 1.0;
      y_score = yhat(:, 1);
      if any(var > 0)
        lcb = y_score - kappa * sqrt(max(var, 0));
      else
        lcb = y_score;
      end
      [~, idx] = sort(lcb, 'ascend');
      idx_lie = idx(1:min(k_lie, numel(idx))).';
      X_lie = X_cand(idx_lie, :);
    end

    function J = evaluate_true(obj, X, true_fn)
    % Punto de extensión: evalúa la "verdadera" planta para los
    % candidatos seleccionados (true_fn devuelve el costo real).
      Y = true_fn(X);
      obj.add_samples(X, Y);
      J = Y;
    end
  end
end
