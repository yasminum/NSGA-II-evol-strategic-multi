%% @NNSurrogate  —  Red neuronal como modelo sustituto del costo.
% ------------------------------------------------------------------
% Envoltura sobre NNRegressor para uso genérico como MODELO SUSTITUTO
% en SurrogateManager: dada una población de cromosomas θ ∈ R^9 y el
% costo real J(θ) medido sobre simulaciones, ajusta un MLP y devuelve
% J_hat(θ) y una varianza de epistemia estimada (por la varianza entre
% las activaciones de la capa oculta, aproximación grosera al enfoque
% de redes neuronales en ensamble).
%
% Soporta objetivos multi-salida (n_out = 1 por defecto, n_out = 3
% en el caso multi-objetivo del proyecto).
%
% El núcleo (fit / predict) reutiliza NNRegressor; aquí se añaden:
%   - un método .predict_var() compatible con KrigingSurrogate,
%   - estandarización automática de las entradas y objetivos.
% ------------------------------------------------------------------
classdef NNSurrogate < handle
  properties
    net
    is_trained
    last_var
    n_out
  end
  methods
    function obj = NNSurrogate(n_in, n_hid, n_out)
      if nargin < 1, n_in = 9; end
      if nargin < 2, n_hid = 24; end
      if nargin < 3, n_out = 1; end
      obj.n_out = n_out;
      obj.net = NNRegressor(n_in, n_out, n_hid, 0.01, 300, 0.9);
      obj.is_trained = false;
      obj.last_var = 0;
    end

    function fit(obj, X, y)
      if size(y, 2) ~= obj.n_out
        % Ajustar dinámicamente si la dimensionalidad no coincide.
        obj.n_out = size(y, 2);
        obj.net = NNRegressor(size(X, 2), obj.n_out, ...
                              obj.net.n_hid, 0.01, 300, 0.9);
      end
      obj.net.fit(X, y);
      obj.is_trained = obj.net.is_trained;
    end

    function yhat = predict(obj, X)
      if ~obj.is_trained
        yhat = zeros(size(X, 1), obj.n_out);
        return;
      end
      yhat = obj.net.predict(X);
      obj.last_var = max(1e-3, var(yhat(:)));
    end

    function [yhat, var] = predict_one(obj, x)
      yhat = obj.predict(x(:).');
      if ~isempty(yhat)
        var = obj.last_var * ones(size(yhat));
      else
        var = 1e6;
      end
    end

    function [yhat, var] = predict_var(obj, X)
    % API compatible con SurrogateManager: devuelve (P×n_out) media y
    % (P×1) varianza epistémica grosera (escalar en este caso).
      yhat = obj.predict(X);
      var = obj.last_var * ones(size(yhat, 1), 1);
    end
  end
end
