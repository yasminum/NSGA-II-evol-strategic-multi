%% @NNRegressor  —  Red neuronal multicapa para identificación de sistemas.
% ------------------------------------------------------------------
% Perceptrón multicapa (MLP) con 1 capa oculta y activación tanh.
%
%   Entradas:  x ∈ R^n_in   (en este proyecto n_in = 9: [q(3), v(3), u(3)])
%   Salidas:   y ∈ R^n_out  (en este proyecto n_out = 6: [q_next, v_next])
%
%   y = W2 * tanh(W1 * x + b1) + b2
%
% Entrenamiento: descenso de gradiente estocástico con momento.
% Para identificación online se reentrena sobre una ventana deslizante
% de s_b muestras (igual que AdaptiveCTC con monomials).
%
% En este proyecto se usa como alternativa al regresor polinomial (PR/SLR)
% dentro del controlador adaptativo NNAdaptiveCTC, para comparar la
% capacidad de generalización de una red neuronal frente a un regresor
% lineal en los parámetros con las mismas dimensiones de entrada/salida.
%
% La red se inicializa con esquema Xavier/Glorot y se normalizan las
% entradas y salidas (z-score) antes de optimizar, lo que estabiliza el
% entrenamiento. Las predicciones se "des-normalizan" al devolverlas.
% ------------------------------------------------------------------
classdef NNRegressor < handle
  properties
    n_in        % dimensionalidad de entrada
    n_hid       % número de neuronas en la capa oculta
    n_out       % dimensionalidad de salida
    W1, b1      % pesos y sesgos capa oculta
    W2, b2      % pesos y sesgos capa de salida
    mu_in,  sd_in   % estadísticas de normalización de entrada
    mu_out, sd_out  % estadísticas de normalización de salida
    rho        % coeficiente de momento (0.9 por defecto)
    lr         % tasa de aprendizaje (0.01 por defecto)
    epochs     % número de épocas de entrenamiento (200 por defecto)
    dW1, dW2, db1, db2  % incrementos previos (para momento)
    is_trained % true una vez que se han ajustado mu/sd y pesos
  end
  methods
    function obj = NNRegressor(n_in, n_out, n_hid, lr, epochs, rho)
      if nargin < 1 || isempty(n_in),  n_in  = 9;  end
      if nargin < 2 || isempty(n_out), n_out = 6;  end
      if nargin < 3 || isempty(n_hid), n_hid = 16; end
      if nargin < 4 || isempty(lr),    lr    = 0.01; end
      if nargin < 5 || isempty(epochs), epochs = 200; end
      if nargin < 6 || isempty(rho),   rho   = 0.9;  end
      obj.n_in  = n_in;
      obj.n_hid = n_hid;
      obj.n_out = n_out;
      obj.lr    = lr;
      obj.epochs= epochs;
      obj.rho   = rho;
      % Xavier/Glorot init
      r1 = sqrt(6 / (n_in + n_hid));
      obj.W1 = -r1 + 2*r1*rand(n_hid, n_in);
      obj.b1 = zeros(n_hid, 1);
      r2 = sqrt(6 / (n_hid + n_out));
      obj.W2 = -r2 + 2*r2*rand(n_out, n_hid);
      obj.b2 = zeros(n_out, 1);
      obj.mu_in  = zeros(1, n_in);
      obj.sd_in  = ones(1, n_in);
      obj.mu_out = zeros(1, n_out);
      obj.sd_out = ones(1, n_out);
      obj.dW1 = zeros(size(obj.W1));
      obj.dW2 = zeros(size(obj.W2));
      obj.db1 = zeros(size(obj.b1));
      obj.db2 = zeros(size(obj.b2));
      obj.is_trained = false;
    end

    function fit(obj, X, Y)
    % Ajusta la red a los datos (X: N×n_in, Y: N×n_out).
      N = size(X, 1);
      if N < 2
        obj.is_trained = false;
        return;
      end
      % Normalización z-score
      obj.mu_in = mean(X, 1);
      obj.sd_in = max(std(X, 0, 1), 1e-6);
      obj.mu_out = mean(Y, 1);
      obj.sd_out = max(std(Y, 0, 1), 1e-6);
      Xn = (X - obj.mu_in) ./ obj.sd_in;
      Yn = (Y - obj.mu_out) ./ obj.sd_out;
      % Reiniciar incrementos de momento
      obj.dW1 = zeros(size(obj.W1));
      obj.dW2 = zeros(size(obj.W2));
      obj.db1 = zeros(size(obj.b1));
      obj.db2 = zeros(size(obj.b2));
      % Entrenamiento: full-batch gradient descent con momento.
      % (en línea con el espíritu del resto del proyecto: determinista,
      % reproducible y sin dependencias externas).
      for ep = 1:obj.epochs
        % Forward
        Z1 = obj.W1 * Xn.' + obj.b1;        % (n_hid, N)
        A1 = tanh(Z1);
        Yhat = obj.W2 * A1 + obj.b2;          % (n_out, N)
        % Backprop
        E = Yhat - Yn.';                     % (n_out, N)
        dW2 = E * A1.' / N;
        db2 = sum(E, 2) / N;
        dA1 = obj.W2.' * E;                  % (n_hid, N)
        dZ1 = dA1 .* (1 - A1.^2);            % derivada de tanh
        dW1 = dZ1 * Xn / N;
        db1 = sum(dZ1, 2) / N;
        % Actualización con momento
        obj.dW2 = obj.rho * obj.dW2 + (1 - obj.rho) * dW2;
        obj.db2 = obj.rho * obj.db2 + (1 - obj.rho) * db2;
        obj.dW1 = obj.rho * obj.dW1 + (1 - obj.rho) * dW1;
        obj.db1 = obj.rho * obj.db1 + (1 - obj.rho) * db1;
        obj.W2 = obj.W2 - obj.lr * obj.dW2;
        obj.b2 = obj.b2 - obj.lr * obj.db2;
        obj.W1 = obj.W1 - obj.lr * obj.dW1;
        obj.b1 = obj.b1 - obj.lr * obj.db1;
      end
      obj.is_trained = true;
    end

    function Yhat = predict(obj, X)
    % Predicción (N×n_out) para entradas X (N×n_in).
      if ~obj.is_trained
        % Si no se ha entrenado, devuelve la media (predicción trivial).
        Yhat = repmat(obj.mu_out, size(X, 1), 1);
        return;
      end
      Xn = (X - obj.mu_in) ./ obj.sd_in;
      A1 = tanh(obj.W1 * Xn.' + obj.b1);
      Yhatn = (obj.W2 * A1 + obj.b2).';
      Yhat = Yhatn .* obj.sd_out + obj.mu_out;
    end

    function yhat = predict_one(obj, x)
    % Predicción para un único vector (1×n_in).
      yhat = obj.predict(x(:).');
    end
  end
end
