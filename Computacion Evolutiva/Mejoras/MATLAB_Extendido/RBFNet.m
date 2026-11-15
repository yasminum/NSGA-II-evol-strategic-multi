%% @RBFNet  —  Red de Base Radial (RBF) para modelado sustitutivo.
% ------------------------------------------------------------------
% RBF con kernel gaussiano y pesos de salida por mínimos cuadrados.
% Diseñada para actuar como MODELO SUSTITUTO del costo cerrado de un
% controlador adaptativo, permitiendo evaluar candidatos de sintonía
% sin necesidad de simular la planta completa.
%
%   f(x) = Σ_k w_k · exp(-||x - c_k||^2 / (2 σ_k^2))
%
% Los centros {c_k} se eligen por muestreo aleatorio del dataset (o
% k-means si está disponible); los anchos σ_k son la distancia media
% al k-vecino más cercano; los pesos w_k se ajustan por mínimos
% cuadrados regularizados (ridge).
%
% API compatible con NNRegressor: .fit(X,Y), .predict(X), .predict_one(x).
% ------------------------------------------------------------------
classdef RBFNet < handle
  properties
    n_in       % dimensionalidad de entrada
    n_out      % dimensionalidad de salida
    n_centers  % número de centros RBF
    C          % matriz de centros (n_centers × n_in)
    sigma      % vector de anchos (n_centers × 1)
    W          % matriz de pesos de salida (n_centers × n_out)
    ridge      % regularización L2 (1e-3 por defecto)
    is_trained
  end
  methods
    function obj = RBFNet(n_in, n_out, n_centers, ridge)
      if nargin < 1 || isempty(n_in),  n_in  = 9;  end
      if nargin < 2 || isempty(n_out), n_out = 1;  end
      if nargin < 3 || isempty(n_centers), n_centers = 32; end
      if nargin < 4 || isempty(ridge), ridge = 1e-3; end
      obj.n_in = n_in;
      obj.n_out = n_out;
      obj.n_centers = n_centers;
      obj.ridge = ridge;
      obj.C = zeros(n_centers, n_in);
      obj.sigma = ones(n_centers, 1);
      obj.W = zeros(n_centers, n_out);
      obj.is_trained = false;
    end

    function fit(obj, X, Y)
      N = size(X, 1);
      M = min(obj.n_centers, N);
      if N < 2
        obj.is_trained = false;
        return;
      end
      % Selección de centros: muestreo uniforme sin reemplazo del dataset.
      idx = randperm(N, M);
      obj.C = X(idx, :);
      obj.n_centers = M;
      % Anchos: distancia media al 1-vecino más cercano entre centros
      % (estimador no paramétrico clásico para RBF, sin dependencias
      % externas como Statistics Toolbox).
      if M > 1
        D = pairwise_dist(obj.C, obj.C);
        D(1:M+1:end) = inf;   % eliminar diagonal
        d_nn = min(D, [], 2);
        d_nn(d_nn < 1e-6) = 1e-6;
        obj.sigma = d_nn;
      else
        obj.sigma = 1;
      end
      % Matriz de diseño Phi (N × M)
      Phi = rbf_design(X, obj.C, obj.sigma);
      % Mínimos cuadrados ridge: w = (Phi' Phi + λ I)^-1 Phi' Y
      A = Phi' * Phi + obj.ridge * eye(M);
      B = Phi' * Y;
      obj.W = A \ B;
      obj.is_trained = true;
    end

    function Yhat = predict(obj, X)
      if ~obj.is_trained
        Yhat = zeros(size(X, 1), obj.n_out);
        return;
      end
      Phi = rbf_design(X, obj.C, obj.sigma);
      Yhat = Phi * obj.W;
    end

    function yhat = predict_one(obj, x)
      yhat = obj.predict(x(:).');
    end

    function [yhat, var] = predict_var(obj, X)
    % Predicción con varianza estimada: para RBF, usamos la distancia
    % mínima al centro más cercano como indicador de incertidumbre
    % (a más distancia, menos confianza).
      if ~obj.is_trained
        yhat = zeros(size(X, 1), obj.n_out);
        var = 1e6 * ones(size(X, 1), 1);
        return;
      end
      Phi = rbf_design(X, obj.C, obj.sigma);
      yhat = Phi * obj.W;
      % var: inverso de la activación máxima (a mayor activación,
      % más cerca de un centro → menor varianza epistémica).
      var = 1 ./ (max(Phi, [], 2) + 1e-6);
    end
  end
end

function Phi = rbf_design(X, C, sigma)
% Matriz de diseño RBF: (N × M) con Φ_ik = exp(-||x_i - c_k||^2/(2 σ_k^2)).
  D2 = (sum(X.^2, 2)) ...
       + sum(C.^2, 2).' ...
       - 2 * (X * C.');
  Phi = exp(-D2 ./ (2 * sigma.^2.'));
end

function D = pairwise_dist(A, B)
% Distancias Euclidianas entre filas de A y filas de B (sin toolbox).
  D2 = (sum(A.^2, 2)) ...
       + sum(B.^2, 2).' ...
       - 2 * (A * B.');
  D = sqrt(max(0, D2));
end
