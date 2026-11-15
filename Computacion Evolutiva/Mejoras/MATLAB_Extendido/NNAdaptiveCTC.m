%% @NNAdaptiveCTC  —  AdaptiveCTC con regresor NN en lugar de polinomios.
% ------------------------------------------------------------------
% Variante del AdaptiveCTC que sustituye el regresor monomial por una
% red neuronal (NNRegressor). Se conservan el resto de elementos del
% esquema adaptativo (regresor sobre ventana s_b, sintonía evolutiva
% por presupuesto equivalente, regla de aceptación conservadora).
%
% Motivación: el regresor polinomial (PR grado 2) tiene 55 parámetros
% para 9 entradas; esto es flexible pero propenso a overfitting en
% ventanas cortas (s_b = 10). La NN tiene arquitectura más parsimoniosa
% (9 → 16 → 6 = 256 parámetros) pero con un regularizador implícito
% (z-score + momento + early-stop suave). Comparar PR vs NN es el
% primer experimento de "modelos sustitutos" del proyecto extendido.
%
% Implementación: hereda de AdaptiveCTC pero sobreescribe `retune`
% para usar la NN directamente en el cálculo del costo (sin pasar por
% `monomials` ni por la matriz `Omega`).
% ------------------------------------------------------------------
classdef NNAdaptiveCTC < AdaptiveCTC
  properties
    nn_regressor  % instancia de NNRegressor
  end
  methods
    function obj = NNAdaptiveCTC(p_nom, traj, dt, Kp, Ki, Kd, ...
                                   bounds_lo, bounds_hi, optimizer, ...
                                   s_b, s_f, pop, gens, seed)
      if nargin < 10 || isempty(s_b),  s_b = 20;   end
      if nargin < 11 || isempty(s_f),  s_f = 1;    end
      if nargin < 12 || isempty(pop),  pop = 20;   end
      if nargin < 13 || isempty(gens), gens = 20;  end
      if nargin < 14 || isempty(seed), seed = 0;   end
      % Llamar al constructor base con degree = 2 (placeholder), s_b = max(s_b, 20)
      obj = obj@AdaptiveCTC(p_nom, traj, dt, Kp, Ki, Kd, ...
                            bounds_lo, bounds_hi, optimizer, 2, ...
                            max(s_b, 20), s_f, pop, gens, seed);
      % Inicializar la red neuronal regresora
      obj.nn_regressor = NNRegressor(9, 6, 16, 0.01, 200, 0.9);
    end

    function retune(obj, t, qm, vm)
    % Sintonía evolutiva con el costo evaluado por la NN del regresor.
      tic;
      s = obj.s_b;
      Q = cell2mat(obj.buf_q(end-s+1:end).');     % (s,3)
      V = cell2mat(obj.buf_v(end-s+1:end).');     % (s,3)
      U = cell2mat(obj.buf_u(end-s+1:end).');     % (s,3)
      Z = [Q, V, U];                              % (s,9)
      Yq = [cell2mat(obj.buf_q(end-s+2:end).'); qm(:).'];
      Yv = [cell2mat(obj.buf_v(end-s+2:end).'); vm(:).'];
      Yt = [Yq, Yv];
      % Entrenar la NN sobre la ventana deslizante
      obj.nn_regressor.fit(Z, Yt);

      % Configurar el resto del contexto (igual que AdaptiveCTC.retune)
      Mh = obj.Mh_;
      hb = bias_vector(obj.p, qm, vm) + gravity_vector(obj.p, qm) ...
           + friction_vector(obj.p, vm);
      qd = obj.qd_; vd = obj.vd_; ad = obj.ad_;
      e  = qd - qm; ed = vd - vm;
      int_e = obj.int_e;
      tau_max = obj.p.tau_max(:);

      % Función de costo basada en la NN (no en monomials)
      fbatch = @(X) nn_batch_cost(X, obj, Mh, hb, ad, e, ed, ...
                                     int_e, qm, vm, tau_max);
      J_cur = fbatch(obj.x_prev);
      if obj.first_retune, warm = []; else, warm = obj.x_prev; end
      [x_best, J_best] = obj.opt_fn(fbatch, 9, warm, obj.pop, obj.gens);
      if J_best <= J_cur
        obj.x_prev = x_best(:).';
        obj.J_prev = J_best;
      else
        obj.J_prev = J_cur;
      end
      theta = obj.denormalize(obj.x_prev);
      obj.Kp = theta(1:3); obj.Ki = theta(4:6); obj.Kd = theta(7:9);
      obj.first_retune = false;
      obj.t_adapt_total = obj.t_adapt_total + toc;
      obj.n_retunes     = obj.n_retunes + 1;
      obj.gain_history  = [obj.gain_history; t, theta];
    end
  end
end

function J = nn_batch_cost(X, obj, Mh, hb, ad, e, ed, int_e, qm, vm, tau_max)
% Costo evaluado con la NN del regresor en lugar de monomials.
  P = size(X, 1);
  theta = obj.lo + X .* (obj.hi - obj.lo);
  Kp = theta(:, 1:3); Ki = theta(:, 4:6); Kd = theta(:, 7:9);
  ar = repmat(ad(:).', P, 1) + Kp .* repmat(e(:).', P, 1) ...
        + Ki .* repmat(int_e(:).', P, 1) ...
        + Kd .* repmat(ed(:).', P, 1);
  U  = ar * Mh.' + repmat(hb(:).', P, 1);
  U  = max(-tau_max.', U);  U = min(tau_max.', U);
  % Características de entrada a la NN: [q, v, u] (P,9)
  Z = [repmat(qm(:).', P, 1), repmat(vm(:).', P, 1), U];
  Yhat = obj.nn_regressor.predict(Z);       % (P,6)
  % Reconstruir el objetivo siguiente estado (mismo enfoque que
  % MultiObjAdaptiveCTC, usando el estado deseado extrapolado un paso).
  qd_next_t = obj.qd_ + obj.ad_ * obj.dt;
  vd_next_t = obj.vd_;
  yd_next_t = [qd_next_t(:); vd_next_t(:)];
  E = repmat(yd_next_t(:).', P, 1) - Yhat;
  J = sum(E.^2, 2);
end
