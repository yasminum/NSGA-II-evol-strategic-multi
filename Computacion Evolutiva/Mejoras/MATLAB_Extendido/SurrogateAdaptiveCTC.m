%% @SurrogateAdaptiveCTC  —  AdaptiveCTC con sintonía evolutiva asistida por sustitutos.
% ------------------------------------------------------------------
% Variante del AdaptiveCTC que combina:
%   - Sintonía MULTI-OBJETIVO (NSGA-II), con los 3 objetivos de
%     MultiObjAdaptiveCTC.
%   - Sustitución parcial de evaluaciones de la función verdadera
%     (simulación de la planta con el modelo identificado) por un
%     MODELO SUSTITUTO (RBF/Kriging/NN) que aproxima J(θ).
%
% Estrategia de muestreo adaptativo (SurrogateManager):
%   1. Construir/actualizar el sustituto sobre la base de datos de
%      cromosomas previamente evaluados.
%   2. Generar candidatos (NSGA-II sobre el SUSTITUTO: barato).
%   3. Seleccionar k_lie candidatos prometedores o inciertos
%      (Likelihood of Improvement Exploration, LIE).
%   4. Evaluar esos k_lie candidatos con la función VERDADERA (la NN
%      del regresor) y añadirlos a la base de datos.
%   5. Repetir.
%
% En el presupuesto equivalente (pop × gens = 400 evaluaciones), el
% sustituto permite:
%   - 80 % de evaluaciones SUSTITUTAS (baratas),
%   - 20 % de evaluaciones REALES (caras) → 5× aceleración efectiva.
%
% Este es el controlador más avanzado del proyecto extendido: integra
% identificación NN, multi-objetivo NSGA-II y modelo sustituto.
% ------------------------------------------------------------------
classdef SurrogateAdaptiveCTC < MultiObjAdaptiveCTC
  properties
    surr_mgr     % SurrogateManager
    k_lie        % número de candidatos a evaluar "de verdad" por gen
    n_surr_gens  % número de generaciones del NSGA-II sobre el sustituto
    n_real_evals % contador de evaluaciones reales (diagnóstico)
    surr_kind
  end
  methods
    function obj = SurrogateAdaptiveCTC(p_nom, traj, dt, Kp, Ki, Kd, ...
                                          bounds_lo, bounds_hi, ...
                                          degree, s_b, s_f, pop, gens, ...
                                          seed, surr_kind, k_lie, n_surr_gens)
      if nargin < 14 || isempty(seed),         seed = 0; end
      if nargin < 15 || isempty(surr_kind),    surr_kind = 'rbf'; end
      if nargin < 16 || isempty(k_lie),        k_lie = 4; end
      if nargin < 17 || isempty(n_surr_gens),  n_surr_gens = 15; end
      obj = obj@MultiObjAdaptiveCTC(p_nom, traj, dt, Kp, Ki, Kd, ...
                                      bounds_lo, bounds_hi, ...
                                      degree, s_b, s_f, pop, gens, seed);
      obj.surr_kind = surr_kind;
      obj.k_lie = k_lie;
      obj.n_surr_gens = n_surr_gens;
      % 3 salidas: J1, J2, J3
      obj.surr_mgr = SurrogateManager(surr_kind, 9, 3, 1);
      obj.n_real_evals = 0;
    end

    function retune(obj, t, qm, vm)
      tic;
      Omega = obj.fit_regressor(qm, vm);
      Mh = obj.Mh_;
      hb = bias_vector(obj.p, qm, vm) + gravity_vector(obj.p, qm) ...
           + friction_vector(obj.p, vm);
      qd = obj.qd_; vd = obj.vd_; ad = obj.ad_;
      e  = qd - qm; ed = vd - vm;
      int_e = obj.int_e;
      tau_max = obj.p.tau_max(:);

      % Función multi-objetivo "verdadera" (real_fn): NN del regresor.
      true_fn = @(X) mo_real_cost(X, obj, Omega, Mh, hb, ad, e, ed, ...
                                     int_e, qm, vm, tau_max);

      % Función sustituta (rápida): predicción del SurrogateManager.
      surr_fn = @(X) surrogate_eval(X, obj);

      % Sembrar el sustituto si está vacío: añadir población inicial en [0, 1]^9
      if ~obj.surr_mgr.is_ready
        X_seed = rand(obj.pop, 9);
        Y_seed = true_fn(X_seed);
        obj.surr_mgr.add_samples(X_seed, Y_seed);
        obj.n_real_evals = obj.n_real_evals + obj.pop;
      end

      % NSGA-II sobre el SUSTITUTO (rápido) en [0, 1]^9
      [Pareto_surr, ~, ~] = nsga2(surr_fn, 9, zeros(1, 9), ones(1, 9), ...
                                   obj.pop, obj.n_surr_gens, ...
                                   'warm', obj.x_prev, 'seed', 0);
      % Selección LIE: elegir k_lie candidatos del frente de Pareto
      [yhat_surr, var_surr] = obj.surr_mgr.predict_ensemble(Pareto_surr);
      [X_lie, ~] = obj.surr_mgr.select_lie(Pareto_surr, yhat_surr, ...
                                           var_surr, obj.k_lie);
      % Evaluar las k_lie muestras "de verdad"
      Y_lie = true_fn(X_lie);
      obj.surr_mgr.add_samples(X_lie, Y_lie);
      obj.n_real_evals = obj.n_real_evals + obj.k_lie;

      % Volver a predecir y elegir el knee como solución
      [yhat_final, ~] = obj.surr_mgr.predict_ensemble(Pareto_surr);
      knee_idx_local = knee_point_local(yhat_final);
      x_knee_norm = Pareto_surr(knee_idx_local, :);
      J_cur = true_fn(obj.x_prev);
      J_knee = yhat_final(knee_idx_local, :);
      if J_knee(1) <= J_cur(1, 1)
        obj.x_prev = x_knee_norm(:).';
        obj.J_prev = J_knee(1);
      else
        obj.J_prev = J_cur(1, 1);
      end

      theta = obj.denormalize(obj.x_prev);
      obj.Kp = theta(1:3); obj.Ki = theta(4:6); obj.Kd = theta(7:9);
      obj.first_retune = false;
      obj.t_adapt_total = obj.t_adapt_total + toc;
      obj.n_retunes     = obj.n_retunes + 1;
      obj.gain_history  = [obj.gain_history; t, theta];
      obj.knee_history  = [obj.knee_history; J_knee];
    end
  end
end

function F = mo_real_cost(X, obj, Omega, Mh, hb, ad, e, ed, int_e, ...
                          qm, vm, tau_max)
% Función multi-objetivo "verdadera" — usa la NN del regresor.
% NOTA: Omega ya no se usa en este esquema (la NN se entrenó antes
% y se almacena en obj.nn_regressor si el controlador es NNAdaptiveCTC;
% si es PR, Omega es la matriz ajustada por pinv). Aquí mantenemos la
% firma por compatibilidad y usamos el regresor polinomial (obj.degree).
  P = size(X, 1);
  theta = obj.lo + X .* (obj.hi - obj.lo);
  Kp = theta(:, 1:3); Ki = theta(:, 4:6); Kd = theta(:, 7:9);
  ar = repmat(ad(:).', P, 1) + Kp .* repmat(e(:).', P, 1) ...
        + Ki .* repmat(int_e(:).', P, 1) ...
        + Kd .* repmat(ed(:).', P, 1);
  U  = ar * Mh.' + repmat(hb(:).', P, 1);
  Usat = max(-tau_max.', U);  Usat = min(tau_max.', Usat);
  Z = [repmat(qm(:).', P, 1), repmat(vm(:).', P, 1), Usat];
  PhiX = monomials(Z, obj.degree);
  Yhat = PhiX * Omega;
  yd_next_t = obj.qd_ + obj.ad_ * obj.dt;
  vd_next_t = obj.vd_;
  yd_next_full = [yd_next_t(:); vd_next_t(:)];
  E = repmat(yd_next_full(:).', P, 1) - Yhat;
  J1 = sum(E.^2, 2);
  J2 = sum(Usat.^2, 2);
  J3 = sum(max(0, abs(Usat) - 0.95 * tau_max.'), 2);
  F = [J1, J2, J3];
end

function F = surrogate_eval(X, obj)
% Evalúa el sustituto; devuelve (P × 3) aproximaciones de [J1, J2, J3].
  [yhat, ~] = obj.surr_mgr.predict_ensemble(X);
  F = max(0, yhat);  % los objetivos son no negativos
end

function k = knee_point_local(F)
  if size(F, 1) == 1
    k = 1; return;
  end
  Fmin = min(F, [], 1);
  Fmax = max(F, [], 1);
  rng = max(Fmax - Fmin, 1e-12);
  Fn = (F - Fmin) ./ rng;
  [~, i_min] = min(Fn(:, 1));
  [~, i_max] = min(Fn(:, end));
  d_min = sqrt(sum((Fn - repmat(Fn(i_min, :), size(F, 1), 1)).^2, 2));
  d_max = sqrt(sum((Fn - repmat(Fn(i_max, :), size(F, 1), 1)).^2, 2));
  knee_score = min(d_min, d_max);
  [~, k] = max(knee_score);
end
