%% @MultiObjAdaptiveCTC  —  AdaptiveCTC con NSGA-II (multi-objetivo).
% ------------------------------------------------------------------
% Variante del AdaptiveCTC que sustituye la sintonía single-objective
% (DE/GA/PSO/CMA minimizando ||y_d - ŷ||²) por una sintonía
% MULTI-OBJETIVO usando NSGA-II con tres objetivos:
%
%   J1(θ) = ISE_pred = ||y_d(t+s_f·Δt) - ŷ_θ(t+s_f·Δt)||²
%   J2(θ) = ||u_θ||²  (esfuerzo de control)
%   J3(θ) = Σ_j max(0, |u_j| - 0.95 τ_max,j)  (violación de saturación)
%
% NSGA-II devuelve un frente de Pareto; el tomador de decisiones
% escoge el "knee point" (punto de codo) como compromiso.
%
% En este proyecto, el "knee" se obtiene por la heurística de la
% distancia normalizada máxima: el punto cuyas distancias normalizadas
% a los extremos del frente son más dispares (método clásico de
% Branke et al. 2004).
%
% El resto de la mecánica (regresor, buffer, regla de aceptación
% conservadora) es idéntica a AdaptiveCTC.
% ------------------------------------------------------------------
classdef MultiObjAdaptiveCTC < AdaptiveCTC
  properties
    knee_history  % (n_retunes × 3) historial de los knees elegidos
  end
  methods
    function obj = MultiObjAdaptiveCTC(p_nom, traj, dt, Kp, Ki, Kd, ...
                                         bounds_lo, bounds_hi, ...
                                         degree, s_b, s_f, pop, gens, seed)
      if nargin < 9 || isempty(degree), degree = 2; end
      if nargin < 10 || isempty(s_b),  s_b = 10;   end
      if nargin < 11 || isempty(s_f),  s_f = 1;    end
      if nargin < 12 || isempty(pop),  pop = 20;   end
      if nargin < 13 || isempty(gens), gens = 20;  end
      if nargin < 14 || isempty(seed), seed = 0;   end
      obj = obj@AdaptiveCTC(p_nom, traj, dt, Kp, Ki, Kd, ...
                            bounds_lo, bounds_hi, 'DE', degree, ...
                            s_b, s_f, pop, gens, seed);
      obj.knee_history = zeros(0, 3);
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
      [qd_next, vd_next, ~] = obj.traj.eval(t + obj.dt * obj.s_f);
      yd_next = [qd_next(:); vd_next(:)];
      tau_max = obj.p.tau_max(:);

      % Función multi-objetivo (P,9) → (P,3)
      fmo = @(X) mo_batch_cost(X, obj, Omega, Mh, hb, ad, e, ed, ...
                                  int_e, qm, vm, yd_next, tau_max);
      % NSGA-II sobre la población actual en espacio normalizado [0, 1]^9
      warm = obj.x_prev;
      [Pareto, ~, ~] = nsga2(fmo, 9, zeros(1, 9), ones(1, 9), ...
                              obj.pop, obj.gens, ...
                              'warm', warm, 'seed', 0);
      % Tomar el knee del frente de Pareto
      F = fmo(Pareto);
      knee_idx = knee_point(F);
      x_knee = Pareto(knee_idx, :);

      % Regla de aceptación conservadora en J1
      J_cur = fmo(obj.x_prev);
      J_cur1 = J_cur(1, 1);
      J_knee1 = F(knee_idx, 1);
      if J_knee1 <= J_cur1
        obj.x_prev = x_knee(:).';
        obj.J_prev = J_knee1;
      else
        obj.J_prev = J_cur1;
      end

      theta = obj.denormalize(obj.x_prev);
      obj.Kp = theta(1:3);
      obj.Ki = theta(4:6);
      obj.Kd = theta(7:9);
      obj.first_retune = false;
      obj.t_adapt_total = obj.t_adapt_total + toc;
      obj.n_retunes     = obj.n_retunes + 1;
      obj.gain_history  = [obj.gain_history; t, theta];
      obj.knee_history  = [obj.knee_history; F(knee_idx, :)];
    end
  end
end

function F = mo_batch_cost(X, obj, Omega, Mh, hb, ad, e, ed, int_e, ...
                            qm, vm, yd_next, tau_max)
% Tres objetivos:
%   J1 = ||y_d - ŷ||²
%   J2 = ||u||²
%   J3 = saturación (penalización de par excedido)
  P = size(X, 1);
  theta = obj.lo + X .* (obj.hi - obj.lo);
  Kp = theta(:, 1:3); Ki = theta(:, 4:6); Kd = theta(:, 7:9);
  ar = repmat(ad(:).', P, 1) + Kp .* repmat(e(:).', P, 1) ...
        + Ki .* repmat(int_e(:).', P, 1) ...
        + Kd .* repmat(ed(:).', P, 1);
  U  = ar * Mh.' + repmat(hb(:).', P, 1);
  Usat = max(-tau_max.', U);  Usat = min(tau_max.', Usat);
  Z = [repmat(qm(:).', P, 1), repmat(vm(:).', P, 1), U];
  PhiX = monomials(Z, obj.degree);
  Yhat = PhiX * Omega;
  % Usar yd_next pasado como argumento (trayectoria en t + s_f*dt)
  E = repmat(yd_next(:).', P, 1) - Yhat;
  J1 = sum(E.^2, 2);
  J2 = sum(Usat.^2, 2);
  J3 = sum(max(0, abs(Usat) - 0.95 * tau_max.'), 2);
  F = [J1, J2, J3];
end

function k = knee_point(F)
% Heurística del "knee point": el punto cuyas distancias normalizadas
% a los extremos del frente son más dispares.
  if size(F, 1) == 1
    k = 1; return;
  end
  % Normalizar por columna
  Fmin = min(F, [], 1);
  Fmax = max(F, [], 1);
  rng = max(Fmax - Fmin, 1e-12);
  Fn = (F - Fmin) ./ rng;
  % Extremos del frente (en el primer y último objetivo)
  [~, i_min] = min(Fn(:, 1));
  [~, i_max] = min(Fn(:, end));
  d_min = sqrt(sum((Fn - repmat(Fn(i_min, :), size(F, 1), 1)).^2, 2));
  d_max = sqrt(sum((Fn - repmat(Fn(i_max, :), size(F, 1), 1)).^2, 2));
  knee_score = min(d_min, d_max);
  [~, k] = max(knee_score);
end
