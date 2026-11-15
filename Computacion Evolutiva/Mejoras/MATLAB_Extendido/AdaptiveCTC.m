%% @AdaptiveCTC  —  Control adaptativo indirecto (regresor + evolutivo online).
% ------------------------------------------------------------------
% Arquitectura por ciclo de muestreo Δt (control digital):
%   1. Medir q(t), v(t) (con ruido según escenario).
%   2. Si la memoria tiene al menos s_b muestras:
%        a) IDENTIFICACIÓN: ajustar un regresor multi-salida (PR grado
%           2 o SLR) por mínimos cuadrados en FORMA CERRADA sobre la
%           ventana retrospectiva de s_b muestras:
%              características x = [q, v, u] ∈ R⁹
%              objetivos      y = [q(t+Δt), v(t+Δt)] ∈ R⁶
%        b) SINTONIZACIÓN: resolver el problema (19) del paper con un
%           algoritmo evolutivo ONLINE (ODE/OGA/OPSO/OCMA, 400 evaluac.):
%              min_θ  J(θ) = ‖y_d(t+Δt) − ŷ_θ(t+Δt)‖²
%           donde la acción de control candidata u_θ se calcula con
%           el CTC-PID y el modelo NOMINAL, y ŷ_θ es la predicción
%           del regresor. Regla de aceptación conservadora:
%              se aplica el nuevo θ sólo si J(θ*) ≤ J(θ_actual).
%   3. Aplicar τ = M̂(q)(a_d + Kp·e + Ki·∫e + Kd·ė) + ĉ + ĝ + F̂
%      (saturado a ±τ_max).
%
% Cromosoma θ = [Kp1..Kp3, Ki1..Ki3, Kd1..Kd3] ∈ R⁹ con cotas por
% ganancia (amplias para Kp y Ki, estrechas para Kd, criterio del
% paper por la alta sensibilidad de Kd).
%
% Valores por defecto del paper base:
%   s_b = 10  pasos retrospectivos de identificación
%   s_f = 1   paso(s) de predicción hacia adelante
%   NP = 20, Gmax = 20  → 400 evaluaciones por ciclo (presupuesto equiv.)
% ------------------------------------------------------------------
classdef AdaptiveCTC < CTC_PID
  properties
    opt_fn     % function handle del optimizador evolutivo
    opt_name   % 'DE' | 'GA' | 'PSO' | 'CMA'
    degree     % grado del regresor (1 = SLR, 2 = PR)
    s_b        % ventana retrospectiva
    s_f        % horizonte de predicción
    pop        % tamaño de población
    gens       % número de generaciones
    lo, hi     % cotas físicas (1×9) por ganancia
    buf_q      % cell array de estados medidos (1×3)
    buf_v      % cell array de velocidades medidas
    buf_u      % cell array de acciones (saturadas) aplicadas
    x_prev     % mejor cromosoma normalizado [0,1]^9 del ciclo anterior
    J_prev     % costo asociado al ciclo anterior
    first_retune  % true hasta el primer retune
    gain_history  % (n_retunes × 10) [t, Kp1..3, Ki1..3, Kd1..3]
    n_retunes     % número de retunes ejecutados
    t_adapt_total % s acumulados de cómputo de adaptación
  end
  methods
    function obj = AdaptiveCTC(p_nom, traj, dt, Kp, Ki, Kd, ...
                                bounds_lo, bounds_hi, optimizer, ...
                                degree, s_b, s_f, pop, gens, seed)
      if nargin < 10 || isempty(s_b),  s_b = 10;   end
      if nargin < 11 || isempty(s_f),  s_f = 1;    end
      if nargin < 12 || isempty(pop),  pop = 20;   end
      if nargin < 13 || isempty(gens), gens = 20;  end
      if nargin < 14 || isempty(seed), seed = 0;   end
      if nargin <  8 || isempty(optimizer), optimizer = 'DE';  end
      if nargin <  9 || isempty(degree),    degree = 2;         end

      obj = obj@CTC_PID(p_nom, traj, dt, Kp, Ki, Kd);
      obj.opt_fn    = get_optimizer(optimizer);
      obj.opt_name  = upper(optimizer);
      obj.degree    = degree;
      obj.s_b       = s_b;
      obj.s_f       = s_f;
      obj.pop       = pop;
      obj.gens      = gens;
      rng(seed);
      obj.lo        = bounds_lo(:).';
      obj.hi        = bounds_hi(:).';
      obj.buf_q     = {};
      obj.buf_v     = {};
      obj.buf_u     = {};
      obj.x_prev    = obj.normalize([obj.Kp, obj.Ki, obj.Kd]);
      obj.J_prev    = inf;
      obj.first_retune = true;
      obj.gain_history  = zeros(0, 10);
      obj.n_retunes     = 0;
      obj.t_adapt_total = 0.0;
    end

    function x = normalize(obj, theta)
      x = (theta(:).' - obj.lo) ./ (obj.hi - obj.lo);
    end
    function theta = denormalize(obj, x)
      theta = obj.lo + max(0, min(1, x(:).')) .* (obj.hi - obj.lo);
    end

    function Omega = fit_regressor(obj, qm, vm)
      s = obj.s_b;
      Q = cell2mat(obj.buf_q(end-s+1:end).');     % (s,3)
      V = cell2mat(obj.buf_v(end-s+1:end).');     % (s,3)
      U = cell2mat(obj.buf_u(end-s+1:end).');     % (s,3)
      Z = [Q, V, U];                              % (s,9)
      % objetivos: estados siguientes; la última fila es el estado actual
      Yq = [cell2mat(obj.buf_q(end-s+2:end).'); qm(:).'];   % (s,3)
      Yv = [cell2mat(obj.buf_v(end-s+2:end).'); vm(:).'];   % (s,3)
      Yt = [Yq, Yv];                                          % (s,6)
      Phi = monomials(Z, obj.degree);                         % (s,p)
      Omega = pinv(Phi) * Yt;   % mínima norma (p > s)
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
      yd_next = [qd_next(:); vd_next(:)];       % (6,1)
      tau_max = obj.p.tau_max(:);

      fbatch = @(X) batch_cost(X, obj, Omega, Mh, hb, ad, e, ed, ...
                                 int_e, qm, vm, yd_next, tau_max);

      J_cur = fbatch(obj.x_prev);               % evaluar las ganancias actuales
      if obj.first_retune
        warm = [];
      else
        warm = obj.x_prev;
      end
      [x_best, J_best] = obj.opt_fn(fbatch, 9, warm, obj.pop, obj.gens);

      if J_best <= J_cur
        obj.x_prev = x_best(:).';
        obj.J_prev = J_best;
      else
        obj.J_prev = J_cur;
      end

      theta = obj.denormalize(obj.x_prev);
      obj.Kp = theta(1:3);
      obj.Ki = theta(4:6);
      obj.Kd = theta(7:9);
      obj.first_retune = false;
      obj.t_adapt_total = obj.t_adapt_total + toc;
      obj.n_retunes     = obj.n_retunes + 1;
      obj.gain_history  = [obj.gain_history; t, theta]; %#ok<AGROW>
    end

    function tau = step(obj, t, qm, vm)
      [qd, vd, ad] = obj.traj.eval(t);
      obj.qd_ = qd(:); obj.vd_ = vd(:); obj.ad_ = ad(:);
      obj.Mh_ = mass_matrix(obj.p, qm);
      e = qd - qm;
      obj.integral_update(e, qm);

      if numel(obj.buf_q) >= obj.s_b
        obj.retune(t, qm, vm);
      end

      tau = obj.ctc_law(qm, vm, obj.Kp, obj.Ki, obj.Kd, obj.int_e);
      % almacenar la acción SATURADA (la que realmente ve la planta)
      tau_max = obj.p.tau_max(:);
      tau_sat = tau_max .* max(-1, min(1, tau ./ tau_max));
      obj.buf_q{end+1} = qm(:).';
      obj.buf_v{end+1} = vm(:).';
      obj.buf_u{end+1} = tau_sat(:).';
    end
  end
end

function J = batch_cost(X, obj, Omega, Mh, hb, ad, e, ed, int_e, ...
                         qm, vm, yd_next, tau_max)
% fbatch(X): dado un lote (P,9) de cromosomas normalizados, devuelve
% J(P,) = ||y_d(t+Δt) − ŷ_θ(t+Δt)||² para cada uno.
  P = size(X, 1);
  theta = obj.lo + X .* (obj.hi - obj.lo);  % (P,9)
  Kp = theta(:, 1:3);
  Ki = theta(:, 4:6);
  Kd = theta(:, 7:9);
  ar = repmat(ad(:).', P, 1) + Kp .* repmat(e(:).', P, 1) ...
        + Ki .* repmat(int_e(:).', P, 1) ...
        + Kd .* repmat(ed(:).', P, 1);     % (P,3)
  U  = ar * Mh.' + repmat(hb(:).', P, 1);   % (P,3)
  U  = max(-tau_max.', U);  U = min(tau_max.', U);  % saturación
  Z = [repmat(qm(:).', P, 1), repmat(vm(:).', P, 1), U];  % (P,9)
  PhiX = monomials(Z, obj.degree);          % (P,p)
  Yhat = PhiX * Omega;                       % (P,6)
  E = repmat(yd_next(:).', P, 1) - Yhat;
  J = sum(E.^2, 2);
end
