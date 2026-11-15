%% @CTC_PID  —  Controlador de par calculado con corrección PID y
%               anti-windup condicional (consciente del par disponible).
% ------------------------------------------------------------------
% Estructura del control CTC-PID (Rodríguez-Molina et al., 2024,
% ecs. 9-10) extendida a 3 GDL:
%
%   τ = M̂(q)·(a_d + Kp·e + Ki·∫e·dt + Kd·ė) + ĉ(q,v) + ĝ(q) + F̂(v)
%
% Anti-windup condicional: el aporte del término integral se limita
% a aw_frac·τ_max en cada articulación, escalado por las ganancias
% vigentes (Ki es variable de decisión en el controlador adaptativo):
%
%   |M̂_jj · Ki_j · I_j| ≤ aw_frac · τ_max_j
%
% Sirve como controlador fijo (θ0 sintonizado offline) y como clase
% base del adaptativo (AdaptiveCTC hereda de CTC_PID).
% ------------------------------------------------------------------
classdef CTC_PID < handle
  properties
    p        % parámetros nominales (ArmParams)
    traj     % QuinticTrajectory
    dt       % período de muestreo [s]
    Kp       % vector 3 (ganancias proporcionales)
    Ki       % vector 3 (ganancias integrales)
    Kd       % vector 3 (ganancias derivativas)
    aw_frac  % fracción de τ_max asignable al término integral (0.5)
    int_e    % vector 3 (integral acumulada del error)
    % estado interno por ciclo (cache para ctc_law)
    qd_, vd_, ad_, Mh_
  end
  methods
    function obj = CTC_PID(p_nom, traj, dt, Kp, Ki, Kd, aw_frac)
      if nargin < 6 || isempty(aw_frac), aw_frac = 0.5; end
      obj.p       = p_nom;
      obj.traj    = traj;
      obj.dt      = dt;
      obj.Kp      = Kp(:).';
      obj.Ki      = Ki(:).';
      obj.Kd      = Kd(:).';
      obj.aw_frac = aw_frac;
      obj.int_e   = zeros(1, 3);
      obj.qd_ = zeros(3,1); obj.vd_ = zeros(3,1); obj.ad_ = zeros(3,1);
      obj.Mh_ = eye(3);
    end
    function reset(obj)
      % Reinicia la integral acumulada (anti-windup).
      obj.int_e = zeros(1, 3);
    end
    function integral_update(obj, e, ~)
    % Integral con anti-windup condicional.
      obj.int_e = obj.int_e + e(:).' * obj.dt;
      Mdg = diag(obj.Mh_);
      for j = 1:3
        if obj.Ki(j) > 1e-9
          lim = (obj.aw_frac * obj.p.tau_max(j)) ...
                / (max(Mdg(j), 1e-6) * obj.Ki(j));
          obj.int_e(j) = max(-lim, min(lim, obj.int_e(j)));
        end
      end
    end
    function tau = ctc_law(obj, qm, vm, Kp, Ki, Kd, int_e)
    % τ (sin saturar) con ganancias y estado actuales.
      qd = obj.qd_; vd = obj.vd_; ad = obj.ad_;
      e  = qd - qm;
      ed = vd - vm;
      ar = ad + Kp(:) .* e + Ki(:) .* int_e + Kd(:) .* ed;
      tau = obj.Mh_ * ar ...
            + bias_vector(obj.p, qm, vm) ...
            + gravity_vector(obj.p, qm) ...
            + friction_vector(obj.p, vm);
    end
    function tau = step(obj, t, qm, vm)
    % Ejecuta un ciclo del controlador y devuelve τ (sin saturar).
      [qd, vd, ad] = obj.traj.eval(t);
      obj.qd_ = qd(:); obj.vd_ = vd(:); obj.ad_ = ad(:);
      obj.Mh_ = mass_matrix(obj.p, qm);
      e = qd - qm;
      obj.integral_update(e, qm);
      tau = obj.ctc_law(qm, vm, obj.Kp, obj.Ki, obj.Kd, obj.int_e);
    end
  end
end
