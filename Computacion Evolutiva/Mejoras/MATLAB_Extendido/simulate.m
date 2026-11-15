%% simulate.m  —  Simulación en lazo cerrado (ZOH + RK4 con sub-pasos).
% ------------------------------------------------------------------
% El controlador se ejecuta una vez por ciclo de muestreo Δt (retención
% de orden cero) sobre estados MEDIDOS (con ruido según escenario);
% la planta se integra con RK4 usando n_sub sub-pasos por ciclo.
%
% El sub-paso (Δt/n_sub = 1 ms con los valores por defecto) es
% necesario porque la relación fricción-viscosa/inercia del brazo
% (hasta ≈155 s⁻¹ en la articulación 3 con los parámetros de la tesis)
% hace rígida la dinámica rápida; con sub-pasos de 1 ms el integrador
% RK4 es preciso y estable, mientras que el período de CONTROL se
% mantiene en Δt = 5 ms, idéntico al del paper base.
%
% Parámetros:
%   p_plant     struct de parámetros de la planta (con incertidumbre)
%   controller  objeto con métodos .reset() y .step(t,qm,vm) -> tau
%   traj        QuinticTrajectory
%   T           duración de la simulación [s]
%   dt          período de muestreo del control [s]
%
% Opcionales:
%   seed        semilla del generador de ruido
%   noise       struct con campos .q y .v (amplitud del ruido uniforme)
%   dist        function handle t -> d(t) (3×1)
%   n_sub       sub-pasos RK4 por ciclo
%
% Devuelve logs con campos:
%   t (N+1,1), q (N+1,3), v (N+1,3), tau (N+1,3), qd (N+1,3),
%   vd (N+1,3), sat (N+1,1), e (N+1,3)
%
% Nota: la API rng() de MATLAB usa un generador distinto al de NumPy;
% los números aleatorios específicos DIFERIRÁN entre las versiones
% Python y MATLAB, pero el experimento estadístico (semillas 1..30,
% common random numbers) preserva la validez de las comparaciones.
% ------------------------------------------------------------------
function logs = simulate(p_plant, controller, traj, T, dt, ...
                         seed, noise, dist, n_sub)
  if nargin < 6 || isempty(seed),  seed = 0;    end
  if nargin < 7 || isempty(noise), noise = struct('q',0,'v',0); end
  if nargin < 8 || isempty(dist),  dist = [];   end
  if nargin < 9 || isempty(n_sub), n_sub = 5;   end

  rng(seed);  % MATLAB/Octave compatible (generador mt19937ar por defecto)
  N = round(T / dt);
  h = dt / n_sub;
  aq = noise.q; av = noise.v;

  q = traj.wp(1,:).';     % estado inicial = primer waypoint
  v = zeros(3,1);

  logs.t   = zeros(N+1, 1);
  logs.q   = zeros(N+1, 3);
  logs.v   = zeros(N+1, 3);
  logs.tau = zeros(N+1, 3);
  logs.qd  = zeros(N+1, 3);
  logs.vd  = zeros(N+1, 3);
  logs.sat = zeros(N+1, 1);

  controller.reset();

  for k = 1:(N+1)
    t = (k-1) * dt;
    [qd, vd, ad] = traj.eval(t);
    qd = qd(:); vd = vd(:); ad = ad(:);
    if aq > 0
      qm = q + (-aq + 2*aq*rand(3,1));
    else
      qm = q;
    end
    if av > 0
      vm = v + (-av + 2*av*rand(3,1));
    else
      vm = v;
    end

    tau_raw = controller.step(t, qm, vm);
    % saturación simétrica ±tau_max (vectorizada)
    taumax  = p_plant.tau_max(:);
    tau     = taumax .* max(-1, min(1, tau_raw ./ taumax));

    logs.t(k)   = t;
    logs.q(k,:) = q(:).';
    logs.v(k,:) = v(:).';
    logs.tau(k,:) = tau(:).';
    logs.qd(k,:) = qd(:).';
    logs.vd(k,:) = vd(:).';
    logs.sat(k) = any(abs(tau) >= 0.999 * taumax);

    if k == N+1, break; end

    % ---- RK4 con n_sub sub-pasos; tau fijo entre sub-pasos (ZOH) ----
    for s = 1:n_sub
      x0 = [q; v];
      ts = t + (s-1) * h;
      k1 = arm_deriv(p_plant, ts,     x0,            tau, dist);
      k2 = arm_deriv(p_plant, ts+h/2, x0 + h/2 * k1, tau, dist);
      k3 = arm_deriv(p_plant, ts+h/2, x0 + h/2 * k2, tau, dist);
      k4 = arm_deriv(p_plant, ts+h,   x0 + h   * k3, tau, dist);
      x  = x0 + h/6 * (k1 + 2*k2 + 2*k3 + k4);
      q = x(1:3); v = x(4:6);
    end
  end
  logs.e = logs.qd - logs.q;
end

function dx = arm_deriv(p, t, x, tau, dist)
  q = x(1:3); v = x(4:6);
  if isempty(dist)
    d = zeros(3,1);
  else
    d = dist(t);
  end
  a = forward_dynamics(p, q, v, tau, d);
  dx = [v; a];
end
