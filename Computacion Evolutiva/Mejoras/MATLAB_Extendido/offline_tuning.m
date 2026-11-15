%% offline_tuning.m  —  Sintonía offline del CTC fijo (θ0) con ODE.
% ------------------------------------------------------------------
% Resuelve, en el escenario IDEAL (planta sin incertidumbres, sin
% Coulomb), el problema de sintonización del CTC-PID fijo mediante
% ODE con cotas amplias:
%
%   Kp ∈ [50, 900]   Ki ∈ [0, 400]   Kd ∈ [10, 150]
%
% La función objetivo es el ISE sobre una corrida corta (T = 2.5 s)
% con control ZOH y 3 sub-pasos RK4.
%
% Verificación de amortiguamiento: cada ζ_j = Kd_j / (2·√Kp_j) se
% fuerza al intervalo [0.8, 1.2] (sobreamortiguado suave), modificando
% Kd si es necesario. Esto evita ganancias con respuesta oscilatoria.
%
% Sintaxis:
%   [theta0, ISE_off] = offline_tuning(traj, pop, gens, T, seed)
%
% En el repositorio MATLAB el resultado se cachea en
%   results/offline_gains.mat  (struct con .theta0 y .ISE_offline)
% para no recalcular en cada ejecución del experimento completo.
% ------------------------------------------------------------------
function [theta0, ISE_off] = offline_tuning(traj, pop, gens, T, seed)
  if nargin < 2 || isempty(pop),  pop = 20;  end
  if nargin < 3 || isempty(gens), gens = 30; end
  if nargin < 4 || isempty(T),    T = 2.5;   end
  if nargin < 5 || isempty(seed), seed = 42; end

  p_nom = arm_params();
  lo = [50.0 50.0 50.0  0.0 0.0 0.0  10.0 10.0 10.0];
  hi = [900.0 900.0 900.0  400.0 400.0 400.0  150.0 150.0 150.0];
  DT = 0.005;

  fbatch = @(X) batch_offline(X, p_nom, traj, DT, T, lo, hi);
  rng(seed);
  [xb, fb] = ode_opt(fbatch, 9, [], pop, gens);
  theta0 = lo + xb .* (hi - lo);
  % verificación de amortiguamiento ζ ∈ [0.8, 1.2]
  for j = 1:3
    zeta = theta0(6 + j) / (2.0 * sqrt(theta0(j)));
    if zeta < 0.8 || zeta > 1.2
      theta0(6 + j) = 2.0 * sqrt(theta0(j));
    end
  end
  ISE_off = fb;
end

function J = batch_offline(X, p_nom, traj, DT, T, lo, hi)
  P = size(X, 1);
  J = zeros(P, 1);
  for i = 1:P
    th = lo + X(i,:) .* (hi - lo);
    ctrl = CTC_PID(p_nom, traj, DT, th(1:3), th(4:6), th(7:9));
    logs = simulate(p_nom, ctrl, traj, T, DT, 0, struct('q',0,'v',0), [], 3);
    J(i) = DT * sum(logs.e(:).^2);
  end
end
