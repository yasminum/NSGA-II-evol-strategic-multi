%% demo_dynamics.m  —  Verifica la dinámica del brazo sin controlador.
% ------------------------------------------------------------------
% Tres pruebas de sanidad numérica (sin controlador, sin adaptación):
%   1) Conservación de la energía en lazo libre (sin gravedad, sin
%      fricción, sin perturbación): E = T + V debe ser constante.
%   2) Cancelación exacta del CTC: si se aplica el par calculado por
%      inverse_dynamics() con la aceleración deseada ad y los
%      parámetros NOMINALES, la planta (sin incertidumbre) sigue
%      exactamente la aceleración comandada.
%   3) Manipulabilidad a lo largo de la trayectoria: debe mantenerse
%      > 0 (sin pasar por singularidad).
%
% Uso:
%   >> demo_dynamics
% ------------------------------------------------------------------
function demo_dynamics()
  fprintf('=== Verificacion de la dinamica del brazo RRR ===\n');
  p = arm_params();

  % 1) Conservacion de energia: sin gravedad, sin friccion
  p_ng = p; p_ng.G = 0; p_ng.Fv = [0 0 0]; p_ng.Fc = [0 0 0];
  q  = [0.5; 0.4; 0.3]; v = [0.1; -0.05; 0.2];
  M  = mass_matrix(p_ng, q);
  T0 = 0.5 * v.' * M * v;            % energia cinetica
  dt = 1e-4; N = 1000;
  E = zeros(N, 1); E(1) = T0;
  for k = 2:N
    a = forward_dynamics(p_ng, q, v, zeros(3,1), zeros(3,1));
    v = v + a * dt;                   % Euler explicito (suficiente a dt=1e-4)
    q = q + v * dt;
    E(k) = 0.5 * v.' * mass_matrix(p_ng, q) * v;
  end
  dE = (E(end) - E(1)) / E(1);
  fprintf('1) Energia: E0=%.4e, E_final=%.4e, drift=%.2e (%.1f ppm)\n', ...
          E(1), E(end), dE, abs(dE)*1e6);
  if abs(dE) > 1e-3
    warning('  No se conserva la energia; revisar M(q) y h(q,v).');
  else
    fprintf('   OK (conservacion dentro de 0.1%%)\n');
  end

  % 2) Cancelacion exacta del CTC en planta nominal
  q = [0.5; 0.4; 0.3]; v = [0.1; 0.05; -0.05];
  a_des = [0.3; -0.2; 0.4];
  tau = inverse_dynamics(p, q, v, a_des);
  a_res = forward_dynamics(p, q, v, tau, zeros(3,1));
  err = max(abs(a_res - a_des));
  fprintf('2) Cancelacion CTC: ||a_res - a_des||_inf = %.2e\n', err);
  if err > 1e-6
    warning('  La cancelacion no es exacta; revisar signos.');
  else
    fprintf('   OK (error < 1e-6)\n');
  end

  % 3) Manipulabilidad a lo largo de la trayectoria
  tr = default_trajectory();
  N = 200;
  w = zeros(N, 1);
  for k = 1:N
    t = (k-1) / (N-1) * 5.0;
    [qd, ~, ~] = tr.eval(t);
    w(k) = manipulability(p, qd);
  end
  fprintf('3) Manipulabilidad: min=%.4f, max=%.4f, media=%.4f\n', ...
          min(w), max(w), mean(w));
  if min(w) > 1e-3
    fprintf('   OK (sin singularidad en la trayectoria)\n');
  else
    warning('  La trayectoria pasa cerca de una singularidad.');
  end
end
