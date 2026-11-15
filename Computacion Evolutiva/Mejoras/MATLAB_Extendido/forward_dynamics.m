%% forward_dynamics.m  —  a = M⁻¹ (τ + d − h − g − F).
% ------------------------------------------------------------------
% Dinámica directa: dado el par aplicado (más perturbación externa d)
% y el estado (q,v), devuelve la aceleración articular resultante.
% Resuelve el sistema lineal M(q)·a = rhs (M es simétrica definida
% positiva en todo el espacio de configuración salvo la singularidad
% de base, evitada por construcción de la trayectoria).
% ------------------------------------------------------------------
function a = forward_dynamics(p, q, v, tau, d)
  if nargin < 5 || isempty(d), d = zeros(3,1); end
  rhs = tau(:) + d(:) - bias_vector(p, q, v) ...
        - gravity_vector(p, q) - friction_vector(p, v);
  a = mass_matrix(p, q) \ rhs;
end
