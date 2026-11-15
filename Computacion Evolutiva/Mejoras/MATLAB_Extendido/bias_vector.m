%% bias_vector.m  —  h(q,v) = C(q,v)·v  vía identidad de Lagrange.
% ------------------------------------------------------------------
% Cálculo del término de Coriolis y fuerzas centrípetas sin derivar
% explícitamente los símbolos de Christoffel. Se usa la identidad
% de Christoffel-Lagrange (Spong, Hutchinson, Vidyasagar, 2006):
%
%   h(q,v) = (dM/dt)·v − ∇_q(½ vᵀ M v)
%
% con dM/dt = Σ_i ∂M/∂q_i · v_i  y cada ∂M/∂q_i por diferencias
% finitas centrales (paso eps).
%
% Esta implementación numérica evita errores algebraicos y coincide
% con la derivada analítica hasta O(eps^2).
% ------------------------------------------------------------------
function h = bias_vector(p, q, v, eps_)
  if nargin < 4 || isempty(eps_), eps_ = 1e-5; end
  dM = zeros(3,3,3);
  for i = 1:3
    dq = zeros(3,1); dq(i) = eps_;
    dM(:,:,i) = (mass_matrix(p, q + dq) - mass_matrix(p, q - dq)) / (2.0 * eps_);
  end
  Mdot = dM(:,:,1) * v(1) + dM(:,:,2) * v(2) + dM(:,:,3) * v(3);
  v_ = v(:);
  grad_T = zeros(3,1);
  for i = 1:3
    grad_T(i) = 0.5 * (v_' * dM(:,:,i) * v_);
  end
  h = Mdot * v_ - grad_T;
end
