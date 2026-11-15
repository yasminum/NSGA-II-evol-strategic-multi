%% gravity_vector.m  —  Vector de pares gravitacionales g(q) = ∂U/∂q.
% ------------------------------------------------------------------
% Sólo las articulaciones 2 y 3 (paralelas, eje horizontal) sienten la
% gravedad; la articulación de base es vertical y por tanto g1 = 0.
%
%  g2 = (m2*lc2 + m3*a2)*G*cos(q2) + m3*G*lc3*cos(q2+q3)
%  g3 = m3*G*lc3*cos(q2+q3)
% ------------------------------------------------------------------
function g = gravity_vector(p, q)
  c2  = cos(q(2));
  c23 = cos(q(2) + q(3));
  G = p.G;
  g2 = (p.m2 * p.lc2 + p.m3 * p.a2) * G * c2 + p.m3 * G * p.lc3 * c23;
  g3 = p.m3 * G * p.lc3 * c23;
  g = [0.0; g2; g3];
end
