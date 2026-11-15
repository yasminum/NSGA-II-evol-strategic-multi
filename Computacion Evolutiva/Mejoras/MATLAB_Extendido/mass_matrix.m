%% mass_matrix.m  —  M(q) simétrica definida positiva (brazo RRR).
% ------------------------------------------------------------------
% Estructura en bloque del brazo RRR: M11 sólo acopla la articulación
% 1 (base); el bloque 2-3 tiene términos de acoplamiento por la
% masa m3 repartida entre codo y efector.
%
% M(q) =
%  [ m11      0       0   ]
%  [  0       m22    m23 ]
%  [  0       m23    m33 ]
%
% con
%  m11 = I1 + (m2*lc2^2 + I2)*c2^2 + m3*r3^2 + I3*c23^2
%  m22 = m2*lc2^2 + I2 + m3*(a2^2 + lc3^2 + 2*a2*lc3*c3) + I3
%  m23 = m3*(lc3^2 + a2*lc3*c3) + I3
%  m33 = m3*lc3^2 + I3
%
% r3 = a2*c2 + lc3*c23  (radio horizontal del efector).
% ------------------------------------------------------------------
function M = mass_matrix(p, q)
  q2 = q(2); q3 = q(3);
  c2  = cos(q2);
  c3  = cos(q3);
  c23 = cos(q2 + q3);
  r3  = p.a2 * c2 + p.lc3 * c23;
  m11 = p.I1 + (p.m2 * p.lc2^2 + p.I2) * c2^2 + p.m3 * r3^2 + p.I3 * c23^2;
  m22 = (p.m2 * p.lc2^2 + p.I2 ...
        + p.m3 * (p.a2^2 + p.lc3^2 + 2.0 * p.a2 * p.lc3 * c3) + p.I3);
  m23 = p.m3 * (p.lc3^2 + p.a2 * p.lc3 * c3) + p.I3;
  m33 = p.m3 * p.lc3^2 + p.I3;
  M = [m11, 0.0, 0.0; ...
       0.0, m22, m23; ...
       0.0, m23, m33];
end
