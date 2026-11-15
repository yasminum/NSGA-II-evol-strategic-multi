%% forward_kinematics.m  —  Posiciones 3D de base, hombro, codo y efector.
% ------------------------------------------------------------------
% Cinemática directa del brazo RRR antropomórfico (Conejo Benítez
% 2021, ec. 3.17):
%
%   p_base  = [0, 0, 0]
%   p_homb = [0, 0, hB]
%   p_codo = [c1*l1*c2,  s1*l1*c2,  hB + l1*s2]
%   p_eef  = [c1*(l1*c2+l2*c23),  s1*(l1*c2+l2*c23),  hB + l1*s2 + l2*s23]
%
% Devuelve un cell {p_base, p_homb, p_codo, p_eef} con vectores 3x1.
% ------------------------------------------------------------------
function P = forward_kinematics(p, q)
  c1  = cos(q(1)); s1 = sin(q(1));
  c2  = cos(q(2)); s2 = sin(q(2));
  c23 = cos(q(2)+q(3)); s23 = sin(q(2)+q(3));
  p_base = [0; 0; 0];
  p_sh   = [0; 0; p.d1];
  r_el = p.a2 * c2;  z_el = p.d1 + p.a2 * s2;
  p_el = [c1 * r_el; s1 * r_el; z_el];
  r_ee = p.a2 * c2 + p.a3 * c23;
  z_ee = p.d1 + p.a2 * s2 + p.a3 * s23;
  p_ee = [c1 * r_ee; s1 * r_ee; z_ee];
  P = {p_base, p_sh, p_el, p_ee};
end
