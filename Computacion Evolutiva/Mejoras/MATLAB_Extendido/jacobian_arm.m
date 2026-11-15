%% jacobian_arm.m  —  Jacobiano geométrico lineal (3×3): v_ee = J(q)·q̇.
% ------------------------------------------------------------------
% Para un brazo RRR con eje 1 vertical y ejes 2-3 horizontales
% paralelos, el Jacobiano de la posición del efector es
%
%   J(:,1) = z0 × (p_ee − p_base),   z0 = [0,0,1]
%   J(:,2) = z1 × (p_ee − p_sh),    z1 = [−s1, c1, 0]
%   J(:,3) = z1 × (p_ee − p_el)
%
% (los ejes 2 y 3 comparten z1 por ser paralelos). Se nombró con
% sufijo _arm para evitar conflicto con jacobian() de MATLAB.
% ------------------------------------------------------------------
function J = jacobian_arm(p, q)
  P = forward_kinematics(p, q);
  p_base = P{1}; p_sh = P{2}; p_el = P{3}; p_ee = P{4};
  z0 = [0; 0; 1];
  z1 = [-sin(q(1)); cos(q(1)); 0];
  J = zeros(3,3);
  J(:,1) = cross(z0, p_ee - p_base);
  J(:,2) = cross(z1, p_ee - p_sh);
  J(:,3) = cross(z1, p_ee - p_el);
end
