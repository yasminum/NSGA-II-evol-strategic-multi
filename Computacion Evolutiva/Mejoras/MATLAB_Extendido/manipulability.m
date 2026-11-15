%% manipulability.m  —  Medida de manipulabilidad de Yoshikawa.
% ------------------------------------------------------------------
%   w(q) = sqrt(det(J(q)·J(q)ᵀ))
%
% Cuantifica la cercanía a una singularidad cinemática. En la
% singularidad de base del brazo RRR (cuando el efector está sobre
% el eje vertical z) M11 colapsa y el lazo CTC se degrada; por eso
% los puntos de paso de la trayectoria se eligen con r3 ≥ 0.09 m.
% ------------------------------------------------------------------
function w = manipulability(p, q)
  J = jacobian_arm(p, q);
  JJt = J * J';
  w = sqrt(max(det(JJt), 0.0));
end
