%% default_trajectory.m  —  Trayectoria articular de referencia.
% ------------------------------------------------------------------
% Dos traslados punto a punto + mantenimiento, duración total T = 5 s
% (1 s de reposo final). Los puntos de paso se eligen de modo que el
% radio horizontal del efector r3 = l1·cos(θ2) + l2·cos(θ2+θ3) se
% mantenga ≥ 0.09 m en todo el recorrido, evitando la singularidad
% cinemática de la articulación de base (en la que M11(q) colapsa y
% el lazo se degrada; véase manipulability()).
%
%   t=0        :  q0 = [0.00, 0.35, 0.55]  (configuración extendida)
%   t=2.5 s    :  qA = [1.30, 0.70, 0.80]  (traslado 1)
%   t=5.0 s    :  qB = [0.45, 0.50, 0.45]  (traslado 2)
% ------------------------------------------------------------------
function tr = default_trajectory()
  wp = [0.00, 0.35, 0.55; ...
        1.30, 0.70, 0.80; ...
        0.45, 0.50, 0.45];
  tt = [0.0, 2.5, 5.0];
  tr = QuinticTrajectory(wp, tt);
end
