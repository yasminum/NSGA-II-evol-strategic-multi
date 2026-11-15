%% online_bounds.m  —  Cotas de sintonización online.
% ------------------------------------------------------------------
% Define el hipercubo de búsqueda por ganancia para el sintonizador
% online, según el criterio del paper:
%   * Kp:   amplio   [0.25·Kp0,  4·Kp0]
%   * Ki:   medio    [min(10, Ki0), 400]
%   * Kd:   estrecho [0.75·Kd0, 1.25·Kd0]
%     (Kd es muy sensible y un rango amplio produce oscilación)
% ------------------------------------------------------------------
function [lo, hi] = online_bounds(theta0)
  kp0 = theta0(1:3);
  ki0 = theta0(4:6);
  kd0 = theta0(7:9);
  lo = [0.25 * kp0,  min(10.0, ki0), 0.75 * kd0];
  hi = [4.00 * kp0,  400*ones(1,3),  1.25 * kd0];
end
