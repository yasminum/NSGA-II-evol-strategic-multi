%% metrics_from_logs.m  —  Métricas de desempeño (protocolo del paper base).
% ------------------------------------------------------------------
% Métricas calculadas sobre las trazas de la simulación:
%
%   ISE     ∫‖e‖² dt            (métrica principal)
%   IAE     ∫‖e‖₁ dt
%   ISE_v   ∫‖v_d − v‖² dt
%   U2      ∫‖τ‖² dt            (esfuerzo de control)
%   u_max   max|τ|               (par pico)
%   e_max   max|e|               (error pico)
%   pct_sat % de ciclos con algún τ saturado
% ------------------------------------------------------------------
function m = metrics_from_logs(logs, dt)
  e   = logs.e;
  tau = logs.tau;
  ev  = logs.vd - logs.v;
  m.ISE     = dt * sum(e(:).^2);
  m.IAE     = dt * sum(abs(e(:)));
  m.ISE_v   = dt * sum(ev(:).^2);
  m.U2      = dt * sum(tau(:).^2);
  m.u_max   = max(abs(tau(:)));
  m.e_max   = max(abs(e(:)));
  m.pct_sat = 100.0 * mean(logs.sat);
end
