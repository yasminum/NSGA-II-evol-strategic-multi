%% save_offline_gains.m  —  Genera results/offline_gains.mat pre-poblado.
% ------------------------------------------------------------------
% Ejecutar UNA VEZ (manualmente) para poblar el cache offline_gains.mat
% con los mismos valores numéricos que el código Python (almacenados en
% offline_gains.json), de modo que el experimento sea reproducible sin
% necesidad de re-ejecutar la sintonía offline (que toma ~30 s).
%
% Si prefieres recalcular, borra este .mat y ejecuta experiment.m:
% se invocará offline_tuning.m que usa ODE con la semilla 42, y los
% números DIFERIRÁN levemente por la diferencia de generadores entre
% NumPy y MATLAB (pero las ganancias son equivalentes en orden de
% magnitud: Kp ≈ 900, Ki ≈ 400, Kd ≈ 60).
% ------------------------------------------------------------------
function save_offline_gains()
  base = fileparts(mfilename('fullpath'));
  res_dir = fullfile(base, 'results');
  if ~exist(res_dir, 'dir'), mkdir(res_dir); end

  theta0 = [900.0, 900.0, 900.0, ...
            400.0, 341.36268333389006, 400.0, ...
             60.0,  60.0,  60.0];
  ISE_offline = 8.6996412943771e-08;

  save(fullfile(res_dir, 'offline_gains.mat'), 'theta0', 'ISE_offline');
  fprintf('Guardado results/offline_gains.mat con theta0 (offline Python):\n');
  fprintf('  Kp = [%.2f %.2f %.2f]\n', theta0(1:3));
  fprintf('  Ki = [%.2f %.2f %.2f]\n', theta0(4:6));
  fprintf('  Kd = [%.2f %.2f %.2f]\n', theta0(7:9));
  fprintf('  ISE_offline = %.3e\n', ISE_offline);
end
