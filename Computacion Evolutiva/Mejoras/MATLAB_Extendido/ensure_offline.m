%% ensure_offline.m  —  Cachea/carga las ganancias offline θ0.
% ------------------------------------------------------------------
% Si results/offline_gains.mat existe, devuelve el contenido cacheado.
% Si no, ejecuta la sintonía offline con ODE (escenario ideal) y
% guarda el resultado en results/offline_gains.mat.
% ------------------------------------------------------------------
function [theta0, ISE_off] = ensure_offline(force, base_path)
  if nargin < 1, force = false; end
  if nargin < 2 || isempty(base_path)
    base_path = fileparts(mfilename('fullpath'));
    if isempty(base_path), base_path = '.'; end
  end
  off_path = fullfile(base_path, 'results', 'offline_gains.mat');
  if exist(off_path, 'file') && ~force
    S = load(off_path);
    theta0 = S.theta0;
    ISE_off = S.ISE_offline;
    return;
  end
  traj = default_trajectory();
  fprintf('Sintonía offline del CTC fijo (ODE, escenario ideal)...\n');
  t0 = tic;
  [theta0, ISE_off] = offline_tuning(traj);
  fprintf('  theta0 = [%.2f %.2f %.2f | %.2f %.2f %.2f | %.2f %.2f %.2f]  (ISE = %.3e, %.0f s)\n', ...
          theta0(1), theta0(2), theta0(3), ...
          theta0(4), theta0(5), theta0(6), ...
          theta0(7), theta0(8), theta0(9), ISE_off, toc(t0));
  if ~exist(fullfile(base_path, 'results'), 'dir')
    mkdir(fullfile(base_path, 'results'));
  end
  save(off_path, 'theta0', 'ISE_off');
end
