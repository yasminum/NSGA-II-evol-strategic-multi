%% run_case.m  —  Ejecuta una corrida individual (método × escenario × semilla).
% ------------------------------------------------------------------
% Construye planta + controlador y dispara la simulación. Devuelve un
% struct con las métricas. Es la unidad de trabajo que el script
% principal invoca (en serie; MATLAB/Octave no usan pool por defecto).
%
% Argumentos:
%   task  struct con campos .method, .scen, .seed, .theta0, .T
%
% Devuelve:
%   m     struct con ISE, IAE, ISE_v, U2, u_max, e_max, pct_sat,
%         T_adapt, ms_retune, scenario, method, seed
%
% Modo traza: si seed == TRACE_SEED (11) se guardan en
%   results/traces/{scen}_{method}.mat
% los campos t, q, qd, e, v, vd, tau, gains (matriz n_retunes×10).
% ------------------------------------------------------------------
function m = run_case(task, base_path)
  if nargin < 2, base_path = '.'; end
  method  = task.method;
  scen    = task.scen;
  seed    = task.seed;
  theta0  = task.theta0;
  T       = task.T;

  DT = 0.005;
  S_B = 10;
  TRACE_SEED = 11;

  [lo, hi] = online_bounds(theta0);

  SCENARIOS = scenarios_table();
  sc = SCENARIOS.(scen);

  p_nom   = arm_params();
  p_plant = arm_with_uncertainty(p_nom, sc.m2f, sc.m3f, sc.payload, ...
                                  sc.vf, sc.coulomb);
  traj = default_trajectory();
  if sc.dist
    dist = make_disturbance();
  else
    dist = [];
  end

  if strcmp(method, 'CTC')
    ctrl = CTC_PID(p_nom, traj, DT, theta0(1:3), theta0(4:6), theta0(7:9));
  else
    tok = strsplit(method, '-');
    reg = tok{1}; opt = tok{2};
    deg = 1; if strcmp(reg, 'PR'), deg = 2; end
    ctrl = AdaptiveCTC(p_nom, traj, DT, theta0(1:3), theta0(4:6), theta0(7:9), ...
                       lo, hi, opt, deg, S_B, 1, 20, 20, seed);
  end

  % semilla de ruido común a todas las alternativas (CRN)
  sim_seed = 10000 + seed;
  if isempty(sc.noise)
    noise = struct('q', 0, 'v', 0);
  else
    noise = sc.noise;
  end
  logs = simulate(p_plant, ctrl, traj, T, DT, sim_seed, noise, dist, 5);
  m = metrics_from_logs(logs, DT);
  m.T_adapt   = ctrl.t_adapt_total;
  if isprop(ctrl, 'n_retunes') && ctrl.n_retunes > 0
    m.ms_retune = 1000.0 * ctrl.t_adapt_total / ctrl.n_retunes;
  else
    m.ms_retune = 0.0;
  end
  m.scenario  = scen;
  m.method   = method;
  m.seed     = seed;

  if seed == TRACE_SEED
    traces_dir = fullfile(base_path, 'results', 'traces');
    if ~exist(traces_dir, 'dir'), mkdir(traces_dir); end
    fn = fullfile(traces_dir, sprintf('%s_%s.mat', scen, method));
    if isprop(ctrl, 'gain_history')
      gh = ctrl.gain_history;
    else
      gh = zeros(0, 10);
    end
    save(fn, 'logs', 'gh');
  end
end
