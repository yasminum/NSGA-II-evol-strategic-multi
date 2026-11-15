%% @HybridRobust  —  Control Robusto Híbrido: CTC-PID + MPC + compensador NN.
% ------------------------------------------------------------------
% Controlador híbrido que fusiona tres leyes de control complementarias
% en un único lazo de control:
%
%   τ(t) = α(t) · τ_CTC(t) + β(t) · τ_MPC(t) + γ · τ_NN(t)
%
% donde:
%   τ_CTC(t)  = ley de torque calculado con PID (estabilizador barato,
%               ideal cuando el modelo es suficientemente exacto).
%   τ_MPC(t)  = ley predictiva con cotas de par y de Δu (mejor cuando
%               se avecinan restricciones o cambios bruscos de setpoint).
%   τ_NN(t)   = compensador neuronal del residual (ajusta errores
%               sistemáticos del modelo nominal debidos a fricción
%               no modelada, perturbaciones, etc.).
%
% Los pesos α, β se obtienen por mezcla suave (softmax) en función de:
%   - Presupuesto de cómputo (si el MPC excede el ciclo Δt, su peso
%     baja y el CTC toma el relevo — esquema de conmutación suave).
%   - Magnitud del error de seguimiento: si |e| es grande, el MPC
%     se refuerza (mira hacia adelante para evitar overshoot).
%   - Variabilidad de la trayectoria (alta derivada de q_ref favorece
%     al MPC; baja derivada favorece al CTC).
%
% El compensador NN se mantiene en estado de "compensador residual":
%   - Se entrena online sobre la ventana de buffer para predecir el
%     error del siguiente paso e_pred = q_next_pred - q_next_medido.
%   - La corrección τ_NN = -K_NN · M_nom · e_pred convierte el
%     error previsto en una corrección equivalente de par.
%
% Esta arquitectura corresponde al título "Control Robusto Híbrido
% basado en Torque Computado, Control Predictivo y Redes Neuronales"
% del proyecto extendido.
%
% API compatible con simulate.m: objeto con métodos .reset() y
% .step(t, qm, vm) -> tau.
% ------------------------------------------------------------------
classdef HybridRobust < handle
  properties
    ctc          % instancia de CTC_PID
    mpc          % instancia de MPC
    nn_comp      % NNRegressor (compensador de residual)
    K_NN         % ganancia escalar de compensación NN (0.1 por defecto)
    alpha_ctc    % peso actual del CTC
    beta_mpc     % peso actual del MPC
    gamma_nn     % peso del término NN (fijo)
    t_max_mpc    % presupuesto [s] del MPC (si se excede, su peso baja)
    last_tau     % acción anterior (para diagnóstico)
    buf_q        % estados recientes (para entrenar el NN)
    buf_v
    buf_u
    buf_qd_next  % siguiente estado real medido (objetivo del NN)
    name         % 'Hybrid'
    verbose
  end
  methods
    function obj = HybridRobust(p_nom, traj, dt, Kp, Ki, Kd, ...
                                  N_p, N_c, rho, n_hid, K_NN)
      if nargin < 7 || isempty(N_p),   N_p = 5;   end
      if nargin < 8 || isempty(N_c),   N_c = 3;   end
      if nargin < 9 || isempty(rho),   rho = 0.1; end
      if nargin < 10 || isempty(n_hid), n_hid = 16; end
      if nargin < 11 || isempty(K_NN),  K_NN = 0.1; end
      obj.ctc  = CTC_PID(p_nom, traj, dt, Kp, Ki, Kd);
      obj.mpc  = MPC(p_nom, traj, dt, N_p, N_c, rho);
      obj.nn_comp = NNRegressor(9, 6, n_hid, 0.01, 100, 0.9);
      obj.K_NN = K_NN;
      obj.alpha_ctc = 0.5;
      obj.beta_mpc  = 0.5;
      obj.gamma_nn  = 0.2;
      obj.t_max_mpc = 0.5 * dt;   % umbral: si MPC > 0.5·Δt, su peso baja
      obj.last_tau = zeros(1, 3);
      obj.buf_q = {};
      obj.buf_v = {};
      obj.buf_u = {};
      obj.buf_qd_next = {};
      obj.name = 'Hybrid';
      obj.verbose = false;
    end

    function reset(obj)
      obj.ctc.reset();
      obj.mpc.reset();
      obj.last_tau = zeros(1, 3);
      obj.buf_q = {};
      obj.buf_v = {};
      obj.buf_u = {};
      obj.buf_qd_next = {};
    end

    function tau = step(obj, t, qm, vm)
      % 1. Calcular CTC (barato, siempre)
      tau_ctc = obj.ctc.step(t, qm, vm);

      % 2. Calcular MPC (caro) con cronómetro
      t_start = tic;
      tau_mpc = obj.mpc.step(t, qm, vm);
      t_mpc = toc(t_start);

      % 3. Calcular corrección NN (sólo si está entrenado)
      tau_nn = zeros(3, 1);
      if obj.nn_comp.is_trained
        x = [qm(:).', vm(:).', obj.last_tau];
        yhat = obj.nn_comp.predict_one(x);   % (1 × 6): predicción de [q_next, v_next]
        q_next_pred = yhat(1:3).';
        [qr, ~, ~] = obj.traj.eval(t + obj.dt);
        e_pred = qr(:) - q_next_pred;
        M_nom = mass_matrix(obj.ctc.p, qm);
        tau_nn = obj.K_NN * (M_nom * e_pred);
      end

      % 4. Pesos: mezcla suave por costo de cómputo + error
      e_now = obj.ctc.qd_(:) - qm(:);
      e_mag = norm(e_now);
      % Si MPC fue caro, bajar su peso
      if t_mpc > obj.t_max_mpc
        w_mpc = 0.2;
      else
        % Errores grandes → más MPC; errores pequeños → más CTC
        w_mpc = 1 / (1 + exp(-(e_mag - 0.05) * 50));
      end
      obj.beta_mpc  = w_mpc;
      obj.alpha_ctc = 1 - w_mpc;

      % 5. Combinación convexa + corrección NN (no normalizada)
      tau = obj.alpha_ctc * tau_ctc(:) + obj.beta_mpc * tau_mpc(:) ...
            + obj.gamma_nn * tau_nn;

      % 6. Actualizar buffers para entrenar el NN residual.
      % Almacenamos (qm_t, vm_t, tau_t) — la acción NUEVA que se
      % aplicará en este ciclo, NO la anterior, para que la NN aprenda
      % la correlación (estado, acción actual) -> estado siguiente.
      obj.buf_q{end+1} = qm(:).';
      obj.buf_v{end+1} = vm(:).';
      obj.buf_u{end+1} = tau(:).';

      % Emparejar las parejas (x_t, y_t+1) — se construye sobre la marcha:
      % buf_q[k+1] es el "siguiente estado" después de aplicar buf_u[k] a
      % (buf_q[k], buf_v[k]).
      if numel(obj.buf_q) >= 2 && numel(obj.buf_q) >= 20
        s = 20;
        Q = cell2mat(obj.buf_q(end-s+1:end).');
        V = cell2mat(obj.buf_v(end-s+1:end).');
        U = cell2mat(obj.buf_u(end-s+1:end).');
        X = [Q(1:s-1, :), V(1:s-1, :), U(1:s-1, :)];
        Yq = Q(2:end, :);
        Yv = V(2:end, :);
        Y = [Yq, Yv];
        obj.nn_comp.fit(X, Y);
      end

      obj.last_tau = tau(:).';
    end
  end
end
