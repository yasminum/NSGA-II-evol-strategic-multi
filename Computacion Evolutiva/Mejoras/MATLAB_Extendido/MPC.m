%% @MPC  —  Control Predictivo Basado en el Modelo (MPC) para el brazo RRR.
% ------------------------------------------------------------------
% Controlador predictivo articular con horizonte de predicción N_p y
% horizonte de control N_c, que minimiza en cada ciclo:
%
%   min_{U}  Σ_{i=1}^{N_p} ‖q_ref(t+i Δt) - q_pred_i‖²_Q + ρ Σ_{j=1}^{N_c} ‖Δu_j‖²_R
%
% Sujeto a:
%   |u| ≤ u_max,  |Δu| ≤ Δu_max
%
% El modelo de predicción es la linealización discreta de la
% dinámica articular del brazo alrededor del estado actual:
%
%   x_{i+1} = x_i + Δt · (A·x_i + B·u_i + d_i)
%
% donde x = [q; v] ∈ R^6, u ∈ R^3. La linealización usa la masa matriz
% M(q) en el estado actual. La solución se obtiene por proyección de
% gradiente (escalón en cada iteración, presupuesto pequeño); esto es
% suficiente para fines comparativos y respeta el espíritu del
% proyecto (sin toolbox de optimización).
%
% Conexión con el esquema "Hybrid Robust":
%   - El MPC se ocupa del seguimiento predictivo y del respeto de
%     cotas de par motor; el CTC-PID proporciona una ley estabilizadora
%     barata y el aporte NN corrige el residual no modelado.
%   - El MPC puede deshabilitarse temporalmente si su presupuesto
%     excede el ciclo de muestreo Δt; en ese caso el CTC-PID toma el
%     relevo (mecanismo de conmutación en HybridRobust.m).
% ------------------------------------------------------------------
classdef MPC < handle
  properties
    p_nom       % parámetros nominales del brazo (ArmParams)
    traj        % trayectoria de referencia (QuinticTrajectory)
    dt          % período de muestreo
    N_p         % horizonte de predicción
    N_c         % horizonte de control
    rho         % peso de control (esfuerzo)
    Q_w         % matriz de pesos de estado (3×3) (default: I)
    u_max       % vector 1×3 (par máximo, igual a p_nom.tau_max)
    du_max      % vector 1×3 (incremento máximo por muestra)
    last_u      % acción del ciclo anterior (1×3) (para Δu)
    max_iter    % iteraciones del optimizador (10)
    name        % etiqueta de controlador
  end
  methods
    function obj = MPC(p_nom, traj, dt, N_p, N_c, rho, du_max, max_iter)
      if nargin < 4 || isempty(N_p),  N_p = 5;  end
      if nargin < 5 || isempty(N_c),  N_c = 3;  end
      if nargin < 6 || isempty(rho),  rho = 0.1; end
      if nargin < 7 || isempty(du_max), du_max = 0.4 * p_nom.tau_max; end
      if nargin < 8 || isempty(max_iter), max_iter = 10; end
      obj.p_nom    = p_nom;
      obj.traj     = traj;
      obj.dt       = dt;
      obj.N_p      = N_p;
      obj.N_c      = N_c;
      obj.rho      = rho;
      obj.Q_w      = eye(3);
      obj.u_max    = p_nom.tau_max(:).';
      obj.du_max   = du_max(:).';
      obj.max_iter = max_iter;
      obj.last_u   = zeros(1, 3);
      obj.name     = 'MPC';
    end

    function reset(obj)
      obj.last_u = zeros(1, 3);
    end

    function tau = step(obj, t, qm, vm)
    % Calcula la acción de control en el ciclo actual.
      qm = qm(:); vm = vm(:);
      % 1. Linealizar el modelo alrededor de (qm, vm): masa matriz M y
      %    términos gravitatorios en qm proporcionan la aceleración "libre"
      %    y la sensibilidad a u.
      M  = mass_matrix(obj.p_nom, qm);
      h  = bias_vector(obj.p_nom, qm, vm) ...
           + gravity_vector(obj.p_nom, qm) ...
           + friction_vector(obj.p_nom, vm);
      % Modelo linealizado continuo:
      %   x_dot = A x + B u + d
      % con A = 0 (linealización del brazo alrededor de un punto fijo),
      %      B = [zeros(3,3); M^-1]
      %      d = [v; -M^-1 (h)]   (vector de "estado libre")
      B = [zeros(3, 3); M \ eye(3)];        % (6×3)
      d_free = [vm; -(M \ h)];              % (6×1)
      % Discretización explícita Euler: x_{k+1} = x_k + dt (B u + d_free)
      B_d = obj.dt * B;                    % (6×3)
      d_free_d = obj.dt * d_free;           % (6×1)

      % 2. Trayectoria de referencia futura
      q_ref_future = zeros(obj.N_p, 3);
      for i = 1:obj.N_p
        [qr, ~, ~] = obj.traj.eval(t + i * obj.dt);
        q_ref_future(i, :) = qr(:).';
      end

      % 3. Optimización: proyección de gradiente simple.
      % Variables: U (N_p × 3) — secuencia de acciones.
      U = repmat(obj.last_u, obj.N_p, 1);
      step_size = 0.3;
      for iter = 1:obj.max_iter
        % Forward simulate
        x = [qm; vm];    % (6×1)
        Xq_pred = zeros(obj.N_p, 3);
        for i = 1:obj.N_p
          x = x + B_d * U(i, :).' + d_free_d;   % (6×1)
          Xq_pred(i, :) = x(1:3).';            % (1×3)
        end
        % Cost y gradiente (lineal en U)
        J = 0; g = zeros(obj.N_p, 3);
        for i = 1:obj.N_p
          e_i = q_ref_future(i, :).' - Xq_pred(i, :).';   % (3×1)
          J = J + e_i' * obj.Q_w * e_i;
          % gradiente de J1 con respecto a U(i, :):
          %   ∂J1 / ∂U(i) = -2 * (B_d' * (Q_w * e_i))
          % pero cada U(i) sólo afecta a los pred X_q(k) para k ≥ i
          for k = i:obj.N_p
            ek = q_ref_future(k, :).' - Xq_pred(k, :).'; %#ok<NASGU>
            g(i, :) = g(i, :) - 2 * (obj.Q_w * (q_ref_future(k, :).' - Xq_pred(k, :).')).' * B_d(1:3, :);
          end
        end
        % Penalización por Δu (primer orden)
        for i = 1:obj.N_c-1
          du = U(i+1, :) - U(i, :);
          J = J + obj.rho * (du * du.');
          g(i, :)   = g(i, :)   + 2*obj.rho * (U(i, :) - U(i+1, :));
          g(i+1, :) = g(i+1, :) + 2*obj.rho * (U(i+1, :) - U(i, :));
        end
        % Paso de gradiente y proyección a la caja [|last_u ± du_max| y |u| ≤ u_max]
        g_max = max(max(abs(g(:))), 1e-6);
        step_eff = step_size / g_max;
        U_new = U - step_eff * g;
        % Asegurar que U(i) = U(N_c) para todo i > N_c (horizonte de control)
        for i = (obj.N_c+1):obj.N_p
          U_new(i, :) = U_new(min(i, obj.N_c), :);
        end
        % Restricciones de Δu y saturación
        for i = 1:obj.N_p
          ub = obj.last_u + i * obj.du_max;
          lb = obj.last_u - i * obj.du_max;
          U_new(i, :) = max(lb, min(ub, U_new(i, :)));
          U_new(i, :) = max(-obj.u_max, min(obj.u_max, U_new(i, :)));
        end
        U = U_new;
      end
      obj.last_u = U(1, :);
      tau = U(1, :).';
    end
  end
end
