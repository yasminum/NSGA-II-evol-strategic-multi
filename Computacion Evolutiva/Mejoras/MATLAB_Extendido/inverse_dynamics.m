%% inverse_dynamics.m  —  τ = M(q)·a + c(q,v) + g(q) + F(v).
% ------------------------------------------------------------------
% Dinámica inversa: dado el estado (q,v) y la aceleración articular
% deseada a, devuelve el par requerido. La usa el CTC con parámetros
% NOMINALES (sin Coulomb, sin incertidumbre) para compensar la no
% linealidad conocida y dejar el lazo PID lineal.
% ------------------------------------------------------------------
function tau = inverse_dynamics(p, q, v, a)
  tau = mass_matrix(p, q) * a(:) + bias_vector(p, q, v) ...
        + gravity_vector(p, q) + friction_vector(p, v);
end
