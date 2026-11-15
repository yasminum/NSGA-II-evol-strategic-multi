%% friction_vector.m  —  F(v) = Fv·v + Fc·tanh(v/ε).
% ------------------------------------------------------------------
% Fricción viscosa lineal más Coulomb suavizado con tanh(v/ε), ε=0.05
% (evita la discontinuidad en v=0 que rompería el integrador RK4).
% El controlador sólo compensa Fv·v (término nominal); el Coulomb es
% "no modelado" y por tanto una perturbación.
% ------------------------------------------------------------------
function F = friction_vector(p, v)
  F = p.Fv(:) .* v(:) + p.Fc(:) .* tanh(v(:) / 0.05);
end
