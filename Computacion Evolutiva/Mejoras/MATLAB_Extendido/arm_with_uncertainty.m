%% arm_with_uncertainty.m  —  Planta con incertidumbre paramétrica.
% ------------------------------------------------------------------
% Devuelve una copia del struct de parámetros modificada según
% factores de incertidumbre (masas, carga útil, fricción, Coulomb no
% modelado).
%
% Argumentos (todos opcionales):
%   m2_factor       factor sobre m2 (1.10 => +10 %)
%   m3_factor       factor sobre m3
%   payload         masa adicional en el efector [kg]
%   viscous_factor  factor sobre Fv (1.5 => fricción viscosa +50 %)
%   coulomb         vector 1x3 de fricción de Coulomb [N*m]
% ------------------------------------------------------------------
function p = arm_with_uncertainty(p0, m2_factor, m3_factor, payload, ...
                                  viscous_factor, coulomb)
  if nargin < 2 || isempty(m2_factor),       m2_factor = 1.0;       end
  if nargin < 3 || isempty(m3_factor),       m3_factor = 1.0;       end
  if nargin < 4 || isempty(payload),         payload = 0.0;        end
  if nargin < 5 || isempty(viscous_factor),  viscous_factor = 1.0;  end
  if nargin < 6 || isempty(coulomb),          coulomb = [0 0 0];    end
  p = p0;
  p.m2 = p0.m2 * m2_factor;
  p.m3 = p0.m3 * m3_factor + payload;
  p.Fv = p0.Fv .* viscous_factor;
  p.Fc = coulomb(:).';
end
