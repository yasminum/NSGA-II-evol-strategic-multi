%% make_disturbance.m  —  Perturbación externa de par.
% ------------------------------------------------------------------
% Devuelve un function handle d(t) que entrega el vector 3×1 de par
% de perturbación: senoidal continua + escalón en t_step.
%
% Amplitudes escaladas a ~15-20 % de los límites de par, inspiradas
% en las perturbaciones sinusoidales de Conejo Benítez (2021, cap.4).
%
%   step_amp = [0.30, 0.20, 0.15]    senoidal_amp = [0.25, 0.18, 0.12]
%   freqs    = [1.2, 0.9, 1.5]       t_step = 4.2 s
%
% Uso:
%   d = make_disturbance();          d = @(t)make_disturbance()(t);
%   d_val = d(3.0);                  % 3×1 vector
% ------------------------------------------------------------------
function d = make_disturbance(step_amp, t_step, sine_amp, freqs)
  if nargin < 1 || isempty(step_amp), step_amp = [0.30, 0.20, 0.15]; end
  if nargin < 2 || isempty(t_step),   t_step   = 4.2;                 end
  if nargin < 3 || isempty(sine_amp), sine_amp = [0.25, 0.18, 0.12];  end
  if nargin < 4 || isempty(freqs),    freqs    = [1.2, 0.9, 1.5];    end
  step_amp = step_amp(:);
  sine_amp = sine_amp(:);
  freqs    = freqs(:);
  d = @(t) disturbance_fn(t, step_amp, t_step, sine_amp, freqs);
end

function out = disturbance_fn(t, step_amp, t_step, sine_amp, freqs)
  out = sine_amp .* sin(freqs * t);
  if t >= t_step
    out = out + step_amp;
  end
end
