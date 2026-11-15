%% opso_opt.m  —  Online Particle Swarm Optimization.
% ------------------------------------------------------------------
% Actualización estándar PSO (Kennedy & Eberhart, 1995) con
%   w  = 0.72   factor de inercia
%   c1 = 1.49   coeficiente cognitivo (atracción a la mejor personal)
%   c2 = 1.49   coeficiente social    (atracción a la mejor global)
%   vmax = 0.2 saturación simétrica de velocidad (fracción del rango)
%
% Cada partícula mantiene su mejor posición personal Pb; gbest es
% la mejor posición global. X y V se mantienen en [0,1]^d por clip.
%
% Sintaxis:
%   [x_best, f_best] = opso_opt(fbatch, dim, prev_best, pop, gens, w, c1, c2, vmax)
% ------------------------------------------------------------------
function [x_best, f_best] = opso_opt(fbatch, dim, prev_best, pop, gens, ...
                                     w, c1, c2, vmax)
  if nargin < 2 || isempty(dim),       dim = 9;       end
  if nargin < 3 || isempty(prev_best), prev_best = []; end
  if nargin < 4 || isempty(pop),       pop = 20;      end
  if nargin < 5 || isempty(gens),       gens = 20;     end
  if nargin < 6 || isempty(w),          w   = 0.72;   end
  if nargin < 7 || isempty(c1),         c1  = 1.49;    end
  if nargin < 8 || isempty(c2),         c2  = 1.49;    end
  if nargin < 9 || isempty(vmax),       vmax = 0.2;    end

  X = rand(pop, dim);
  if ~isempty(prev_best), X(1,:) = prev_best(:).'; end
  V = -0.1 + 0.2 * rand(pop, dim);
  F = fbatch(X);

  Pb   = X; Fpb = F;
  [~, gi] = min(Fpb);

  for g = 1:(gens-1)
    r1 = rand(pop, dim);
    r2 = rand(pop, dim);
    V  = w * V + c1 * r1 .* (Pb - X) + c2 * r2 .* (X(gi,:) - X);
    V  = max(-vmax, min(vmax, V));
    X  = max(0, min(1, X + V));
    F  = fbatch(X);
    imp = F < Fpb;
    Pb(imp,:) = X(imp,:);
    Fpb(imp)  = F(imp);
    [~, gi] = min(Fpb);
  end

  f_best = Fpb(gi);
  x_best = Pb(gi,:);
end
