%% oga_opt.m  —  Online Genetic Algorithm (torneo + cruce aritmético).
% ------------------------------------------------------------------
% Operadores:
%   * selección  por torneo k = 3
%   * cruce      aritmético completo  child = a·P1 + (1-a)·P2, a ~ U(0,1)
%     con probabilidad pc = 0.9 (si no cruza, se copia P1)
%   * mutación   gaussiana N(0, σ²), σ = 0.08, con prob. pm = 0.15
%     por gen; clip al [0,1]
%   * elitismo   el mejor padre reemplaza al peor hijo
%   * warm-start: el mejor individuo previo se inyecta en X[1]
%
% Sintaxis:
%   [x_best, f_best] = oga_opt(fbatch, dim, prev_best, pop, gens, pc, pm, sigma_mut, tour_k)
% ------------------------------------------------------------------
function [x_best, f_best] = oga_opt(fbatch, dim, prev_best, pop, gens, ...
                                    pc, pm, sigma_mut, tour_k)
  if nargin < 2 || isempty(dim),       dim = 9;        end
  if nargin < 3 || isempty(prev_best), prev_best = []; end
  if nargin < 4 || isempty(pop),       pop = 20;       end
  if nargin < 5 || isempty(gens),       gens = 20;      end
  if nargin < 6 || isempty(pc),         pc  = 0.9;      end
  if nargin < 7 || isempty(pm),         pm  = 0.15;     end
  if nargin < 8 || isempty(sigma_mut),   sigma_mut = 0.08; end
  if nargin < 9 || isempty(tour_k),      tour_k = 3;    end

  X = rand(pop, dim);
  if ~isempty(prev_best), X(1,:) = prev_best(:).'; end
  F = fbatch(X);

  for g = 1:(gens-1)
    P1 = tournament(X, F, pop, tour_k);
    P2 = tournament(X, F, pop, tour_k);
    do_c = (rand(pop,1) < pc);                 % (pop,1)
    a   = rand(pop, 1);                          % (pop,1)
    children = P1;
    do_c_full = repmat(do_c, 1, dim);
    children(do_c_full) = (repmat(a,1,dim) .* P1 + ...
                            (1 - repmat(a,1,dim)) .* P2)(do_c_full);
    mut   = rand(pop, dim) < pm;
    noise = sigma_mut * randn(pop, dim);
    children(mut) = children(mut) + noise(mut);
    children = max(0, min(1, children));

    Fc = fbatch(children);
    % elitismo: el mejor padre reemplaza al peor hijo
    [~, elite]  = min(F);
    [v_worst, worst] = max(Fc);
    children(worst,:) = X(elite,:);
    Fc(worst)         = F(elite);
    X = children; F = Fc;
  end

  [f_best, b] = min(F);
  x_best = X(b,:);
end

function W = tournament(X, F, pop, k)
  cand = randi(pop, pop, k);                  % (pop,k)
  Fc = F(cand);                              % (pop,k)
  [~, w] = min(Fc, [], 2);                   % (pop,1) índice local
  idx = sub2ind([pop, k], (1:pop).', w);
  W = X(cand(idx), :);
end
