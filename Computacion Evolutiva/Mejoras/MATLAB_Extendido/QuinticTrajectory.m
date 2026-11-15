%% @QuinticTrajectory  —  Trayectoria quíntica rest-to-rest por tramos.
% ------------------------------------------------------------------
% Polinomio quíntico entre dos puntos de paso: posición, velocidad y
% aceleración continuas, con v = a = 0 en cada punto de paso.
%
%   s = t − t_k            T = t_{k+1} − t_k            Δq = q_{k+1} − q_k
%   q(s) = q_k + (10·Δq/T^3)·s^3 − (15·Δq/T^4)·s^4 + (6·Δq/T^5)·s^5
%   v(s) = 30·Δq/T^3 ·s^2 − 60·Δq/T^4 ·s^3 + 30·Δq/T^5 ·s^4
%   a(s) = 60·Δq/T^3 ·s   − 180·Δq/T^4·s^2 + 120·Δq/T^5·s^3
%
% Constructor:
%   tr = QuinticTrajectory(waypoints, times)
%     waypoints : cell o matriz n×3 de puntos de paso [q1 q2 q3]
%     times     : vector 1×n de tiempos (times(1) = 0)
%
% Llamada (devuelve [q; v; a] como 3×3):
%   [q, v, a] = tr.eval(t)
% ------------------------------------------------------------------
classdef QuinticTrajectory
  properties
    wp   % n×3  puntos de paso
    tt   % 1×n  tiempos
  end
  methods
    function obj = QuinticTrajectory(waypoints, times)
      if iscell(waypoints)
        obj.wp = cell2mat(cellfun(@(x) x(:).', waypoints, ...
                                  'UniformOutput', false));
      else
        obj.wp = waypoints;
      end
      obj.tt = times(:).';
      assert(size(obj.wp,1) == numel(obj.tt), ...
              'waypoints y times deben tener la misma longitud');
      assert(obj.tt(1) == 0.0, 'times(1) debe ser 0');
    end
    function [q, v, a] = eval(obj, t)
      if t >= obj.tt(end)
        q = obj.wp(end,:).'; v = zeros(3,1); a = zeros(3,1);
        return;
      end
      k = find(t < obj.tt(2:end), 1, 'first');
      if isempty(k), k = 1; end
      T  = obj.tt(k+1) - obj.tt(k);
      s  = t - obj.tt(k);
      dq = (obj.wp(k+1,:) - obj.wp(k,:)).';
      c3 = 10 * dq / T^3;
      c4 = -15 * dq / T^4;
      c5 = 6  * dq / T^5;
      q = obj.wp(k,:).' + c3 * s^3 + c4 * s^4 + c5 * s^5;
      v = 3 * c3 * s^2 + 4 * c4 * s^3 + 5 * c5 * s^4;
      a = 6 * c3 * s + 12 * c4 * s^2 + 20 * c5 * s^3;
    end
  end
end
