%% arm_params.m  —  Parámetros del brazo robótico RRR de 3 GDL.
% ------------------------------------------------------------------
% Parámetros físicos tomados de Conejo Benítez (2021), "Modelado y
% control de un brazo robótico de 3 grados de libertad", Tesis de
% Maestría, Universidad de Guanajuato (Tabla 3.7 y Sección 3.2).
%
% Geometría antropomórfica: articulación 1 (base, eje vertical z,
% Dynamixel AX-18A) y articulaciones 2-3 paralelas (ejes horizontales,
% AX-12A). Masas puntuales concentradas en los extremos de los
% eslabones (láminas 3D, inercias despreciables).
%
% Convención de ángulos (idéntica a la tesis, ec. 3.17):
%   q = [theta1, theta2, theta3], theta2 medido desde el plano
%   horizontal, theta3 relativo al eslabón 2.
%
% Devuelve un struct con campos escalares y vectores 1x3.
% ------------------------------------------------------------------
function p = arm_params()
  p.d1 = 0.079;            % hB: altura de la base [m]
  p.a2 = 0.113;            % l1: longitud del eslabón 1 [m]
  p.a3 = 0.140;            % l2: longitud del eslabón 2 [m]
  p.m1 = 0.147;            % masa base/eslabón 1 [kg]
  p.m2 = 0.116;            % masa eslabón 2 (codo) [kg]
  p.m3 = 0.080;            % masa efector + herramienta [kg]
  p.I1 = 2.0e-4;           % inercia equivalente rotor+base z [kg*m^2]
  p.I2 = 0.0;
  p.I3 = 0.0;
  p.lc2 = 0.113;           % dist. c.d.m. eslabón 2 (= a2)
  p.lc3 = 0.140;           % dist. c.d.m. eslabón 3 (= a3)
  p.Fv = [0.1772, 0.2428, 0.2428];   % fricción viscosa [N*m*s/rad]
  p.Fc = [0.0, 0.0, 0.0];             % fricción Coulomb [N*m]
  p.tau_max = [1.8, 1.5, 1.5];        % par máx AX-18A/AX-12A [N*m]
  p.G  = 9.81;                        % m/s^2
end
