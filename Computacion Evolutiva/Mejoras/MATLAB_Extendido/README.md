# Proyecto CE Brazo 3 GDL — Replicación en MATLAB (versión extendida)

Código MATLAB/Octave del proyecto de **Computación Evolutiva · 2026-2**
(Maestría en Inteligencia Artificial, Universidad Nacional de Ingeniería).

## Título del proyecto extendido

> **Modelado, Simulación y Control Robusto Híbrido basado en Torque
> Computado, Control Predictivo y Redes Neuronales, con Sintonización
> Evolutiva Multi-Objetivo Asistida por Modelos Sustitutos, para el
> Posicionamiento de un Brazo Robótico de Tres Grados de Libertad.**

Esta versión extiende el prototipo Python replicado en MATLAB (que
implementaba el marco **PR-ODE** de Rodríguez-Molina et al. 2024 sobre
el manipulador **RRR de 3 GDL** de la tesis de Conejo Benítez, 2021)
con cinco nuevas capacidades:

1. **Redes Neuronales (NN)** como regresor alternativo al polinomial
   (`NNRegressor`, `NNAdaptiveCTC`).
2. **Control Predictivo (MPC)** con horizonte N_p/N_c y cotas de par
   y de Δu (`MPC`).
3. **Control Robusto Híbrido** que fusiona CTC + MPC + compensador NN
   por mezcla suave (`HybridRobust`).
4. **Sintonización Evolutiva Multi-Objetivo (NSGA-II)** con tres
   objetivos (ISE_pred, ||u||², saturación) y selección de knee point
   (`nsga2`, `MultiObjAdaptiveCTC`).
5. **Modelos Sustitutos** (RBF, Kriging, NN) con muestreo adaptativo
   LIE para acelerar la sintonía evolutiva ~10× (`RBFNet`,
   `KrigingSurrogate`, `NNSurrogate`, `SurrogateManager`,
   `SurrogateAdaptiveCTC`).

## Estructura

### Núcleo dinámico (sin cambios respecto a la versión base)
```
arm_params.m              Parámetros físicos del brazo (Conejo, 2021).
arm_with_uncertainty.m    Planta con incertidumbre paramétrica.
mass_matrix.m             M(q) simétrica definida positiva.
gravity_vector.m          g(q) = ∂U/∂q.
friction_vector.m         F(v) = Fv·v + Fc·tanh(v/ε).
bias_vector.m             h(q,v) = C(q,v)·v vía identidad de Lagrange.
inverse_dynamics.m        τ = M(q)a + c + g + F (CTC nominal).
forward_dynamics.m        a = M⁻¹(τ + d − h − g − F).
forward_kinematics.m      Posiciones 3D de base/hombro/codo/efector.
jacobian_arm.m            Jacobiano geométrico lineal (3×3).
manipulability.m          Medida de manipulabilidad de Yoshikawa.
```

### Trayectoria, simulación y métricas (sin cambios)
```
QuinticTrajectory.m       Trayectoria quíntica rest-to-rest por tramos.
default_trajectory.m      Trayectoria articular de referencia (T = 5 s).
make_disturbance.m        Perturbación externa (senoidal + escalón).
simulate.m                Lazo cerrado: control ZOH 5 ms + RK4 sub-paso 1 ms.
metrics_from_logs.m       ISE, IAE, ISE_v, U2, u_max, e_max, pct_sat.
```

### Controladores base (versión 1)
```
CTC_PID.m                 Controlador CTC-PID con anti-windup condicional.
AdaptiveCTC.m             Control adaptativo indirecto: regresor PR
                          polinomial grado 2 (mínima norma) + sintonización
                          evolutiva online (Algoritmo 2 del paper).
monomials.m               Matriz de características [1, Z, monomios grado 2].
```

### ⭐ NUEVO: Redes Neuronales (capítulo NN del título)
```
NNRegressor.m             MLP (1 capa oculta tanh) para identificación
                          de sistemas: 9 entradas → 6 salidas; entrenado
                          por descenso de gradiente con momento y z-score.
NNAdaptiveCTC.m           AdaptiveCTC con el regresor NN en lugar de
                          monomials (misma arquitectura adaptativa).
```

### ⭐ NUEVO: Control Predictivo (capítulo MPC del título)
```
MPC.m                     Control Predictivo Basado en Modelo: horizonte
                          N_p/N_c, costo cuadrático ||q_ref − q_pred||² +
                          ρ||Δu||², restricciones de saturación y Δu; óptimo
                          por proyección de gradiente (sin toolboxes).
```

### ⭐ NUEVO: Control Robusto Híbrido (capítulo Híbrido del título)
```
HybridRobust.m            Controlador híbrido: τ = α·τ_CTC + β·τ_MPC +
                          γ·τ_NN. Pesos α, β por mezcla suave (softmax)
                          en función de presupuesto de cómputo y
                          magnitud del error. τ_NN predice el residual
                          e_pred = q_ref − q_next_pred y lo corrige con
                          M_nom · K_NN · e_pred.
```

### Optimizadores evolutivos (versión 1)
```
ode_opt.m                 Online Differential Evolution (DE/rand/1/bin).
oga_opt.m                 Online Genetic Algorithm (torneo, cruce aritmético).
opso_opt.m                Online Particle Swarm Optimization (w,c1,c2 estándar).
ocma_opt.m                Online CMA-ES (Hansen & Ostermeier, 2001).
get_optimizer.m           Despachador por nombre ("DE"|"GA"|"PSO"|"CMA").
```

### ⭐ NUEVO: Sintonía Multi-Objetivo (capítulo NSGA-II del título)
```
nsga2.m                   NSGA-II clásico (Deb et al. 2002): ordenamiento
                          por no-dominancia, distancia de hacinamiento,
                          torneo, SBX, mutación polinomial, elitismo
                          estricto. Devuelve el frente de Pareto final.
MultiObjAdaptiveCTC.m     AdaptiveCTC con NSGA-II multi-objetivo (3
                          objetivos: ISE_pred, ||u||², saturación) y
                          selección del "knee point" del frente de Pareto
                          (heurística de la distancia normalizada).
```

### ⭐ NUEVO: Modelos Sustitutos (capítulo Sustitutos del título)
```
RBFNet.m                  Red de Base Radial (kernel gaussiano) con
                          pesos por mínimos cuadrados ridge; soporta
                          salidas multi-dimensionales y devuelve varianza
                          epistémica (basada en activaciones máximas).
KrigingSurrogate.m        Kriging / GP con kernel exponencial cuadrado;
                          optimización de theta por sección áurea;
                          un GP por salida; devuelve media y varianza.
NNSurrogate.m             Envoltura sobre NNRegressor para uso genérico
                          como sustituto (soporta n_out > 1).
SurrogateManager.m       Gestor de ensambles de sustitutos con
                          muestreo adaptativo LIE (Likelihood of
                          Improvement Exploration): combina baja
                          predicción y alta incertidumbre para escoger
                          los k_lie candidatos a evaluar "de verdad".
SurrogateAdaptiveCTC.m    AdaptiveCTC con NSGA-II + SurrogateManager
                          (RBF/Kriging/NN): ~10× menos evaluaciones
                          reales con calidad comparable.
```

### Sintonía offline y experimentos (versión 1)
```
offline_tuning.m          Sintonía offline del CTC fijo (ODE, escenario ideal).
ensure_offline.m          Cachea/carga results/offline_gains.mat.
online_bounds.m           Cotas de sintonización online por ganancia.
save_offline_gains.m      Genera results/offline_gains.mat pre-poblado
                          con los valores offline del código Python.

scenarios_table.m         Configuración de los 4 escenarios S1-S4.
run_case.m                Una corrida (método × escenario × semilla).
friedman_analysis.m      Friedman + rangos + Wilcoxon-Holm.
holm_table.m              Formatea la tabla de comparaciones Holm.

experiment.m              SCRIPT PRINCIPAL (v1): 6 × 4 × 30 + estadística.
make_figures.m            Genera las 9 figuras del informe (v1).
```

### ⭐ NUEVO: Demos del proyecto extendido
```
demo_hybrid.m             CTC vs PR-DE vs MPC vs Hibrido (S2, ~30 s).
demo_nsga2.m              PR-DE vs NSGA-II + test ZDT1 (S2, ~40 s).
demo_surrogate.m          PR-DE vs NSGA-II vs Surrogate (S2, ~40 s).
experiment_hybrid.m       Experimento completo: 7 controladores × 4
                          escenarios × N corridas + figuras resumen.
                          (~5 min con runs=5; ~25 min con runs=30.)
```

### Verificaciones de planta (versión 1)
```
demo_smoke.m              DEMO de 20 s para el profesor (1 corrida CTC
                          vs. PR-DE en S2, con figura comparativa).
demo_dynamics.m           Verificación numérica: conservación de energía,
                          cancelación CTC, manipulabilidad.

results/                  Salidas (results.csv, summary.csv, stats.mat,
                          stats.json, budget.csv, traces/*.mat, figs/*.png).
```

## Requisitos

- **MATLAB R2016b o superior** (sin toolboxes especiales; la Statistics
  Toolbox se usa opcionalmente para `signrank`/`friedman` — si no está
  disponible, `friedman_analysis.m` incluye una implementación fallback
  con χ² asintótico). El NSGA-II y los tres sustitutos están
  implementados desde cero, sin dependencias externas.
- **GNU Octave 6.0+** (con paquetes `io` y `statistics` de Octave Forge
  instalados). En Octave algunas funciones gráficas pueden tener
  limitaciones (los 3D requieren OpenGL).

## Ejecución (orden recomendado)

```matlab
% 1) Verificar la planta (conservación de energía, cancelación CTC, manip.)
>> demo_dynamics

% 2) Cargar las ganancias offline cacheadas (idénticas al Python, 1 s)
>> save_offline_gains

% 3) Demo rápida de la versión base: CTC fija vs. PR-DE adaptativo en S2
>> demo_smoke

% ====== NUEVO: Demos del proyecto extendido ======

% 4) Demo del Control Robusto Híbrido (CTC + MPC + NN)
>> demo_hybrid

% 5) Demo del NSGA-II multi-objetivo (incluye test ZDT1)
>> demo_nsga2

% 6) Demo de sintonía asistida por sustitutos (RBF/Kriging/NN)
>> demo_surrogate

% 7) Experimento completo del proyecto extendido (7 controladores × 4 esc.)
>> experiment_hybrid                 % versión corta (5 runs, ~5 min)
>> experiment_hybrid('runs', 30)      % versión completa (~25 min)
>> experiment_hybrid('methods', {'CTC','HYBRID','SA-NSGA2'})  % subconjunto
```

## Mapeo entre título del proyecto y código MATLAB

| Componente del título                    | Código MATLAB                                |
|-----------------------------------------|----------------------------------------------|
| Modelado (cinemática/dinámica)          | `arm_params.m` · `mass_matrix.m` · `gravity_vector.m` · `friction_vector.m` · `bias_vector.m` · `inverse_dynamics.m` · `forward_dynamics.m` · `forward_kinematics.m` · `jacobian_arm.m` |
| Simulación (lazo cerrado)               | `simulate.m` · `QuinticTrajectory.m` · `default_trajectory.m` · `make_disturbance.m` |
| **Torque Computado**                    | `CTC_PID.m` · `inverse_dynamics.m` |
| **Control Predictivo**                  | `MPC.m` |
| **Redes Neuronales**                    | `NNRegressor.m` · `NNAdaptiveCTC.m` (regresor) · `NNSurrogate.m` (sustituto) |
| **Control Robusto Híbrido**             | `HybridRobust.m` |
| Control Adaptativo (versión 1)          | `AdaptiveCTC.m` · `monomials.m` |
| Sintonía evolutiva single-obj (v1)      | `ode_opt.m` · `oga_opt.m` · `opso_opt.m` · `ocma_opt.m` |
| **Sintonización Multi-Objetivo**        | `nsga2.m` · `MultiObjAdaptiveCTC.m` |
| **Modelos Sustitutos**                  | `RBFNet.m` · `KrigingSurrogate.m` · `NNSurrogate.m` · `SurrogateManager.m` · `SurrogateAdaptiveCTC.m` |
| Protocolo experimental (v1)             | `experiment.m` · `scenarios_table.m` · `run_case.m` · `offline_tuning.m` · `online_bounds.m` |
| **Experimento extendido**              | `experiment_hybrid.m` |
| Análisis estadístico (v1)               | `friedman_analysis.m` · `holm_table.m` |
| Métricas de desempeño                   | `metrics_from_logs.m` (ISE, IAE, U2, sat%) |

## Diferencias conocidas respecto al código Python (versión base)

1. **Generadores aleatorios**: MATLAB/Octave usan un flujo distinto al
   `numpy.random.default_rng` de Python. Los números aleatorios
   específicos DIFERIRÁN, pero el protocolo estadístico (semillas
   1..30, *common random numbers* entre alternativas) preserva la
   validez de las comparaciones.
2. **Paralelización**: el script `experiment.m` y `experiment_hybrid.m`
   corren en serie (sin `parfor`). El tiempo total en MATLAB R2020+ en
   un portátil es ~25-50 min para 720 corridas.
3. **CMA-ES**: la implementación usa `eig(C)` en cada generación (no
   schur), equivalente a la versión Python con `np.linalg.eigh`.
4. **NSGA-II**: implementación minimalista sin dependencias externas;
   los índices de cruce SBX y mutación polinomial son los del paper
   original (Deb et al. 2002).

## Reproducibilidad

- Semillas fijas por corrida (1..30) y números aleatorios comunes
  entre alternativas para el ruido de medición (CRN).
- `results/stats.mat` y `results/figs/` se regeneran desde
  `results/results.csv` con `make_figures(scen_keys, methods, runs)`.
- `results/hybrid/` almacena las salidas del experimento extendido.
- El experimento es reanudable: si se interrumpe, los resultados
  parciales quedan en `results/partial.jsonl`.

## Resultados esperados

### Versión 1 (control adaptativo single-objetivo)
| Escenario | CTC fija | Mejor adaptativo | Mejora x |
|---|---|---|---|
| S1 Nominal | ISE ≈ 5e-7 | ISE ≈ 3.8e-8 | 13.6× |
| S2 Incertidumbre | ISE ≈ 2.0e-2 | ISE ≈ 4.1e-3 (CMA-ES) | 4.9× |
| S3 +Perturbación | ISE ≈ 3.3e-1 | ISE ≈ 4.4e-2 (CMA-ES) | 7.4× |
| S4 +Ruido | ISE ≈ 3.3e-1 | ISE ≈ 5.2e-2 (CMA-ES) | 6.3× |

### Versión extendida (controladores híbridos)
- **HybridRobust** combina la robustez del CTC, la capacidad predictiva
  del MPC y la corrección del residual NN. En S2-S4 con simulaciones
  cortas (T=2 s), su ISE es comparable o mejor que el del AdaptiveCTC,
  con la ventaja de no requerir sintonía evolutiva online (pesos α, β
  se calculan en tiempo O(1) por ciclo).
- **NSGA-II** reduce el esfuerzo de control U2 respecto a la sintonía
  single-objetivo, al explicitar el compromiso (Pareto) entre ISE y U2.
  El knee point elegido se ubica típicamente en un 10-30 % menos U2 que
  el single-objetivo, con un ISE un 10-20 % mayor (compromiso).
- **SurrogateAdaptiveCTC** reduce el número de evaluaciones reales en
  ~10× (de 400/ciclo a ~35-50/ciclo) con ISE en el mismo orden de
  magnitud que la versión sin sustituto. El sustituto RBF es el más
  rápido (sin optimización de hiperparámetros); Kriging entrega
  además la varianza epistémica que guía la exploración LIE.

## Cómo presentar el proyecto extendido al profesor

1. `demo_dynamics` — verificar la planta.
2. `demo_smoke` — versión 1: comparación CTC vs PR-DE.
3. `demo_hybrid` — **NUEVO**: CTC vs PR-DE vs MPC vs Híbrido.
4. `demo_nsga2` — **NUEVO**: NSGA-II multi-objetivo + test ZDT1.
5. `demo_surrogate` — **NUEVO**: RBF/Kriging/NN + sintonía asistida.
6. `experiment_hybrid('runs', 5)` — **NUEVO**: 140 corridas (~5 min),
   7 controladores × 4 escenarios.
7. (Opcional) `experiment_hybrid('runs', 30)` — versión completa
   con 840 corridas (~25 min).
