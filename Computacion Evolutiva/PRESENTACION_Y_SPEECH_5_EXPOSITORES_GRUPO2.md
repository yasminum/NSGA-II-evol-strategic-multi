# Sustentación Magistral (15 Minutos): 5 Expositores × 3 Minutos + Demo MuJoCo 3D en Vivo

> [!IMPORTANT]
> **Estructura Oficial de la Exposición (15:00 min totales — 3:00 min exactos por integrante)**
> * **Diapositivas PDF Widescreen (16:9) listas para proyectar**: [Presentacion_Final_CE_Grupo2_20de20.pdf](file:///c:/Users/yurrego/OneDrive%20-%20EFICITEC%20S.A.C/Documentos/COMPUTACION%20EVOLUTIVA/PrimerTrabajoGrupo2/Presentacion_Final_CE_Grupo2_20de20.pdf)
> * **Lanzador 1-Clic del Simulador para el Cierre de José Luis**: [LANZAR_SIMULADOR_MUJOCO_NSGA2.bat](file:///C:/Users/yurrego/OneDrive%20-%20EFICITEC%20S.A.C/Documentos/ROBOTICA%20Y%20SISTEMAS%20AUTONOMOS/EXAMEN%20PARCIAL/ROBOT_IA_3DOF_v0/ROBOT_IA_3DOF_v0/LANZAR_SIMULADOR_MUJOCO_NSGA2.bat)
> * **Repositorio GitHub**: https://github.com/yasminum/NSGA-II-evol-strategic-multi

| Orden | Expositor(a) | Ventana de Tiempo | Diapositivas | Rol Estratégico en la Rúbrica del Dr. Túpac |
| :---: | :--- | :---: | :---: | :--- |
| **1** | **Amit Roy Flores Rivera** | `00:00 – 03:00` | Slides 1–3 | **Fase I (Modelado)**: Física Euler-Lagrange 3-DOF, por qué fracasa el diseño clásico ($\omega_n=13, \zeta=1.1$) y formulación matemática multiobjetivo en MuJoCo. |
| **2** | **Franklin Enrique Alegría Barboza** | `03:00 – 06:00` | Slides 4–5 | **Fases II y III (Algoritmo y Software)**: Arquitectura NSGA-II (SBX, PM, Torneo Aglomerado), selector post-Pareto utópico, paralelización $18.7\times$ y Frente de Pareto hallado. |
| **3** | **Raquel Medianero Huari** | `06:00 – 09:00` | Slides 6–7 | **Fase IV-A (Dinámica Evolutiva y Biomecánica)**: Convergencia generacional, Hipervolumen $HV(g)$, estudio de sensibilidad ($S_1\dots S_9$) y ley biomecánica emergente ($K_{d1} \gg K_{d2} \gg K_{d3}$). |
| **4** | **Yasmín Mariela Urrego Montoya** | `09:00 – 12:00` | Slides 8–9 | **Fase IV-B (Benchmark y Confrontación Hostil $N=30$)**: Comparación contra DE/CMA-ES/MPC y auditoría adversarial en MuJoCo hostil (ruido, retardo $10\text{ ms}$, genotipo modal $[\omega_n,\zeta]$, RBF y Wilcoxon-Holm). |
| **5** | **José Luis Espinoza Chamaya** | `12:00 – 15:00` | Slide 10 + **Simulador MuJoCo en Vivo** | **Cierre Magistral (Gemelo Digital 3D en Vivo + Conclusiones)**: Ejecución en tiempo real de los 3 actos en MuJoCo (captura $<2\text{ mm}$, golpe externo 3D y circuito de 5 aros en modo Turbo) + Conclusiones, Limitaciones y Trabajos Futuros. |

---

## EXPOSITOR 1: Amit Roy Flores Rivera (`00:00 – 03:00`)
**Tema:** *Fase I — Modelado Físico del Manipulador Espacial 3-DOF y Formulación Matemática del Problema Evolutivo*
**Diapositivas:** Slides 1, 2 y 3 de [Presentacion_Final_CE_Grupo2_20de20.pdf](file:///c:/Users/yurrego/OneDrive%20-%20EFICITEC%20S.A.C/Documentos/COMPUTACION%20EVOLUTIVA/PrimerTrabajoGrupo2/Presentacion_Final_CE_Grupo2_20de20.pdf)

![Gemelo Digital 3D MuJoCo](/C:/Users/yurrego/.gemini/antigravity/brain/25a616a6-cf4e-444b-b49e-9a332bb324ee/fig_estados_3d_enceste.png)

### Speech Palabra por Palabra (Cronometrado a 140 palabras/min)

* **`[00:00 – 01:00]` (Slide 1 $\to$ Slide 2: El Problema Físico y por qué falla el Diseño Analítico):**
  > "Buenas tardes, Doctor Yván Túpac y compañeros. Somos el Grupo 2 y presentamos la sintonización evolutiva robusta de un Controlador por Torque Computado aplicado a un manipulador robótico espacial antropomórfico de 3 grados de libertad en MuJoCo.
  >
  > *(Pasar a Slide 2)* La dinámica de nuestro robot RRR —cintura, hombro y codo accionados por servomotores Dynamixel AX-18A y AX-12A— está gobernada por la ecuación no lineal acoplada de Euler-Lagrange: $M(q)\ddot{q} + C(q,\dot{q})\dot{q} + g(q) + f_v\dot{q} = \tau + \tau_{\text{ext}}$.
  > En la ingeniería de control clásica, el método de Torque Computado utiliza un modelo nominal $\hat{M}(q)$ para cancelar las no linealidades y asigna ganancias uniformes colocando polos en $\omega_n = 13\text{ rad/s}$ y $\zeta = 1.1$, lo que produce $K_p = 169$ y $K_d = 28.6$ idénticos en las tres articulaciones.
  > **¿Por qué fracasa este diseño clásico en la planta real?** Por tres razones físicas severas: primero, una **asimetría inercial de 10 a 1** entre la base que mueve toda la cadena y el codo distal limitado a solo $0.45\text{ N}\cdot\text{m}$; segundo, la **saturación dura no diferenciable** del actuador; y tercero, que cuando la masa real varía en $+25\%$ y la fricción en $+30\%$, $\hat{M}(q)$ ya no cancela a $M(q)$ y el paisaje de rendimiento pierde toda solución analítica cerrada."

* **`[01:00 – 02:10]` (Slide 3: Cromosoma, Escenarios Robustos y Funciones Objetivo en Conflicto):**
  > "*(Pasar a Slide 3)* Para resolver este problema sin derivadas analíticas, formulamos un problema de optimización continua sobre un cromosoma real de seis genes: $\theta = [K_{p1}, K_{p2}, K_{p3}, K_{d1}, K_{d2}, K_{d3}]^T$ acotado en $[20, 250]^3 \times [3, 55]^3$.
  > En lugar de evaluar una trayectoria ideal aislada, nuestro Gemelo Digital en MuJoCo —que integra la física a $500\text{ Hz}$ y el control a $100\text{ Hz}$— somete a cada individuo a **cuatro escenarios dinámicos de 9 segundos**:
  > 1) Régimen nominal con trayectoria quíntica $\mathcal{C}^2$;
  > 2) Perturbación externa tridimensional inyectada entre $6.0$ y $6.6\text{ segundos}$;
  > 3) Incertidumbre paramétrica con $+25\%$ de masa en el eslabón 1, $+15\%$ en el eslabón 2 y $+30\%$ de fricción viscosa; y
  > 4) El régimen combinado extremo."

* **`[02:10 – 03:00]` (Slide 3: Objetivos $f_1, f_2, f_3$ y Pase al Expositor 2):**
  > "Sobre estos cuatro escenarios minimizamos simultáneamente dos objetivos en conflicto directo:
  > * **Objetivo $f_1(\theta)$:** el error cuadrático medio de seguimiento cartesiano $\text{RMSE}$ en milímetros de la punta del efector.
  > * **Objetivo $f_2(\theta)$:** la energía integral cuadrática de torque $\int_0^T \|\tau(t)\|_2^2\,dt$ en $\text{N}^2\text{m}^2\text{s}$, que mide el esfuerzo y calentamiento Joule de los servomotores.
  > * Y en nuestra extensión adversarial en gemelo hostil, incorporamos además el **tercer objetivo $f_3(\theta) = \int \|\dot{\tau}\|^2\,dt$** para suprimir el *chattering* de alta frecuencia bajo ruido de encoder.
  >
  > Doy pase a Franklin Alegría para explicar el diseño del algoritmo evolutivo NSGA-II y nuestra arquitectura de software paralelizada."

---

## EXPOSITOR 2: Franklin Enrique Alegría Barboza (`03:00 – 06:00`)
**Tema:** *Fases II y III — Diseño del Algoritmo NSGA-II, Regla de Decisión Post-Pareto, Paralelización $18.7\times$ y Frente de Pareto*
**Diapositivas:** Slides 4 y 5 de [Presentacion_Final_CE_Grupo2_20de20.pdf](file:///c:/Users/yurrego/OneDrive%20-%20EFICITEC%20S.A.C/Documentos/COMPUTACION%20EVOLUTIVA/PrimerTrabajoGrupo2/Presentacion_Final_CE_Grupo2_20de20.pdf)

![Frente de Pareto Bi-Criterio](/C:/Users/yurrego/.gemini/antigravity/brain/25a616a6-cf4e-444b-b49e-9a332bb324ee/fig_ce_01_frente_pareto.png)

### Speech Palabra por Palabra (Cronometrado a 140 palabras/min)

* **`[03:00 – 04:05]` (Slide 4: Justificación de NSGA-II y Operadores Genéticos Continuos):**
  > "Gracias, Amit. *(Mostrar Slide 4)* En la Fase II seleccionamos el paradigma **NSGA-II con codificación de punto flotante** por tres razones fundamentales frente a un algoritmo genético monobjetivo o discreto:
  > Primero, nuestros objetivos tienen escalas dispares —$f_1$ varía entre $7.6$ y $69\text{ mm}$, mientras que $f_2$ varía apenas entre $0.44$ y $0.52\text{ N}^2\text{m}^2\text{s}$—; la dominancia de Pareto es invariante a la escala y no requiere inventar pesos arbitrarios a priori.
  > Segundo, empleamos **Cruce Binario Simulado (SBX)** con probabilidad $p_c = 0.90$ e índice $\eta_c = 15$, cuyo poder de búsqueda se adapta automáticamente a la separación entre padres sin sufrir acantilados de Hamming, acoplado con **Mutación Polinomial (PM)** con $p_m = 1/6$ (mutando en promedio un gen por cromosoma) y $\eta_m = 20$.
  > Tercero, aplicamos reemplazo elitista $(\mu+\lambda)$ con **Torneo Binario Aglomerado ($\prec_c$)** y eliminación de duplicados, garantizando conservar siempre los extremos del frente mientras la *Crowding Distance* rellena uniformemente la curva de compromiso."

* **`[04:05 – 05:00]` (Slide 4: Selector Post-Pareto y Paralelización Multiproceso $18.7\times$):**
  > "Una vez que NSGA-II entrega las 24 soluciones del frente de Pareto $\mathcal{F}_1$, ¿cómo elegimos sin sesgo humano el controlador que va al robot? Diseñamos un **selector automático post-Pareto en dos etapas**: primero filtramos el subconjunto $\mathcal{F}_{\text{dom}}$ que supera en precisión al controlador analítico base, y dentro de ese cono elegimos el individuo $\theta^*$ que minimiza la distancia euclidiana $L_2$ normalizada al punto ideal utópico $z^{\min}$.
  > En la Fase III de ingeniería de software resolvimos el cuello de botella computacional: evaluar un individuo exige integrar $18\,000$ pasos de física 3D en MuJoCo. Pre-cargando en memoria RAM los modelos nominal e incierto dentro de un pool multiproceso de 12 núcleos, redujimos el tiempo por evaluación de $1.16\text{ segundos}$ a solo **$0.062\text{ segundos}$** —un **speedup real de $18.7$ veces**—, ejecutando $1\,920$ simulaciones dinámicas ($4.8\text{ horas}$ de operación física del robot) en **29.88 segundos**."

* **`[05:00 – 06:00]` (Slide 5: Frente de Pareto Obtenido y Dominancia Estricta):**
  > "*(Pasar a Slide 5)* En esta gráfica observamos el espacio objetivo y los frentes de Pareto obtenidos. El punto rojo representa el controlador clásico CTC Base con $11.353\text{ mm}$ de error y $0.5159$ de esfuerzo.
  > La región sombreada en verde es el **cono de dominancia estricta de Pareto**: todas las soluciones evolucionadas dentro de esta caja ganan simultáneamente en precisión y en ahorro de energía.
  > Nuestro extremo $\#1$ reduce el error a **$7.640\text{ mm}$** ($-32.7\%$), y nuestra solución compromiso $\theta^*$ alcanza **$7.827\text{ mm}$** (**$-31.06\%$ de error**) reduciendo a la vez el esfuerzo energético a $0.5114\text{ N}^2\text{m}^2\text{s}$. Doy pase a Raquel Medianero para el análisis de convergencia, sensibilidad y biomecánica del cromosoma."

---

## EXPOSITORA 3: Raquel Medianero Huari (`06:00 – 09:00`)
**Tema:** *Fase IV-A — Convergencia Generacional, Hipervolumen Dominado, Sensibilidad Hiperparamétrica ($S_1\dots S_9$) y Ley Biomecánica*
**Diapositivas:** Slides 6 y 7 de [Presentacion_Final_CE_Grupo2_20de20.pdf](file:///c:/Users/yurrego/OneDrive%20-%20EFICITEC%20S.A.C/Documentos/COMPUTACION%20EVOLUTIVA/PrimerTrabajoGrupo2/Presentacion_Final_CE_Grupo2_20de20.pdf)

````carousel
![Convergencia Generacional e Hipervolumen](/C:/Users/yurrego/.gemini/antigravity/brain/25a616a6-cf4e-444b-b49e-9a332bb324ee/fig_ce_02_convergencia_generacional.png)
<!-- slide -->
![Sensibilidad Paramétrica S1-S9](/C:/Users/yurrego/.gemini/antigravity/brain/25a616a6-cf4e-444b-b49e-9a332bb324ee/fig_ce_03_sensibilidad_y_comparacion.png)
<!-- slide -->
![Estructura Biomecánica del Cromosoma](/C:/Users/yurrego/.gemini/antigravity/brain/25a616a6-cf4e-444b-b49e-9a332bb324ee/fig_ce_04_ganancias_cromosoma.png)
````

### Speech Palabra por Palabra (Cronometrado a 140 palabras/min)

* **`[06:00 – 07:05]` (Slide 6 Izquierda: Dinámica Poblacional, Paradoja del Promedio e Hipervolumen):**
  > "Gracias, Franklin. *(Mostrar Slide 6, panel izquierdo)* Para auditar la dinámica interna de NSGA-II a lo largo de las 20 generaciones, rastreamos tres indicadores: los extremos elitistas, el promedio poblacional y el **Hipervolumen Dominado normalizado $HV(g)$**.
  > Primero, confirmamos la **monotonía elitista**: el mejor RMSE cae sin retrocesos desde $8.316\text{ mm}$ en la generación 1 hasta **$7.640\text{ mm}$** en la generación 20, mientras el mínimo esfuerzo cae de $0.4999$ a **$0.4466\text{ N}^2\text{m}^2\text{s}$**.
  > Segundo, explicamos un fenómeno clave de la computación evolutiva multiobjetivo: **¿por qué el RMSE promedio poblacional $\bar{f}_1$ sube de $15.6\text{ mm}$ a $38.6\text{ mm}$ mientras el esfuerzo promedio baja?** En optimización monobjetivo eso parecería divergencia, pero en NSGA-II ocurre porque el operador de *Crowding Distance* descubre y puebla progresivamente la cola derecha del frente: soluciones ecológicas que ahorran mucha batería a cambio de menor rigidez en el hombro.
  > Tercero, el Hipervolumen crece desde $0.4834$ hasta estabilizarse asintóticamente en **$0.5489$** desde la generación 10, manteniendo una diversidad genotípica sana de $\sigma_\theta = 0.153 > 0$, lo que descarta cualquier convergencia prematura."

* **`[07:05 – 08:00]` (Slide 6 Derecha: Estudio de Sensibilidad Hiperparamétrica $S_1$ a $S_9$):**
  > "*(Señalar panel derecho de Slide 6)* Para no elegir hiperparámetros a ciegas, ejecutamos un estudio factorial de **9 configuraciones ($S_1$ a $S_9$)**. Demostramos que la configuración $S_3$ ($N_{\text{pop}}=24, G=20$, 480 evaluaciones) supera en Hipervolumen ($0.5489$ vs. $0.5422$) y en precisión ($7.827\text{ mm}$ vs. $7.912\text{ mm}$) a una población más ancha pero menos profunda como $S_4$ ($N_{\text{pop}}=36, G=15$, 540 evaluaciones) ahorrando $21\%$ de tiempo de cómputo. Asimismo, demostramos que una mutación muy baja ($S_7, p_m=0.05$) estanca el frente en $8.98\text{ mm}$, mientras que una mutación agresiva de paso largo ($S_9, \eta_m=5$) destruye los bloques constructores y degrada el Hipervolumen a $0.5119$."

* **`[08:00 – 09:00]` (Slide 7: Interpretación Física y Biomecánica del Cromosoma $\theta^*$):**
  > "*(Pasar a Slide 7)* Al inspeccionar los genes del cromosoma ganador $\theta^*_{\text{tesis}}$, descubrimos que NSGA-II encontró por sí solo una ley biomecánica asimétrica:
  > 1) **Rigidización proporcional ($K_{p,i} > 169$)** en las tres articulaciones para vencer la incertidumbre de masa $+25\%$ y rechazar perturbaciones; y
  > 2) Un **gradiente decreciente de amortiguamiento derivativo ($K_{d1} = 49.65 \gg K_{d2} = 17.33 \gg K_{d3} = 4.20$)**. NSGA-II sobreamortigua la cintura ($\zeta_1 = 1.74$) porque soporta la inercia rotacional de todo el brazo, y reduce la ganancia derivativa del codo ligero ($\zeta_3 = 0.133$). Ahora bien, ¿es ese bajo amortiguamiento distal $\zeta_3 = 0.133$ robusto si añadimos ruido real de sensores? Doy pase a Yasmín Urrego para responder esta pregunta con nuestro benchmark y la confrontación estadística de 30 semillas."

---

## EXPOSITORA 4: Yasmín Mariela Urrego Montoya (`09:00 – 12:00`)
**Tema:** *Fase IV-B — Benchmark Multi-Paradigma y Confrontación Adversarial en Gemelo MuJoCo Hostil ($N=30$ Semillas, Genotipo Modal, RBF y Wilcoxon-Holm)*
**Diapositivas:** Slides 8 y 9 de [Presentacion_Final_CE_Grupo2_20de20.pdf](file:///c:/Users/yurrego/OneDrive%20-%20EFICITEC%20S.A.C/Documentos/COMPUTACION%20EVOLUTIVA/PrimerTrabajoGrupo2/Presentacion_Final_CE_Grupo2_20de20.pdf)

````carousel
![Validación Operativa entre Controladores](/C:/Users/yurrego/.gemini/antigravity/brain/25a616a6-cf4e-444b-b49e-9a332bb324ee/fig_curvas_simulacion.png)
<!-- slide -->
![Confrontación Hostil 30 Semillas](/C:/Users/yurrego/.gemini/antigravity/brain/25a616a6-cf4e-444b-b49e-9a332bb324ee/fig_ce_05_confrontacion_hostil_30semillas.png)
````

### Speech Palabra por Palabra (Cronometrado a 140 palabras/min)

* **`[09:00 – 09:55]` (Slide 8: Benchmark contra 6 Optimizadores y 5 Controladores a $100\text{ Hz}$):**
  > "Gracias, Raquel. *(Mostrar Slide 8)* Nuestra validación comparativa se ejecutó en dos niveles.
  > En el **primer nivel**, contrastamos NSGA-II contra cinco algoritmos sobre el mismo Gemelo Digital: Random Search, Nelder-Mead Simplex, Evolución Diferencial (`DE/rand/1/bin`) y **CMA-ES**. Mientras CMA-ES y DE optimizan una función escalarizada fija logrando $7.576\text{ mm}$, **NSGA-II iguala virtualmente ese extremo ($7.640\text{ mm}$), logra el menor gasto energético en su solución compromiso ($0.5114\text{ N}^2\text{m}^2\text{s}$)** y entrega 24 controladores óptimos en una sola ejecución.
  > En el **segundo nivel**, validamos el controlador evolucionado a $100\text{ Hz}$ frente a PD clásico y Control Predictivo MPC: nuestro controlador evolutivo tarda apenas **$0.018\text{ milisegundos}$ por ciclo** —$810$ veces más rápido que los $14.6\text{ ms}$ del MPC—, reduciendo el error nominal en **$-33.5\%$** ($4.57\text{ mm}$ vs. $6.88\text{ mm}$) y el tiempo de recuperación post-impacto en **$-34.4\%$** ($0.382\text{ s}$ vs. $0.582\text{ s}$)."

* **`[09:55 – 11:05]` (Slide 9: Auditoría Adversarial en Gemelo Digital Hostil y Reparametrización Modal):**
  > "*(Pasar a Slide 9)* Sin embargo, nos hicimos una autocrítica científica rigurosa: en el gemelo ideal sin ruido, NSGA-II llevó el codo a la frontera $K_{p3}\to 250, K_{d3}\to 4.2$ ($\zeta_3 = 0.133$). ¿Qué ocurre cuando existen imperfecciones reales de hardware?
  > Construimos un **Gemelo Digital MuJoCo Hostil** con cinco mejoras estructurales:
  > 1) Inyectamos ruido gaussiano de encoders ($\sigma_q = 0.002\text{ rad}$, $\sigma_{\dot{q}} = 0.02\text{ rad/s}$), retardo de transporte de $10\text{ ms}$ y un **mini-batch espacial de trayectorias 3D** muestreado por *Latin Hypercube Sampling* bajo Números Aleatorios Comunes (CRN);
  > 2) Formulamos el problema **Tri-Objetivo en $\mathbb{R}^3$** añadiendo la tasa de par anti-chattering $f_3 = \int \|\dot{\tau}\|^2\,dt$ con agregación robusta media–peor caso ($0.5\mu + 0.5\max$);
  > 3) **Reparametrizamos el genotipo en coordenadas modales físicas $\tilde{\theta} = [\omega_{n,i}, \zeta_i]$**, decodificando $K_{p,i} = \omega_{n,i}^2$ y $K_{d,i} = 2\zeta_i\omega_{n,i}$, lo que alinea el cruce SBX con las variedades de amortiguamiento constante e impone $\zeta_i \ge 0.35$ como una simple cota de caja; y
  > 4) Implementamos el controlador **C5 con Gain-Scheduling no lineal de 12 genes**, asistido por un modelo sustituto **RBF con criterio Lower Confidence Bound** y búsqueda local memética."

* **`[11:05 – 12:00]` (Slide 9: Veredicto Estadístico de 30 Semillas con Friedman y Wilcoxon-Holm):**
  > "Al confrontar los 5 controladores a lo largo de **$N = 30$ semillas independientes ($1\,800$ simulaciones 3D en MuJoCo)** mediante pruebas no paramétricas de Friedman ($p = 6.36 \times 10^{-24}$) y Wilcoxon con corrección de Holm-Bonferroni, demostramos dos hallazgos decisivos:
  > Primero, el cromosoma de borde $\zeta_3 = 0.133$ (**C2**) conserva buena precisión ($9.71\text{ mm}$), pero **amplifica el ruido del encoder disparando el chattering $f_3$ en $+64.0\%$** ($3.684$ vs. $2.247$ del Base, peor rango de Friedman $5.00$).
  > Segundo, nuestros controladores evolucionados **C4 (Modal, $\zeta \in [0.81, 1.28]$)** y **C5 (Gain-Scheduled + RBF + Memético)** dominan estrictamente al CTC Base en los tres objetivos y **suprimen el chattering en un $-47.8\%$ respecto a C2 y en un $-14.5\%$ respecto al propio CTC Base analítico** ($p_{\text{Holm}} = 7.45 \times 10^{-9}$).
  > Ahora doy pase a José Luis Espinoza para la demostración en vivo de nuestro Gemelo Digital MuJoCo 3D y las conclusiones finales."

---

## EXPOSITOR 5 (CIERRE ESTELAR): José Luis Espinoza Chamaya (`12:00 – 15:00`)
**Tema:** *Demostración Interactiva en Vivo del Simulador 3D MuJoCo + Conclusiones, Limitaciones y Trabajos Futuros*
**Diapositivas y Ventana:** Slide 10 de [Presentacion_Final_CE_Grupo2_20de20.pdf](file:///c:/Users/yurrego/OneDrive%20-%20EFICITEC%20S.A.C/Documentos/COMPUTACION%20EVOLUTIVA/PrimerTrabajoGrupo2/Presentacion_Final_CE_Grupo2_20de20.pdf) + **Ventana en Vivo de [LANZAR_SIMULADOR_MUJOCO_NSGA2.bat](file:///C:/Users/yurrego/OneDrive%20-%20EFICITEC%20S.A.C/Documentos/ROBOTICA%20Y%20SISTEMAS%20AUTONOMOS/EXAMEN%20PARCIAL/ROBOT_IA_3DOF_v0/ROBOT_IA_3DOF_v0/LANZAR_SIMULADOR_MUJOCO_NSGA2.bat)**

> [!TIP]
> **Preparación Técnica para José Luis antes de iniciar la exposición (Toma 5 segundos):**
> Antes de que empiece a hablar Amit (o mientras habla Yasmín en el minuto 11:30), haz doble clic en [LANZAR_SIMULADOR_MUJOCO_NSGA2.bat](file:///C:/Users/yurrego/OneDrive%20-%20EFICITEC%20S.A.C/Documentos/ROBOTICA%20Y%20SISTEMAS%20AUTONOMOS/EXAMEN%20PARCIAL/ROBOT_IA_3DOF_v0/ROBOT_IA_3DOF_v0/LANZAR_SIMULADOR_MUJOCO_NSGA2.bat) y déjalo listo en segundo plano con `Alt + Tab`. Así, en el minuto `12:00` exacto, pasas al simulador 3D sin esperar un solo segundo de carga.

### Speech Palabra por Palabra + Coreografía en Vivo en el Simulador (Cronometrado)

* **`[12:00 – 12:50]` (Acto 1 en Vivo: Panel Científico, Cromosoma NSGA-II y Captura Verde Esmeralda `< 2.0 mm`):**
  > "Muchas gracias, Yasmín. Doctor Túpac, compañeros: para demostrar que nuestros resultados no son solo números en tablas estáticas, tengo corriendo en tiempo real nuestro **Gemelo Digital 3D en MuJoCo a $500\text{ Hz}$** gobernado por el controlador evolucionado con NSGA-II.
  >
  > *(Hacer `Alt + Tab` al Simulador MuJoCo: mostrar a la izquierda el Panel Científico y a la derecha el Visor OpenGL 3D)*
  > Observen en el panel izquierdo el bloque **'Ley de Control Activa: CTC_EVO (NSGA-II)'**, que inyecta en línea el cromosoma óptimo desde `best_ctc_gains.json` junto con las matrices dinámicas $\hat{M}(q)$ y $\hat{h}(q,\dot{q})$.
  > En el visor 3D derecho *(rotar ligeramente la cámara con clic izquierdo para mostrar la perspectiva 3D)*, vemos cómo el efector final sigue la trayectoria polinomial suave hasta la esfera objetivo. En el instante en que ingresa al umbral sub-milimétrico menor a **$2.0\text{ milímetros}$**, la esfera cambia a **Verde Esmeralda**, estabilizándose con un error estacionario de apenas **$0.068\text{ mm}$** —doce veces más preciso que el diseño analítico— y con **$0.0\%$ de saturación** en los tres servomotores Dynamixel."

* **`[12:50 – 13:35]` (Acto 2 en Vivo: Disparo de Perturbación Externa 3D $\to$ Estado Magenta $\to$ Recuperación en `0.382 s`):**
  > "Ahora pongamos a prueba en vivo la robustez física que evolucionó NSGA-II.
  > *(Hacer clic en el botón de **Aplicar Perturbación / Impulso Externo** en la interfaz mientras el brazo sostiene la esfera)*
  > Acabo de inyectar un pulso perturbador tridimensional directo sobre las articulaciones. Observen cómo el indicador cambia inmediatamente a color **Magenta (Perturbación Activa)** y en las curvas inferiores se registra el pico de par correctivo; sin embargo, gracias a la alta rigidez proporcional evolucionada y al amortiguamiento desacoplado de la cintura y el hombro, el brazo absorbe el impacto sin oscilar y recupera el estado **Verde Esmeralda en apenas $0.382\text{ segundos}$**, superando ampliamente los $0.582\text{ segundos}$ del controlador clásico."

* **`[13:35 – 14:15]` (Acto 3 en Vivo: Circuito Autónomo de 5 Aros en Modo Rápido / Turbo):**
  > "Finalmente, activemos el desafío más exigente del simulador: el **Modo Juego — Circuito Espacial de 5 Aros** con trayectorias séptuples de clase $\mathcal{C}^3$.
  > *(Activar en el panel el **Circuito de 5 Aros** y cambiar la velocidad de **Estándar** a **Rápido / Turbo**)*
  > Observen cómo el manipulador recorre secuencialmente los cinco aros tridimensionales en régimen **Turbo ($1.4\text{ segundos}$ por tramo)**, manteniendo el índice de manipulabilidad de Yoshikawa $w(q)$ lejos de singularidades y logrando **5 de 5 encestes perfectos** con movimiento suave y libre de *chattering*."

* **`[14:15 – 15:00]` (Slide 10: Conclusiones Definitivas, Limitación Honesta y Trabajos Futuros — Cierre de Oro):**
  > "*(Volver a Slide 10 para el cierre formal)* Para concluir nuestra presentación, sintetizamos tres mensajes finales:
  > 1. **En Logros:** Demostramos que NSGA-II domina estrictamente al diseño analítico clásico reduciendo el RMSE robusto en **$-31.06\%$**, y probamos estadísticamente en el gemelo MuJoCo hostil ($N=30$ semillas, Friedman y Wilcoxon-Holm) que la reparametrización modal $[\omega_n, \zeta]$ con *Gain-Scheduling* y sustituto RBF **suprime el chattering en un $-47.8\%$** ($p_{\text{Holm}} = 7.45 \times 10^{-9}$).
  > 2. **En Limitaciones (con total rigor metodológico):** Nuestro protocolo de $N=30$ semillas certifica la robustez del *controlador* frente a ruido y trayectorias LHS no vistas, evaluando controladores fijos evolucionados una única vez; medir la varianza del *algoritmo* entre 30 corridas evolutivas independientes queda abierto como línea de medición.
  > 3. **En Trabajos Futuros:** Proponemos escalar hacia **Coevolución Competitiva Host–Parasite ($\min\text{--}\max$)** que evolucione perturbaciones adversarias, **NSGA-III en $\mathbb{R}^4$** añadiendo la manipulabilidad $-w_{\min}(q)$, y **Programación Genética / NEAT residual** para transferencia *Sim-to-Real*.
  >
  > Todo el código, los datos de las 30 semillas y este simulador están publicados en nuestro GitHub. Muchas gracias, Doctor Túpac; quedamos atentos a sus preguntas."

---

## Blindaje de Preguntas y Respuestas (Q&A Cheat-Sheet para los 5 Integrantes)

| Posible Pregunta del Dr. Yván Túpac | Quién Responde | Respuesta Quirúrgica de 20 Segundos |
| :--- | :---: | :--- |
| **1. "¿Por qué el RMSE promedio poblacional $\bar{f}_1(g)$ sube de $15.6\text{ mm}$ a $38.6\text{ mm}$ en la Figura 2 si el algoritmo está convergiendo?"** | **Raquel Medianero** | *"Porque en NSGA-II no optimizamos un único escalar sino la extensión de toda la frontera de Pareto. Mientras el extremo elitista $f_1^{\min}$ baja monótonamente de $8.31$ a $7.64\text{ mm}$, el operador de Crowding Distance descubre y puebla la cola derecha de mínimo consumo energético ($f_2 \to 0.4466$), desplazando la media aritmética del frente completo mientras el Hipervolumen crece hasta $0.5489$."* |
| **2. "¿Por qué en la Tabla 2 el codo evolucionado tiene $K_{d3} = 4.20$ ($\zeta_3 = 0.133$)? ¿No es peligroso subamortiguar tanto una articulación?"** | **Yasmín Urrego** | *"Excelente observación, Doctor. Justamente auditamos ese hallazgo en la Sección 4.6: en el gemelo ideal sin ruido, NSGA-II aprovecha la fricción viscosa física de MuJoCo y baja $K_{d3}$ para ahorrar torque. Pero cuando inyectamos ruido de encoder ($\sigma_q=0.002\text{ rad}$) y retardo de $10\text{ ms}$ en 30 semillas, ese $\zeta_3=0.133$ amplifica el ruido en $+64\%$ de chattering. Al reparametrizar el cromosoma en coordenadas modales $[\omega_n, \zeta]$ e incluir $f_3 = \int \|\dot{\tau}\|^2 dt$, NSGA-II evoluciona $\zeta \in [0.81, 1.28]$ y suprime el chattering en $-47.8\%$ ($p_{\text{Holm}} = 7.45 \times 10^{-9}$)."* |
| **3. "¿Por qué usar NSGA-II si CMA-ES obtuvo $7.576\text{ mm}$ con 160 evaluaciones?"** | **Franklin Alegría** | *"Porque CMA-ES requiere fijar a priori una suma ponderada arbitraria $0.5\hat{f}_1 + 0.5\hat{f}_2$ y entrega un único punto. NSGA-II alcanza virtualmente la misma precisión en su extremo #1 ($7.640\text{ mm}$, apenas $0.8\%$ de diferencia), logra menor consumo energético en su solución compromiso ($0.5114$ vs. $0.5136$) y entrega 24 controladores óptimos en una sola corrida."* |
| **4. "¿Por qué los valores de $f_2$ en la Sección 4.6 ($0.44\text{ N}^2\text{m}^2\text{s}$) o en su gráfica hostil difieren de la Figura 1?"** | **Amit Flores** | *"Por dos razones explícitas en la Sección 4.6: primero, en el experimento hostil evaluamos un mini-batch de trayectorias 3D generadas por Latin Hypercube Sampling bajo la agregación robusta media–peor caso ($\lambda = 0.5$); y segundo, interviene la energía adicional asociada al rechazo del ruido de cuantización bajo Números Aleatorios Comunes (CRN)."* |
| **5. "¿Las 30 semillas son 30 ejecuciones de NSGA-II desde cero o 30 bancos estocásticos de validación?"** | **José Luis Espinoza** | *"Como declaramos expresamente en la Sección 4.7 de Limitaciones, las 30 semillas certifican la robustez operativa de los 5 controladores frente a 30 bancos independientes de trayectorias LHS y ruido en MuJoCo 3D ($1\,800$ simulaciones con diseño en bloques de Friedman y CRN); medir la varianza inter-corrida de 30 evoluciones completas desde cero queda planteado como línea abierta."* |
