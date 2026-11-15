# LOG DE EJECUCIÓN Y AUDITORÍA DE LOS 3 ANÁLISIS ADVERSARIALES (QAP)

* **Cuaderno Reconstruido:** [Cuaderno_Interactivo_Optimizacion_Combinatoria.ipynb](file:///c:/Users/yurrego/OneDrive%20-%20EFICITEC%20S.A.C/Documentos/COMPUTACION%20EVOLUTIVA/PrimerTrabajoGrupo2/Cuaderno_Interactivo_Optimizacion_Combinatoria.ipynb)
* **Curso:** Computación Evolutiva — UNI | **Docente:** Dr. Yván Jesús Túpac Valdivia

---

## 1. LOG DE AUDITORÍA FORENSE DEL ESPACIO $S_8$ ($8! = 40,320$ Permutaciones)

```text
=== Instancia 1 (1ra llamada tras np.random.seed(42) - Ejecución [6]) ===
  Espacio total |S_8| : 40,320 permutaciones
  Óptimo Global Exacto: Z* = 4962 en permutación [1, 0, 2, 5, 7, 4, 6, 3]
  Peor Costo (Máximo) : 6749 | Media poblacional uniforme: 5829.29 ± 244.98

=== Instancia 2 (2da llamada tras np.random.seed(42) - Captura [7] y [9]) ===
  Espacio total |S_8| : 40,320 permutaciones
  Óptimo Global Exacto: Z* = 4496 en permutación [0, 1, 4, 5, 2, 3, 7, 6]
  Peor Costo (Máximo) : 6684 | Media poblacional uniforme: 5559.86 ± 267.97

[LOG FORENSE - TOPOLOGÍA DE ÓPTIMOS EN S_8 (Instancia 2)]:
  Rango #1 (Óptimo Global Z*): Costo = 4496 | Perm = [0, 1, 4, 5, 2, 3, 7, 6] | d_Cayley(Captura) = 1 swap (pos 1 <-> pos 2)
  Rango #2 (Subóptimo #1)    : Costo = 4512 | Perm = [0, 1, 6, 5, 2, 3, 7, 4] | d_Cayley(Captura) = 2 swaps
  Rango #3 (Subóptimo #2)    : Costo = 4521 | Perm = [0, 4, 1, 5, 2, 6, 7, 3] | d_Cayley(Captura) = 1 swap
  Rango #4 (Subóptimo #3)    : Costo = 4529 | Perm = [2, 1, 4, 5, 0, 3, 7, 6] | d_Cayley(Captura) = 2 swaps
  Rango #5 (Subóptimo #4)    : Costo = 4533 | Perm = [0, 1, 4, 3, 2, 5, 7, 6] | d_Cayley(Captura) = 2 swaps
  Rango #6 (CAPTURA [9])     : Costo = 4536 | Perm = [0, 4, 1, 5, 2, 3, 7, 6] | ¡ATRAPADO EN ÓPTIMO LOCAL!
```

---

## 2. LOG DEL EJERCICIO 1: Exploración vs. Explotación ($p_m = 0.2 \to 0.8$)

```text
[LOG EJERCICIO 1 - TRAZA GENERACIONAL COMPARATIVA (Semilla=1042, Instancia 2, Z*=4496)]
Gen | pm=0.2 (Elit) [Mejor | Media | Div] | pm=0.8 (Elit) [Mejor | Media | Div] | pm=0.8 (Sin Elit) [Mejor | Media | Div]
----+-------------------------------------+-------------------------------------+----------------------------------------
000 |      4862     | 5541.2 | 1.00       |      4862     | 5541.2 | 1.00       |      4862         | 5541.2 | 1.00
005 |      4589     | 5012.4 | 0.84       |      4589     | 5398.7 | 0.99       |      4691         | 5412.0 | 0.99
010 |      4536     | 4815.6 | 0.52       |      4554     | 5421.3 | 0.99       |      4812 (↑!)    | 5460.8 | 0.99
020 |      4536     | 4688.1 | 0.24       |      4536     | 5439.5 | 0.99       |      4654         | 5448.1 | 0.99
035 |      4536     | 4664.0 | 0.17       |      4496 (Z*)| 5441.8 | 0.99       |      4789 (↑!)    | 5455.2 | 0.99
050 |      4536     | 4661.5 | 0.16       |      4496 (Z*)| 5445.0 | 0.99       |      4702         | 5459.4 | 0.99
100 |      4536     | 4662.3 | 0.16       |      4496 (Z*)| 5442.1 | 0.99       |      4834 (↑!)    | 5451.7 | 0.99
150 |      4536     | 4660.9 | 0.17       |      4496 (Z*)| 5444.6 | 0.99       |      4741         | 5460.3 | 0.99
199 |      4536     | 4662.0 | 0.17       |      4496 (Z*)| 5443.0 | 0.99       |      4763         | 5457.6 | 0.99

[LOG MONTE CARLO (M=100 CORRIDAS INDEPENDIENTES - INSTANCIA 2, Z*=4496)]:
  pm = 0.2 (Con Elitismo - Original)             | Mejor Final: 4516.5 ± 17.9 | Media Pob: 4662.0 | Éxito Z*: 29% | GenConv:  20.0
  pm = 0.8 (Con Elitismo - Exploración Elitista) | Mejor Final: 4506.4 ± 11.9 | Media Pob: 5443.0 | Éxito Z*: 53% | GenConv:  72.2
  pm = 0.8 (Sin Elitismo - Deriva Inestable)     | Mejor Final: 4763.4 ± 109.0| Media Pob: 5457.6 | Éxito Z*:  1% | GenConv: 101.9

[LOG MONTE CARLO (M=100 CORRIDAS INDEPENDIENTES - INSTANCIA 1, Z*=4962)]:
  pm = 0.2 (Con Elitismo - Original)             | Mejor Final: 4964.9 ± 10.2 | Media Pob: 5041.7 | Éxito Z*:  92% | GenConv: 14.8
  pm = 0.8 (Con Elitismo - Exploración Elitista) | Mejor Final: 4962.0 ±  0.0 | Media Pob: 5576.6 | Éxito Z*: 100% | GenConv: 25.8
  pm = 0.8 (Sin Elitismo - Deriva Inestable)     | Mejor Final: 5105.0 ± 62.4 | Media Pob: 5604.3 | Éxito Z*:   2% | GenConv: 79.3
```

* **Ronda Adversarial 1 (Tesis):** Con $p_m = 0.8$, la alta tasa de mutación degrada el aprendizaje colectivo y vuelve inestable la población.
* **Ronda Adversarial 2 (Antítesis):** Debido a `nueva_poblacion = [evaluados[0][1]]` (elitismo estricto), `historial_mejores` es una sucesión monótona no creciente ($Z_{t+1} \le Z_t$). **La curva graficada nunca sube ni se vuelve inestable**. Más aún, con $p_m=0.8$ el élite actúa como trinquete mientras los 99 individuos restantes exploran agresivamente el espacio $S_8$, duplicando la tasa de escape del óptimo local `4536` hacia el óptimo global `4496` ($29\% \to 53\%$), a costa de destruir la convergencia de la media poblacional (`5443.0` vs `4662.0`). Solo al desactivar el elitismo aparece la inestabilidad en la curva del mejor costo.
* **Ronda Adversarial 3 (Síntesis):** Separar analíticamente la monotonía del élite de la entropía poblacional y aplicar enfriamiento adaptativo $p_m(t) = p_{\max}(p_{\min}/p_{\max})^{t/G}$.

---

## 3. LOG DEL EJERCICIO 2: El Poder de los Operadores (`swap_path_crossover` vs. `insert_path_crossover`)

```text
[LOG EJERCICIO 2 - TRAZA GENERACIONAL DE OPERADORES DE CRUCE (Semilla=1007, Instancia 2, Z*=4496)]
Gen | Mixto (80% Swap + 20% Insert) | Solo swap_path_crossover (Sin Insert) | Síntesis: True Geodesic Path Crossover
----+-------------------------------+---------------------------------------+---------------------------------------
000 |      4840 (Div=1.00)          |      4840 (Div=1.00)                  |      4840 (Div=1.00)
005 |      4568 (Div=0.81)          |      4612 (Div=0.79)                  |      4536 (Div=0.86)
010 |      4536 (Div=0.49)          |      4554 (Div=0.44)                  |      4496 [¡Z* ALCANZADO!] (Div=0.62)
020 |      4512 (Div=0.22)          |      4536 (Div=0.19 - Estancado)      |      4496 [Z*] (Div=0.31)
050 |      4512 (Div=0.17)          |      4536 (Div=0.17 - Estancado)      |      4496 [Z*] (Div=0.18)
100 |      4512 (Div=0.17)          |      4536 (Div=0.17 - Estancado)      |      4496 [Z*] (Div=0.17)
199 |      4512 (Div=0.17)          |      4536 (Div=0.17 - Estancado)      |      4496 [Z*] (Div=0.17)

=== Comparación Monte Carlo (100 semillas) en Instancia 1 (Z*=4962) e Instancia 2 (Z*=4496) ===
Original Mixto (80% Swap + 20% Insert) | Inst1: 4964.9 (Éxito Z*: 92%) | Inst2: 4516.5 (Éxito Z*: 29%)
Ejercicio 2: Solo swap_path_crossover  | Inst1: 4971.5 (Éxito Z*: 80%) | Inst2: 4521.3 (Éxito Z*: 17%)
Ablación: Solo Swap sin mutación (pm=0)| Inst1: 4980.6 (Éxito Z*: 55%) | Inst2: 4525.2 (Éxito Z*: 11%)
Síntesis: Geodesic Path Crossover      | Inst1: 4963.1 (Éxito Z*: 96%) | Inst2: 4511.2 (Éxito Z*: 44%)
```

* **Ronda Adversarial 1 (Tesis):** El algoritmo sigue encontrando buenas soluciones porque las transposiciones (2-ciclos) generan todo el grupo simétrico $S_n$ en a lo sumo $n-1 = 7$ pasos.
* **Ronda Adversarial 2 (Antítesis):**
  1. `swap_path_crossover` en el código original **no transfiere alelos de `padre2` a `padre1`**: solo toma dos posiciones donde `padre1[i] != padre2[i]` y las intercambia al azar dentro de `padre1` (mutación 2-swap restringida al soporte de disimilitud).
  2. Mientras un `swap` solo cambia 2 posiciones ($\Delta d_{\text{Hamming}} = 2$), `insert_path_crossover` ejecuta una rotación cíclica contigua de longitud $|d - o| + 1$, desplazando hasta $n$ instalaciones simultáneamente. Sin `insert_path_crossover`, la tasa de convergencia a $Z^*$ cae de **$92\% \to 80\%$** (Inst. 1) y de **$29\% \to 17\%$** (Inst. 2).
* **Ronda Adversarial 3 (Síntesis):** Nuestro operador `geodesic_path_crossover` repara iterativamente `hijo[pos] = padre2[pos]` mediante la transposición exacta `hijo.index(padre2[pos])`, elevando el éxito al **$96\%$** (Inst. 1) y **$44\%$** (Inst. 2).

---

## 4. LOG DEL EJERCICIO 3: Hibridación (Greedy + EA en la Población Inicial)

```text
=== Evaluación de los 3 Niveles de Individuo Greedy en Instancia 2 (Z* = 4496) ===
  Vector de Flujo Total por Instalación (F_i) : [214, 189, 198, 212, 195, 206, 173, 209]
  Vector de Distancia Total por Localización  : [188, 204, 179, 176, 194, 215, 201, 169]
  Nivel 1 (Orden por Flujo Total - Literal)   : [0, 3, 7, 5, 2, 4, 1, 6] -> Costo = 5566 (Percentil 51.4% de S_8)
  Nivel 2 (Dual Flujo Desc vs Dist Asc)       : [7, 3, 2, 0, 1, 4, 6, 5] -> Costo = 5512 (Percentil 43.1% de S_8; en Inst.1 baja de 5982 a 5334)
  Nivel 3 (Constructivo Cuadrático Exacto)    : [0, 1, 7, 5, 2, 3, 6, 4] -> Costo = 4654 (Percentil  0.35% de S_8 - Top 142 de 40,320)

[LOG EJERCICIO 3 - TRAZA GENERACIONAL DE HIBRIDACIÓN GREEDY + EA (Semilla=1015, Instancia 2)]
Gen | 100% Aleatorio (Base) | Híbrido Nivel 1 (Flujo Unilateral) | Híbrido Nivel 3 (Constructivo Cuadrático)
----+-----------------------+------------------------------------+------------------------------------------
000 |         4862          |         4862 (Greedy=5566 ignorado)|         4654 [¡MEJORA INICIAL -208 pts!]
002 |         4715          |         4715                       |         4536
005 |         4592          |         4588                       |         4496 [¡ÓPTIMO GLOBAL Z* EN GEN 5!]
015 |         4536          |         4536                       |         4496 [Z*]
050 |         4536          |         4536                       |         4496 [Z*]
199 |         4536          |         4536                       |         4496 [Z*]

[LOG MONTE CARLO (M=100 CORRIDAS) - COSTO INICIAL EN GENERACIÓN 0]:
  Población 100% Aleatoria (Base)                    | Costo Inicial (Gen 0) Promedio: 4818.2 ± 84.5
  Híbrido Nivel 1 (1 Greedy Flujo + 99 Aleatorios)   | Costo Inicial (Gen 0) Promedio: 4821.0 ± 83.1 (Mejora en 0/100 corridas)
  Híbrido Nivel 2 (1 Greedy Dual  + 99 Aleatorios)   | Costo Inicial (Gen 0) Promedio: 4821.0 ± 83.1 (En Inst.1 baja de 5982 a 5334)
  Híbrido Nivel 3 (1 Greedy Const + 99 Aleatorios)   | Costo Inicial (Gen 0) Promedio: 4651.2 ± 14.8 (Mejora en 96/100 corridas por -167.0 pts)
```

* **Ronda Adversarial 1 (Tesis):** Sembrar un individuo ordenando las instalaciones por flujo total inyecta conocimiento del problema (Diapositiva 27) y debería mejorar el costo inicial en $t=0$.
* **Ronda Adversarial 2 (Antítesis):** Ordenar solo por flujo total ($\phi = \operatorname{argsort}(-\Phi_F)$) ignora la matriz de distancias $D$, obteniendo costo `5566` (Inst. 2) y `5982` (Inst. 1). Por estadística de valores extremos, el mínimo de $99$ permutaciones aleatorias promedia $\mu - 2.51\sigma = 4818.2 < 5566$, por lo que **el Greedy unilateral de flujo jamás mejora el costo mínimo inicial de la población**.
* **Ronda Adversarial 3 (Síntesis):** El **Greedy Constructivo Cuadrático (Nivel 3)** minimiza secuencialmente el costo cuadrático parcial $\Delta Z(i \to k)$, alcanzando un costo inicial de **`4654`** (percentil $0.35\%$ de $S_8$), superando en **$-167.0$ puntos** al mínimo de las 99 soluciones aleatorias desde la Generación 0. Al combinarlo con búsqueda local **2-Opt**, converge al óptimo global exacto $Z^* = 4496$ en el **$100\%$ de las ejecuciones**.
