# NSGA-II Multi-Objective & Modal Evolutionary Control of a 3-DOF Spatial Robotic Manipulator in MuJoCo

**Universidad Nacional de Ingeniería (UNI)** — Facultad de Ingeniería Industrial y de Sistemas — Unidad de Posgrado (Maestría en Inteligencia Artificial)  
**Asignatura:** MAE-IA: Computación Evolutiva (2026-2) | **Docente:** Dr. Yván Jesús Túpac Valdivia  
**Autores (Grupo 2):** Amit Roy Flores Rivera, Franklin Enrique Alegría Barboza, José Luis Espinoza Chamaya, Raquel Medianero Huari, Yasmín Mariela Urrego Montoya

![Gemelo Digital 3D en MuJoCo del Brazo Robótico de 3 GDL](Computacion%20Evolutiva/figuras_ce/fig_estados_3d_enceste.png)

---

## 📄 Entregables Oficiales del Trabajo Final (Acceso Directo)

* 📘 **Manuscrito Final Completo en PDF (Versión Definitiva 20/20 con Protocolo de 30 Semillas y Test de Wilcoxon-Holm):**  
  👉 [**`MAE-IA_Trabajo_Final_Computacion_Evolutiva_Grupo2_v2_20de20.pdf`**](MAE-IA_Trabajo_Final_Computacion_Evolutiva_Grupo2_v2_20de20.pdf)
* 📑 **Código Fuente LaTeX del Manuscrito:** [`MAE-IA_Trabajo_Final_Computacion_Evolutiva_Grupo2.tex`](MAE-IA_Trabajo_Final_Computacion_Evolutiva_Grupo2.tex)
* 📊 **Informe Técnico en Markdown:** [`INFORME_FINAL_COMPUTACION_EVOLUTIVA_ROBOT_3DOF.md`](INFORME_FINAL_COMPUTACION_EVOLUTIVA_ROBOT_3DOF.md)
* 🧬 **Suite Experimental Base (Modo Tesis 480 evals + Sensibilidad $S_1\dots S_9$ + Benchmark 7 Paradigmas):** [`Computacion Evolutiva/ejecutar_estudio_completo_ce.py`](Computacion%20Evolutiva/ejecutar_estudio_completo_ce.py)
* 🛡️ **Suite de Confrontación en Gemelo MuJoCo 3D Hostil ($N=30$ Semillas + Reparametrización Modal Tri-Objetivo + Gain-Scheduling + Sustituto RBF + Wilcoxon-Holm):** [`Computacion Evolutiva/Mejoras/confrontacion_definitiva_mujoco_30semillas.py`](Computacion%20Evolutiva/Mejoras/confrontacion_definitiva_mujoco_30semillas.py)

---

## 🖥️ ¿Cómo presentar el Simulador 3D MuJoCo en vivo ante el Profesor?

El repositorio incluye un **Laboratorio Interactivo 3D en tiempo real (`main.py`)** acoplado al motor físico **MuJoCo** (`500 Hz` física / `100 Hz` control).

### Opción 1: Doble clic (Windows)
Haz doble clic sobre el archivo:
```bat
LANZAR_SIMULADOR_MUJOCO_NSGA2.bat
```

### Opción 2: Desde terminal PowerShell / CMD
```powershell
.venv\Scripts\python.exe main.py --controller EVO
```

### Guion de Demostración en Vivo (3 Minutos para la Exposición):
1. **Panel Matemático en Vivo (Izquierda) y Gemelo 3D (Centro):** Al abrirse la ventana, el controlador activo ya es **`CTC NSGA-II`** con el cromosoma Pareto-óptimo cargado automáticamente. Muestra cómo se actualizan en tiempo real la cinemática directa $p(q)$, el Jacobiano $J(q)$, la manipulabilidad de Yoshikawa $w(q)$ y los pares $\tau(t)$.
2. **Prueba 1 — Seguimiento Sub-Milimétrico (`Mover al objetivo`):** Haz clic en **`Mover al objetivo`**. El brazo ejecuta la trayectoria quíntica $C^2$ y al ingresar en la esfera de tolerancia ($< 2.0\text{ mm}$) el efector y el aro cambian a **Verde Esmeralda (`ENCESTADO`)**, logrando un RMSE de **$4.573\text{ mm}$** ($-33.5\%$ respecto al CTC Base de $6.879\text{ mm}$) con **$0.0\%$ de saturación**.
3. **Prueba 2 — Rechazo de Perturbación Externa (`Perturbación`):** Haz clic en **`Perturbación`**. Se inyecta un pulso tridimensional de torque externo $\tau_{\text{ext}} = [0.035, -0.040, 0.025]^T\text{ N}\cdot\text{m}$; el efector cambia a **Magenta (`PERTURBACIÓN ACTIVA`)** y las ganancias evolucionadas recuperan el brazo en **$0.382\text{ s}$** ($-34.4\%$ más rápido que el diseño clásico).
4. **Prueba 3 — Modo Juego Autónomo (`Circuito 5 Aros`):** Selecciona la velocidad (`Estándar 4.5s`, `Rápido 2.5s` o `Turbo 1.5s`) y pulsa **`Circuito 5 Aros (Juego)`** para demostrar que el controlador evolucionado generaliza en todo el espacio de trabajo logrando **$5/5$ encestes consecutivos**.

---

## 📈 Resultados Principales de Computación Evolutiva

### 1. Frente de Pareto Bi-Objetivo (Modo Tesis: $N_{\text{pop}}=24, G=20$, 1,920 Simulaciones MuJoCo)
![Frente de Pareto NSGA-II](Computacion%20Evolutiva/figuras_ce/fig_ce_01_frente_pareto.png)

### 2. Dinámica de Convergencia Generacional e Hipervolumen Dominado $HV(g)$
![Convergencia Generacional](Computacion%20Evolutiva/figuras_ce/fig_ce_02_convergencia_generacional.png)

### 3. Estudio de Sensibilidad ($S_1 \dots S_9$) y Benchmark contra DE, CMA-ES, Nelder-Mead y Monte Carlo
![Sensibilidad y Comparación](Computacion%20Evolutiva/figuras_ce/fig_ce_03_sensibilidad_y_comparacion.png)

### 4. Confrontación en Gemelo Digital 3D MuJoCo Hostil ($N=30$ Semillas Independientes, Ruido de Encoder $\sigma_q=0.002\text{ rad}$, Retardo $10\text{ ms}$, Reparametrización Modal Tri-Objetivo y Test de Wilcoxon-Holm)
![Confrontación 30 Semillas en MuJoCo Hostil](Computacion%20Evolutiva/figuras_ce/fig_ce_05_confrontacion_hostil_30semillas.png)

| Controlador Evaluado ($N = 30$ Semillas CRN) | $f_1^{\text{rob}}$ Mediana (IQR) [mm] ($\downarrow$) | $f_2^{\text{rob}}$ Mediana (IQR) [$\text{N}^2\text{m}^2\text{s}$] ($\downarrow$) | $f_3^{\text{rob}}$ Chattering Med. (IQR) [$\text{N}^2\text{m}^2/\text{s}$] ($\downarrow$) | Rango Friedman $f_1$ | Rango Friedman $f_3$ | $p_{\text{Holm}}$ ($f_1$) vs. C1 Base | $p_{\text{Holm}}$ ($f_3$) vs. C2 Ideal |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **C1: CTC Base Analítico** ($\omega_n=13, \zeta=1.10$) | $13.235\ (0.324)$ | $0.4422\ (0.0040)$ | $2.247\ (0.258)$ | $5.00$ | $2.33$ | — | $7.45 \times 10^{-9}$ |
| **C2: $\theta^*_{\text{ideal}}$ Tabla 2** ($\zeta_3=0.133$, sin ruido) | **$9.713\ (0.165)$** | **$0.4404\ (0.0038)$** | $3.684\ (0.499)$ | **$1.00$** | $5.00$ | $7.45 \times 10^{-9}$ | — |
| **C3: NSGA-II Directo Hostil** ($[K_p,K_d] \in \mathbb{R}^6$) | $9.916\ (0.267)$ | $0.4406\ (0.0040)$ | $3.334\ (0.468)$ | $2.00$ | $4.00$ | $7.45 \times 10^{-9}$ | $7.45 \times 10^{-9}$ |
| **C4: NSGA-II Modal Tri-Obj.** ($[\omega_n,\zeta], \zeta \in [0.81, 1.28]$) | $11.302\ (0.330)$ | $0.4415\ (0.0041)$ | $2.337\ (0.315)$ | $3.50$ | $2.67$ | $7.45 \times 10^{-9}$ | $7.45 \times 10^{-9}$ |
| **C5: NSGA-II Gain-Sched + RBF + Memético** ($D=12$) | $11.364\ (0.172)$ | $0.4409\ (0.0040)$ | **$1.922\ (0.289)$** | $3.50$ | **$1.00$** | $7.45 \times 10^{-9}$ | $7.45 \times 10^{-9}$ |

* **Test Omnibus de Friedman ($N=30, k=5$):** Precisión $f_1$: $\chi^2_F = 114.00, p = 1.02 \times 10^{-23}$. Chattering $f_3$: $\chi^2_F = 114.96, p = 6.36 \times 10^{-24}$.
