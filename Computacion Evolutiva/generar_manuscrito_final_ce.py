"""
Generador perfeccionado (exit code 0 en pdflatex + tablas autoajustadas + Word .docx)
del Manuscrito Final para MAE-IA: Computación Evolutiva 2026-2 (UNI).
"""
import shutil
import subprocess
import zipfile
from pathlib import Path

CE_DIR = Path(__file__).resolve().parent
WORKSPACE_GRUPO2 = Path(r"c:\Users\yurrego\OneDrive - EFICITEC S.A.C\Documentos\COMPUTACION EVOLUTIVA\PrimerTrabajoGrupo2")

latex_content = r"""\documentclass[11pt,a4paper]{article}
\usepackage[utf8]{inputenc}
\usepackage[T1]{fontenc}
\usepackage[spanish,es-tabla,es-nodecimaldot]{babel}
\usepackage{geometry}
\geometry{top=2.2cm, bottom=2.2cm, left=2.2cm, right=2.2cm, headheight=14pt}
\usepackage{amsmath,amssymb,amsfonts}
\usepackage{graphicx}
\usepackage{booktabs}
\usepackage{multirow}
\usepackage{array}
\usepackage{float}
\usepackage{xcolor}
\usepackage{hyperref}
\usepackage{fancyhdr}
\usepackage{caption}

\definecolor{uniblue}{RGB}{13, 71, 161}
\definecolor{unigray}{RGB}{55, 71, 79}

\hypersetup{
    colorlinks=true,
    linkcolor=uniblue,
    citecolor=uniblue,
    urlcolor=uniblue
}

\pagestyle{fancy}
\fancyhf{}
\fancyhead[L]{\footnotesize\textcolor{unigray}{\textbf{UNI — Posgrado FIIS | Computación Evolutiva 2026-2}}}
\fancyhead[R]{\footnotesize\textcolor{unigray}{\textbf{Control Robótico 3-DOF con NSGA-II}}}
\fancyfoot[C]{\small\thepage}

\begin{document}

% ==============================================================================
% PORTADA OFICIAL
% ==============================================================================
\begin{titlepage}
    \centering
    \vspace*{0.2cm}
    \includegraphics[width=3.1cm]{figuras_ce/logo_uni.png}\\[0.35cm]
    {\Large \textbf{UNIVERSIDAD NACIONAL DE INGENIERÍA}}\\[0.15cm]
    {\large FACULTAD DE INGENIERÍA INDUSTRIAL Y DE SISTEMAS}\\[0.1cm]
    {\large UNIDAD DE POSGRADO — MAESTRÍA EN INTELIGENCIA ARTIFICIAL}\\[0.55cm]
    
    \rule{\linewidth}{0.6mm}\\[0.35cm]
    {\LARGE \bfseries \textcolor{uniblue}{TRABAJO FINAL DE COMPUTACIÓN EVOLUTIVA:\\[0.15cm]
    Modelado, Implementación y Análisis de una Estrategia Evolutiva Multiobjetivo Elitista (NSGA-II) para la Sintonización Robusta de un Controlador por Torque Computado en un Manipulador Robótico Espacial de 3 GDL}}\\[0.25cm]
    \rule{\linewidth}{0.6mm}\\[0.45cm]
    
    {\large \textbf{Asignatura:} MAE-IA: Computación Evolutiva (2026-2)}\\[0.15cm]
    {\large \textbf{Docente:} Dr. Yván Jesús Túpac Valdivia}\\[0.45cm]
    
    {\large \textbf{Autores — Grupo 2:}}\\[0.2cm]
    \begin{tabular}{c}
        \textbf{Amit Roy Flores Rivera} \\
        \textbf{Franklin Enrique Alegría Barboza} \\
        \textbf{José Luis Espinoza Chamaya} \\
        \textbf{Raquel Medianero Huari} \\
        \textbf{Yasmín Mariela Urrego Montoya}
    \end{tabular}\\[0.55cm]
    
    \includegraphics[width=0.92\linewidth]{figuras_ce/fig_estados_3d_enceste.png}\\[0.2cm]
    {\footnotesize \textit{Gemelo Digital 3D en MuJoCo del Manipulador RRR de 3 GDL gobernado por el controlador CTC evolucionado con NSGA-II.}}\\[0.4cm]
    
    \vfill
    {\large Lima, Perú — Noviembre 2026}
\end{titlepage}

\tableofcontents
\newpage

% ==============================================================================
% RESUMEN EJECUTIVO
% ==============================================================================
\begin{abstract}
\noindent Este manuscrito presenta el desarrollo integral de las cuatro fases del Trabajo Final del curso de \textit{Computación Evolutiva} (Maestría en Inteligencia Artificial — UNI), aplicando técnicas evolutivas multiobjetivo y de optimización continua al problema de ingeniería de \textbf{sintonización óptima robusta de ganancias de un Controlador por Torque Computado (CTC)} en un brazo robótico espacial antropomórfico de 3 Grados de Libertad (configuración RRR accionada por servomotores Dynamixel AX-18A y AX-12A). El problema se formula como una optimización bi-objetivo no lineal con restricciones de saturación de torque y estabilidad dinámica sobre un Gemelo Digital físico 3D en \textbf{MuJoCo}, buscando minimizar simultáneamente el error cuadrático medio de seguimiento cartesiano ($f_1$: $\text{RMSE}_{\text{seg}}$ en mm) y la energía integral de actuación ($f_2$: $\int_0^T \|\tau(t)\|_2^2\,dt$ en $\text{N}^2\text{m}^2\text{s}$) promediados bajo cuatro regímenes operacionales (nominal, perturbación externa impulsiva, incertidumbre paramétrica de masa $+25\%/+15\%$ y amortiguamiento $+30\%$, y régimen combinado). Se selecciona, justifica e implementa el algoritmo \textbf{NSGA-II} (\textit{Non-dominated Sorting Genetic Algorithm II}) con codificación real $\theta \in \mathbb{R}^6$, cruce binario simulado (SBX) y mutación polinomial (PM), complementado con una regla de decisión post-Pareto basada en la mínima distancia euclidiana normalizada al punto ideal utópico dentro del cono de dominancia del controlador analítico base. Asimismo, se ejecuta un estudio de sensibilidad paramétrica de 9 configuraciones ($S_1$ a $S_9$) y un benchmark cruzado contra seis métodos adicionales: Asignación Analítica de Polos (CTC Base), Búsqueda Aleatoria Uniforme (Monte Carlo), Nelder-Mead Simplex, Evolución Diferencial (\texttt{DE/rand/1/bin}), Estrategia Evolutiva de Adaptación de Matriz de Covarianza (\textbf{CMA-ES}) y esquemas de control avanzado (MPC y CLIK-DLS). Los resultados experimentales demuestran que NSGA-II domina estrictamente al controlador analítico base en ambos objetivos en conflicto, reduciendo el $\text{RMSE}$ robusto en un \textbf{31.06\%} (de $11.353\text{ mm}$ a $7.827\text{ mm}$ en evaluación multi-escenario de 4 casos, y de $6.879\text{ mm}$ a $4.573\text{ mm}$ [$-33.5\%$] en seguimiento nominal), acortando el tiempo de captura sub-milimétrica ($<2.0\text{ mm}$) de $5.84\text{ s}$ a $4.50\text{ s}$ y reduciendo simultáneamente el consumo energético de los servomotores sin incurrir en saturación ($0.0\%$).
\end{abstract}

% ==============================================================================
% FASE I: DEFINICIÓN Y MODELADO DEL PROBLEMA
% ==============================================================================
\section{Fase I: Definición y Modelado del Problema de Optimización}

\subsection{Problema Elegido y Relevancia en Ingeniería Robótica}
El control de precisión de manipuladores robóticos antropomórficos de múltiples grados de libertad constituye un problema altamente no lineal, acoplado y multivariable en la industria manufacturera, instrumentación quirúrgica y sistemas autónomos. La dinámica de cuerpo rígido de un manipulador espacial de $n = 3$ grados de libertad rotacionales (configuración antropomórfica RRR) está gobernada por la ecuación diferencial vectorial de Euler-Lagrange:
\begin{equation}
    M(q)\ddot{q} + C(q, \dot{q})\dot{q} + g(q) + f_v\dot{q} + \tau_f(\dot{q}) = \tau + \tau_{\text{ext}}(t)
    \label{eq:lagrange}
\end{equation}
donde $q, \dot{q}, \ddot{q} \in \mathbb{R}^3$ representan los vectores de posición, velocidad y aceleración articular para la cintura ($q_1$), hombro ($q_2$) y codo ($q_3$); $M(q) \in \mathbb{R}^{3\times 3}$ es la matriz de inercia simétrica definida positiva; $C(q,\dot{q})\dot{q} \in \mathbb{R}^3$ agrupa las fuerzas centrífugas y de Coriolis; $g(q) \in \mathbb{R}^3$ es el vector de pares gravitacionales; $f_v\dot{q} + \tau_f(\dot{q})$ modela la fricción viscosa y seca de Coulomb; $\tau \in \mathbb{R}^3$ es el vector de torques aplicados por los actuadores; y $\tau_{\text{ext}}(t) \in \mathbb{R}^3$ representa perturbaciones externas desconocidas.

En la arquitectura clásica de \textbf{Control por Torque Computado (CTC)}, se emplea un modelo interno nominal $(\hat{M}(q), \hat{h}(q,\dot{q}))$ para cancelar las no linealidades de la planta mediante realimentación desacoplante:
\begin{equation}
    \tau_{\text{cmd}}(t) = \hat{M}(q)\Big(\ddot{q}_r(t) + K_p\big(q_r(t) - q(t)\big) + K_d\big(\dot{q}_r(t) - \dot{q}(t)\big)\Big) + \hat{h}(q, \dot{q})
    \label{eq:ctc}
\end{equation}
donde $\hat{h}(q,\dot{q}) = \hat{C}(q,\dot{q})\dot{q} + \hat{g}(q) + \hat{f}_v\dot{q}$ es el vector de fuerzas de sesgo (\textit{bias}) y $K_p = \operatorname{diag}(K_{p1}, K_{p2}, K_{p3})$, $K_d = \operatorname{diag}(K_{d1}, K_{d2}, K_{d3})$ son las matrices de ganancia proporcional y derivativa.

\textbf{¿Por qué falla el diseño analítico clásico y por qué se requiere Computación Evolutiva?}
En la teoría lineal ideal (cuando $\hat{M} = M$, $\hat{h} = h$, $\tau_{\text{ext}} = 0$ y no existe saturación de actuadores), el error articular $e(t) = q_r(t) - q(t)$ satisface $\ddot{e} + K_d\dot{e} + K_p e = 0$, lo que lleva a la práctica común de elegir ganancias idénticas para todas las articulaciones mediante asignación de polos (en nuestro caso base: $\omega_n = 13\text{ rad/s} \implies K_{p,i} = \omega_n^2 = 169.0\text{ s}^{-2}$ y $\zeta = 1.1 \implies K_{d,i} = 2\zeta\omega_n = 28.6\text{ s}^{-1}$). Sin embargo, en el sistema físico real basado en el prototipo de Conejo Benítez (2021) ocurren cuatro fenómenos no lineales severos que invalidan la sintonización analítica uniforme:
\begin{enumerate}
    \item \textbf{Asimetría inercial y gravitacional extrema entre eslabones:} La cintura ($q_1$, servomotor Dynamixel AX-18A, $m_1 = 0.250\text{ kg}$) soporta la inercia rotacional de toda la cadena cinemática; el hombro ($q_2$, AX-12A, $L_1 = 0.113\text{ m}$, $m_2 = 0.120\text{ kg}$) soporta el par gravitacional máximo en voladizo; mientras que el codo ($q_3$, AX-12A, $L_2 = 0.140\text{ m}$, $m_3 = 0.100\text{ kg}$) posee una inercia diez veces menor y un límite de par más estricto ($\tau_{\max,3} = 0.45\text{ N}\cdot\text{m}$ frente a $0.70\text{ N}\cdot\text{m}$ en $q_1, q_2$).
    \item \textbf{Saturación dura no diferenciable de los actuadores:} El torque real aplicado a la planta está truncado por $\tau_i(t) = \operatorname{clip}(\tau_{\text{cmd},i}(t), -\tau_{\max,i}, +\tau_{\max,i})$.
    \item \textbf{Discrepancia planta-modelo (Incertidumbre Paramétrica):} Cuando la masa de la carga útil o de los eslabones varía ($+25\%$ en eslabón 1, $+15\%$ en eslabón 2 y $+30\%$ en amortiguamiento viscoso), $\hat{M}(q) \neq M(q)$ y la cancelación exacta se rompe, haciendo que el paisaje de rendimiento dependa de integrales de trayectoria en lazo cerrado sin forma analítica cerrada.
    \item \textbf{Conflicto multiobjetivo intrínseco:} Incrementar $K_p$ y $K_d$ reduce el error de seguimiento cartesiano ($\text{RMSE}$), pero incrementa cuadráticamente la energía de control $\int \|\tau(t)\|_2^2\,dt$, el calentamiento Joule de los servomotores y el riesgo de saturación.
\end{enumerate}

\subsection{Formulación Matemática como Problema de Optimización Multiobjetivo}

\paragraph{1. Vector de Variables de Decisión (Cromosoma $\theta$):}
Se define el vector continuo de $D = 6$ variables de decisión reales positivas correspondientes a las ganancias articulares desacopladas del controlador CTC:
\begin{equation}
    \theta = \begin{bmatrix} K_{p1} & K_{p2} & K_{p3} & K_{d1} & K_{d2} & K_{d3} \end{bmatrix}^T \in \Omega \subset \mathbb{R}^6
\end{equation}
donde el espacio de búsqueda factible $\Omega$ es el hiperrectángulo acotado:
\begin{equation}
    \Omega = [20.0,\, 250.0]^3 \times [3.0,\, 55.0]^3 \subset \mathbb{R}^6
\end{equation}

\paragraph{2. Funciones Objetivo en Conflicto ($\mathbf{F}(\theta)$):}
Sea $\mathcal{S} = \{s_1, s_2, s_3, s_4\}$ el conjunto de cuatro escenarios de simulación dinámica de duración $T = 9.0\text{ s}$ sobre el Gemelo Digital 3D:
\begin{itemize}
    \item $s_1$ (\textbf{Nominal}): Planta nominal sin perturbación externa ($\tau_{\text{ext}} = 0$).
    \item $s_2$ (\textbf{Perturbación Externa}): Pulso de par perturbador tridimensional $\tau_{\text{ext}}(t) = [0.035,\, -0.040,\, 0.025]^T\text{ N}\cdot\text{m}$ inyectado durante $t \in [6.0,\, 6.6]\text{ s}$.
    \item $s_3$ (\textbf{Incertidumbre Paramétrica}): Planta física alterada con $+25\%$ de masa en el eslabón 1, $+15\%$ de masa en el eslabón 2 y $+30\%$ de fricción viscosa articular, manteniendo el controlador CTC con la matriz nominal $\hat{M}(q)$.
    \item $s_4$ (\textbf{Combinado Robusto}): Incertidumbre paramétrica simultánea con el pulso de perturbación externa.
\end{itemize}

Se formula el problema de minimización vectorial bi-objetivo:
\begin{equation}
    \min_{\theta \in \Omega} \mathbf{F}(\theta) = \begin{bmatrix} f_1(\theta) \\[0.15cm] f_2(\theta) \end{bmatrix} = \begin{bmatrix} \displaystyle \frac{1}{|\mathcal{S}|} \sum_{s \in \mathcal{S}} \sqrt{\frac{1}{N_m} \sum_{k=1}^{N_m} \big\| p^{(s)}(t_k; \theta) - p_{\text{ref}}(t_k) \big\|_2^2} \times 10^3 \\[0.35cm] \displaystyle \frac{1}{|\mathcal{S}|} \sum_{s \in \mathcal{S}} \int_{0}^{T} \big\| \tau^{(s)}(t; \theta) \big\|_2^2 \, dt \end{bmatrix}
    \label{eq:biobjetivo}
\end{equation}
donde:
\begin{itemize}
    \item $f_1(\theta)$ es el \textbf{Error Cuadrático Medio de Seguimiento Cartesiano Robusto ($\text{RMSE}_{\text{seg}}$)} expresado en milímetros $[\text{mm}]$, calculado entre la posición real de la punta del efector en MuJoCo $p^{(s)}(t_k;\theta) \in \mathbb{R}^3$ y la trayectoria de referencia $p_{\text{ref}}(t_k) = \operatorname{FK}(q_r(t_k))$ generada mediante un polinomio quíntico suave de clase $C^2$.
    \item $f_2(\theta)$ es el \textbf{Esfuerzo Integral Cuadrático de Torque Robusto} expresado en $[\text{N}^2\text{m}^2\text{s}]$, integrado numéricamente mediante la regla trapezoidal sobre las $N_m = 450$ muestras ($50\text{ Hz}$) de cada episodio de $T = 9.0\text{ s}$.
\end{itemize}

\paragraph{3. Restricciones Físicas y Operacionales:}
\begin{align}
    g_1(\theta): &\quad 20.0 \le K_{p,i} \le 250.0, \quad \forall i \in \{1,2,3\} \quad \text{(Cotas de rigidez proporcional)} \\
    g_2(\theta): &\quad 3.0 \le K_{d,i} \le 55.0, \quad \forall i \in \{1,2,3\} \quad \text{(Cotas de amortiguamiento derivativo)} \\
    g_3(t): &\quad |\tau_i^{(s)}(t)| \le \tau_{\max,i}, \quad \tau_{\max} = [0.70,\, 0.70,\, 0.45]^T\text{ N}\cdot\text{m}, \quad \forall t \in [0, T] \\
    g_4(t): &\quad q_{\min,i} \le q_i^{(s)}(t) \le q_{\max,i}, \quad q_{\min}=[-2.80, -1.25, -2.40]^T,\ q_{\max}=[+2.80, +1.35, +2.40]^T\text{ rad} \\
    g_5(\theta): &\quad \mathbf{F}(\theta) \in \mathbb{R}^2 \text{ finito (si la integración diverge, se aplica penalización } \mathbf{F}(\theta) = [10^8,\, 10^8]^T\text{)}
\end{align}

\subsection{Modelo del Proceso a Optimizar (Gemelo Digital 3D en MuJoCo)}
En cumplimiento de la guía oficial del proyecto, el proceso a optimizar combina el \textbf{desarrollo completo del modelado cinemático y dinámico} del manipulador (\texttt{models/brazo\_3d.xml} y \texttt{robot\_ia/kinematics.py}) con el motor físico de cuerpo rígido articulado \textbf{MuJoCo} (\textit{Multi-Joint dynamics with Contact}, Todorov et al., 2012). El simulador integra las ecuaciones de movimiento con paso físico $\Delta t_{\text{sim}} = 2.0\text{ ms}$ ($500\text{ Hz}$), mientras que el lazo de control digital CTC se actualiza cada $\Delta t_{\text{ctrl}} = 10.0\text{ ms}$ ($100\text{ Hz}$). La arquitectura implementa dos instancias independientes en memoria: (1) la \textit{planta física real} (\texttt{model}, \texttt{data}), que sufre las perturbaciones externas y las variaciones de masa/amortiguamiento, y (2) el \textit{modelo interno del controlador} (\texttt{nominal}, \texttt{dn}), que calcula $\hat{M}(q)$ mediante el algoritmo de cuerpo rígido compuesto (\texttt{mj\_fullM}) y $\hat{h}(q,\dot{q})$ (\texttt{dn.qfrc\_bias}) sin conocer la incertidumbre inyectada en la planta.

% ==============================================================================
% FASE II: DISEÑO DEL ALGORITMO EVOLUTIVO
% ==============================================================================
\section{Fase II: Diseño del Algoritmo Evolutivo}

\subsection{Selección y Justificación del Paradigma Evolutivo}
Dado que el problema formulado en la Ecuación~(\ref{eq:biobjetivo}) posee un espacio de búsqueda continuo $\Omega \subset \mathbb{R}^6$, una función de evaluación de caja negra no diferenciable (debido al operador $\operatorname{clip}(\cdot)$ de saturación de torque y los contactos articulares en MuJoCo) y dos objetivos físicos en conflicto con escalas numéricas dispares ($f_1 \in [7.5, 70.0]\text{ mm}$ vs. $f_2 \in [0.44, 0.56]\text{ N}^2\text{m}^2\text{s}$), se seleccionó el paradigma de \textbf{Algoritmos Evolutivos Multiobjetivo de Codificación Real basados en Dominancia de Pareto}, específicamente el algoritmo \textbf{NSGA-II} (\textit{Non-dominated Sorting Genetic Algorithm II}, Deb et al., 2002).

La justificación técnica frente a otros paradigmas estudiados en el curso es triple:
\begin{enumerate}
    \item \textbf{Superioridad frente a la escalarización lineal a priori (AG Canónico Monobjetivo):} Un Algoritmo Genético simple requiere fijar pesos arbitrarios $w_1 f_1 + w_2 f_2$ antes de conocer la geometría del frente de Pareto, y es incapaz de descubrir soluciones en regiones no convexas del frente. NSGA-II obtiene en una sola corrida el frente de Pareto completo, permitiendo analizar la sensibilidad física entre precisión milimétrica y esfuerzo energético.
    \item \textbf{Adecuación de operadores reales (SBX + PM) frente a codificación binaria o de orden:} A diferencia de los problemas combinatorios discretos o de permutaciones (como TSP o QAP), las ganancias $(K_{p,i}, K_{d,i})$ pertenecen a un espacio métrico continuo $\mathbb{R}^6$ con fuerte epistasis entre $K_{p,i}$ y $K_{d,i}$ de la misma articulación (que determinan conjuntamente la razón de amortiguamiento $\zeta_i = K_{d,i} / (2\sqrt{K_{p,i}})$). El cruce binario simulado (SBX) preserva la distancia media entre padres e hijos y explota la topología continua de $\mathbb{R}^6$ sin sufrir acantilados de Hamming (\textit{Hamming cliffs}).
    \item \textbf{Elitismo $(\mu+\lambda)$ y preservación de diversidad sin parámetros adicionales:} NSGA-II garantiza que ningún individuo Pareto-óptimo hallado se pierda entre generaciones (propiedad crítica cuando cada evaluación cuesta 4 simulaciones dinámicas en MuJoCo) y mantiene una distribución uniforme sobre el frente mediante la distancia de apilamiento (\textit{Crowding Distance}).
\end{enumerate}

\subsection{Definición Detallada de Componentes del Algoritmo Evolutivo}

\begin{table}[H]
\centering
\caption{Resumen de la Arquitectura y Operadores del Algoritmo Evolutivo NSGA-II Propuesto.}
\label{tab:componentes_ce}
\resizebox{\linewidth}{!}{%
\begin{tabular}{p{3.6cm} p{4.4cm} p{7.0cm}}
\toprule
\textbf{Componente Evolutivo} & \textbf{Especificación Técnica} & \textbf{Justificación Matemática y de Dominio} \\
\midrule
\textbf{Representación (Genotipo)} & Vector de punto flotante de doble precisión $\theta \in \mathbb{R}^6$: $[K_{p1}, K_{p2}, K_{p3}, K_{d1}, K_{d2}, K_{d3}]^T$ & Evita cuantización artificial y permite ajuste fino continuo en $[20,250]^3 \times [3,55]^3$. \\
\addlinespace
\textbf{Población Inicial} & Muestreo uniforme aleatorio en $\Omega$ con eliminación de duplicados & Garantiza cobertura imparcial del hiperrectángulo de ganancias en $g = 1$. \\
\addlinespace
\textbf{Función de Aptitud} & Vector bi-objetivo crudo $\mathbf{F}(\theta) = [f_1(\theta), f_2(\theta)]^T$ sobre 4 escenarios & Invariante a reescalamientos monótonos gracias al ordenamiento por dominancia de Pareto. \\
\addlinespace
\textbf{Asignación de Fitness} & \textit{Fast Non-Dominated Sorting} ($O(M N^2)$) + \textit{Crowding Distance} & Clasifica en frentes $\mathcal{F}_1, \mathcal{F}_2, \dots$ y premia soluciones en regiones poco pobladas del frente. \\
\addlinespace
\textbf{Esquema de Selección} & Torneo Binario Aglomerado ($\prec_c$) & $\theta^{(a)}$ gana a $\theta^{(b)}$ si $r_a < r_b$, o si $r_a = r_b$ y $d_{\text{crowd}}(a) > d_{\text{crowd}}(b)$. \\
\addlinespace
\textbf{Operador de Cruce} & \textit{Simulated Binary Crossover} (SBX) acotado, $p_c = 0.90$, $\eta_c = 15$ & Genera hijos $c_{1,2} = \frac{1}{2}[(1\pm\beta_q)p_1 + (1\mp\beta_q)p_2]$ con poder de búsqueda auto-adaptativo a $|p_1 - p_2|$. \\
\addlinespace
\textbf{Operador de Mutación} & Mutación Polinomial (PM) acotada, $p_m = 1/6 \approx 0.167$, $\eta_m = 20$ & Modifica en promedio 1 gen por cromosoma con perturbación local $\delta_q (u_j - l_j)$ que respeta $\Omega$. \\
\addlinespace
\textbf{Esquema de Reemplazo} & Elitista $(\mu + \lambda)$ con $\mu = \lambda = N_{\text{pop}}$ y filtro de duplicados & Combina $R_g = P_g \cup Q_g$ ($2N_{\text{pop}}$) y retiene exactamente los mejores $N_{\text{pop}}$ individuos. \\
\addlinespace
\textbf{Selector Post-Pareto} & Mínima distancia euclidiana normalizada al ideal $z^*$ en región $\mathbf{F}(\theta) \preceq \mathbf{F}(\theta_{\text{base}})$ & Garantiza elegir automáticamente del frente una ganancia que domine simultáneamente al controlador base en ambos criterios. \\
\bottomrule
\end{tabular}%
}
\end{table}

\paragraph{Regla Matemática de Decisión Post-Pareto (Selección del Controlador Final):}
Una vez concluida la evolución en la generación $G$, se obtiene el frente de Pareto aproximado $\mathcal{F}_1 = \{\theta^{(1)}, \dots, \theta^{(K)}\}$. Para seleccionar de manera determinista y reproducible el vector de ganancias que se despliega en el robot, primero se filtra el subconjunto que mejora al controlador base en precisión:
\begin{equation}
    \mathcal{F}_{\text{dom}} = \big\{ \theta \in \mathcal{F}_1 \;\big|\; f_1(\theta) \le f_1(\theta_{\text{base}}) \big\}
\end{equation}
Sea $z_m^{\min} = \min_{\theta \in \mathcal{F}_{\text{dom}}} f_m(\theta)$ y $z_m^{\max} = \max_{\theta \in \mathcal{F}_{\text{dom}}} f_m(\theta)$ para $m \in \{1, 2\}$. Se selecciona el individuo compromiso $\theta^*$ que minimiza la distancia $L_2$ normalizada al punto ideal utópico $z^{\min} = [z_1^{\min}, z_2^{\min}]^T$:
\begin{equation}
    \theta^* = \arg\min_{\theta \in \mathcal{F}_{\text{dom}}} \sqrt{ \left( \frac{f_1(\theta) - z_1^{\min}}{z_1^{\max} - z_1^{\min}} \right)^2 + \left( \frac{f_2(\theta) - z_2^{\min}}{z_2^{\max} - z_2^{\min}} \right)^2 }
    \label{eq:compromiso}
\end{equation}

\subsection{Librerías, Plataforma y Paralelización Computacional}
La implementación se desarrolló en \textbf{Python 3.11.15} integrando:
\begin{itemize}
    \item \texttt{mujoco 3.17.0}: Motor físico en C++ para la evaluación dinámica tridimensional.
    \item \texttt{pymoo 0.6.2}: Framework académico estándar para optimización evolutiva multiobjetivo (creado por el propio K. Deb), utilizado para los operadores SBX, PM, NSGA-II y Evolución Diferencial (DE).
    \item \texttt{cma 4.5.0} y \texttt{scipy 1.17.1}: Empleados para la comparación cruzada contra CMA-ES (Hansen, 2016) y Nelder-Mead Simplex.
    \item \textbf{Paralelización Multiproceso de Cero Sobrecarga (\texttt{ProcessPoolExecutor}):} En la implementación ingenua inicial, recargar el archivo XML de MuJoCo desde disco tomaba $1.16\text{ s}$ por candidato. Al pre-cargar en memoria los modelos nominal e incierto en cada proceso trabajador, pre-computar la trayectoria quíntica $C^2$ y distribuir la población en 12 núcleos paralelos, el tiempo de evaluación cayó a \textbf{0.062 s efectivos por candidato} (aceleración de $18.7\times$), permitiendo ejecutar 480 evaluaciones robustas ($1,920$ simulaciones MuJoCo de $9.0\text{ s}$, equivalentes a $4.8\text{ horas}$ de tiempo físico del robot) en tan solo \textbf{29.88 segundos}.
\end{itemize}

% ==============================================================================
% FASE III: IMPLEMENTACIÓN Y EXPERIMENTACIÓN
% ==============================================================================
\section{Fase III: Implementación y Experimentación}

\subsection{Estructura Modular del Código Implementado}
El sistema evolutivo está estructurado en cuatro módulos desacoplados dentro del repositorio del proyecto:
\begin{enumerate}
    \item \texttt{robot\_ia/ctc\_evolutivo.py}: Define los límites del hiperrectángulo \texttt{LIMITES\_GANANCIAS}, el validador vectorial \texttt{validar\_ganancias(theta)} y la ley de control \texttt{torque\_ctc(q, dq, qr, dqr, ddqr, M, bias, limites, theta)}.
    \item \texttt{optimizar\_ctc\_nsga2.py}: Implementa el evaluador multi-escenario \texttt{evaluar\_robusto()}, el acoplamiento con \texttt{pymoo.algorithms.moo.nsga2.NSGA2}, el registro de evaluaciones y el selector automático de la solución compromiso del frente de Pareto.
    \item \texttt{Computacion Evolutiva/ejecutar\_estudio\_completo\_ce.py}: Suite experimental paralelizada que ejecuta el Modo Tesis ($N_{\text{pop}}=24, G=20$), calcula el Hipervolumen Dominado $HV(g)$ y la diversidad genotípica por generación, ejecuta el estudio de sensibilidad de 9 configuraciones ($S_1 \dots S_9$) y el benchmark contra Random Search, Nelder-Mead, DE y CMA-ES.
    \item \texttt{evaluar\_controladores.py}: Validador externo independiente a $100\text{ Hz}$ que somete las ganancias evolucionadas $\theta^*$ a pruebas de seguimiento, rechazo de perturbación, manipulabilidad de Yoshikawa $w(q)$ y al Modo Juego \textit{``Circuito de 5 Aros''}.
\end{enumerate}

\subsection{Definición y Validación de las Métricas de Evaluación}
Para evaluar rigurosamente tanto la calidad del proceso evolutivo como el desempeño ingenieril del brazo robótico, se emplean dos niveles de métricas:
\begin{enumerate}
    \item \textbf{Métricas Evolutivas Multiobjetivo (Espacio Objetivo y Genotípico):}
    \begin{itemize}
        \item \textbf{Aptitud Directa Bi-Criterio ($f_1, f_2$):} $\text{RMSE}_{\text{seg}}$ robusto [mm] y esfuerzo integral $\int \|\tau\|^2 dt$ [$\text{N}^2\text{m}^2\text{s}$].
        \item \textbf{Métrica Externa Multiobjetivo — Hipervolumen Dominado Normalizado ($HV$):} Mide el área del espacio objetivo dominada por el frente de Pareto $\mathcal{F}_1(g)$ respecto al punto de referencia nadir $r_{\text{ref}} = [40.0\text{ mm},\, 0.560\text{ N}^2\text{m}^2\text{s}]^T$:
        \begin{equation}
            HV(\mathcal{F}_1, r_{\text{ref}}) = \frac{1}{A_{\max}} \operatorname{Lebesgue}\left( \bigcup_{\theta \in \mathcal{F}_1} [f_1(\theta), 40.0] \times [f_2(\theta), 0.560] \right)
        \end{equation}
        El Hipervolumen es la única métrica unaria estrictamente monótona con respecto a la dominancia de Pareto: evalúa simultáneamente \textit{convergencia hacia el frente verdadero} y \textit{extensión/diversidad sobre el frente}.
        \item \textbf{Diversidad Genotípica Normalizada ($\sigma_\theta(g)$):} Desviación estándar media de los genes normalizados en $[0, 1]^6$ a lo largo de las generaciones, permitiendo diagnosticar si existe convergencia prematura.
    \end{itemize}
    \item \textbf{Métricas Externas de Ingeniería de Control (Validación Cruzada en Planta):} Error estacionario en los últimos $2\text{ s}$ ($\text{RMSE}_{2\text{s}}$), error final puntual $e_{\text{final}}$ [mm], tiempo de captura sub-milimétrica $t_{\text{cap}}$ (instante en que el efector ingresa y permanece dentro de la esfera de tolerancia $< 2.0\text{ mm}$), tiempo de recuperación post-perturbación $t_{\text{rec}}$ ($\le 5.0\text{ mm}$) y porcentaje de saturación de actuadores.
\end{enumerate}

% ==============================================================================
% FASE IV: ANÁLISIS DE RESULTADOS Y CONCLUSIONES
% ==============================================================================
\section{Fase IV: Análisis de Resultados y Conclusiones}

\subsection{Frente de Pareto Hallado y Dominancia sobre el Controlador Clásico}
La Figura~\ref{fig:pareto} muestra la distribución de los 480 individuos evaluados en el espacio objetivo bi-criterio y la evolución del frente de Pareto desde el Modo Rápido ($N_{\text{pop}}=8, G=4$, 32 evaluaciones) hasta el Modo Tesis ($N_{\text{pop}}=24, G=20$, 480 evaluaciones).

\begin{figure}[H]
    \centering
    \includegraphics[width=0.92\linewidth]{figuras_ce/fig_ce_01_frente_pareto.png}
    \caption{Espacio objetivo bi-criterio $(f_1, f_2)$ y Frentes de Pareto obtenidos con NSGA-II sobre los 4 escenarios dinámicos en MuJoCo. La zona sombreada verde indica la región que domina estrictamente al controlador analítico CTC Base en ambos objetivos simultáneamente.}
    \label{fig:pareto}
\end{figure}

Se observa una curva de compromiso de Pareto claramente definida que se extiende desde regímenes de \textbf{máxima precisión cinemática} ($\text{RMSE} = 7.640\text{ mm}$, con esfuerzo $0.5128\text{ N}^2\text{m}^2\text{s}$) hasta regímenes de \textbf{ultra-bajo consumo energético} ($\text{RMSE} = 69.104\text{ mm}$, con esfuerzo reducido a $0.4466\text{ N}^2\text{m}^2\text{s}$). Crucialmente, la región superior izquierda del frente (soluciones $\#1$, $\#2$ y $\#3$ de la Tabla~\ref{tab:pareto_solutions}) se ubica dentro del cuadrante verde de \textbf{dominancia estricta de Pareto} sobre el CTC Base analítico ($11.353\text{ mm}$, $0.5159\text{ N}^2\text{m}^2\text{s}$), demostrando que el diseño analítico clásico con ganancias uniformes era simultáneamente subóptimo en precisión y en consumo de energía.

\begin{table}[H]
\centering
\caption{Subconjunto Representativo del Frente de Pareto $\mathcal{F}_1$ Obtenido por NSGA-II ($N_{\text{pop}}=24, G=20$).}
\label{tab:pareto_solutions}
\resizebox{\linewidth}{!}{%
\begin{tabular}{c cccccc cc c}
\toprule
\textbf{ID} & $K_{p1}$ & $K_{p2}$ & $K_{p3}$ & $K_{d1}$ & $K_{d2}$ & $K_{d3}$ & $f_1$: $\text{RMSE}$ [mm] & $f_2$: Esfuerzo [$\text{N}^2\text{m}^2\text{s}$] & \textbf{Domina al Base} \\
\midrule
\textbf{Base} & 169.00 & 169.00 & 169.00 & 28.60 & 28.60 & 28.60 & 11.3533 & 0.515921 & --- \\
\midrule
\textbf{\#1 (Máx. Precisión)} & 212.19 & 215.31 & 249.14 & 51.09 & 26.77 & 26.60 & \textbf{7.6404} & 0.512791 & \textbf{Sí ($\prec$)} \\
\textbf{\#2 (Compromiso Tesis $\theta^*$)} & \textbf{203.94} & \textbf{195.39} & \textbf{249.58} & \textbf{49.65} & \textbf{17.33} & \textbf{4.20} & \textbf{7.8267} & \textbf{0.511447} & \textbf{Sí ($\prec$)} \\
\textbf{Rápido ($\theta^*_{\text{rap}}$)} & 239.88 & 191.92 & 215.83 & 50.22 & 29.67 & 9.00 & 8.8082 & 0.512962 & \textbf{Sí ($\prec$)} \\
\textbf{\#3 (Rodilla Baja Energía)} & 212.28 & 126.03 & 210.66 & 10.97 & 10.55 & 10.47 & 11.2507 & 0.509062 & \textbf{Sí ($\prec$)} \\
\textbf{\#5 (Ahorro Moderado)} & 212.17 & 94.46 & 249.15 & 38.18 & 13.00 & 4.35 & 17.4399 & 0.502589 & No \\
\textbf{\#10 (Bajo Par Hombro)} & 149.49 & 56.42 & 241.88 & 51.80 & 13.03 & 3.69 & 32.3657 & 0.489446 & No \\
\textbf{\#15 (Modo Eco)} & 207.51 & 36.64 & 249.15 & 51.10 & 27.10 & 6.17 & 45.2224 & 0.476613 & No \\
\textbf{\#24 (Mínimo Esfuerzo)} & 212.23 & 24.46 & 249.15 & 51.09 & 12.41 & 4.38 & 69.1043 & \textbf{0.446574} & No \\
\bottomrule
\end{tabular}%
}
\end{table}

\subsection{Análisis de Convergencia, Eficiencia y Dinámica Poblacional}
La Figura~\ref{fig:convergencia} y la Tabla~\ref{tab:convergencia_gen} muestran la evolución generacional ($g = 1 \dots 20$) de la mejor aptitud, la aptitud promedio poblacional, el Hipervolumen Dominado $HV(g)$ y la diversidad genotípica $\sigma_\theta(g)$.

\begin{figure}[H]
    \centering
    \includegraphics[width=1.0\linewidth]{figuras_ce/fig_ce_02_convergencia_generacional.png}
    \caption{Dinámica evolutiva generacional de NSGA-II ($N_{\text{pop}}=24, G=20$): (a) Evolución del mejor RMSE $f_1^{\min}(g)$ y promedio poblacional $\bar{f}_1(g)$, (b) Evolución del esfuerzo energético mínimo $f_2^{\min}(g)$ y promedio $\bar{f}_2(g)$, y (c) Crecimiento del Hipervolumen Dominado $HV(g)$ frente a la diversidad genotípica $\sigma_\theta(g)$.}
    \label{fig:convergencia}
\end{figure}

El análisis de estas tres curvas revela un fenómeno fundamental propio de los algoritmos evolutivos multiobjetivo:
\begin{enumerate}
    \item \textbf{Monotonía Elitista de los Extremos ($f_1^{\min}(g)$ y $f_2^{\min}(g)$):} El mejor RMSE de seguimiento desciende monótonamente desde $8.316\text{ mm}$ ($g=1$) hasta $7.873\text{ mm}$ ($g=4$), $7.694\text{ mm}$ ($g=10$) y finalmente \textbf{$7.640\text{ mm}$} ($g=20$). Simultáneamente, el extremo de mínimo esfuerzo cae de $0.4999\text{ N}^2\text{m}^2\text{s}$ ($g=1$) a \textbf{$0.4466\text{ N}^2\text{m}^2\text{s}$} ($g=20$).
    \item \textbf{¿Por qué el RMSE promedio poblacional $\bar{f}_1(g)$ aumenta de $15.63\text{ mm}$ ($g=2$) a $38.63\text{ mm}$ ($g=20$) mientras el esfuerzo promedio $\bar{f}_2(g)$ disminuye?} En un algoritmo monobjetivo, un aumento del promedio indicaría divergencia; sin embargo, en NSGA-II ocurre porque el operador de \textit{Crowding Distance} descubre y puebla progresivamente la cola derecha del frente de Pareto (individuos con $K_{p2} \in [24, 60]$ que consumen poquísima energía $f_2 < 0.47\text{ N}^2\text{m}^2\text{s}$ a cambio de mayor error $f_1 \in [35, 69]\text{ mm}$).
    \item \textbf{Ausencia de Convergencia Prematura y Suficiencia de Generaciones:} El Hipervolumen Normalizado $HV(g)$ crece rápidamente entre $g=1$ ($0.4834$) y $g=10$ ($0.5489$), estabilizándose asintóticamente en torno a $0.5489$ hacia $g=20$, mientras que la diversidad genotípica se estabiliza en un piso saludable $\sigma_\theta \approx 0.153 > 0$ (lejos del colapso $\sigma_\theta \to 0$). Esto confirma que no hubo convergencia prematura y que $G = 20$ generaciones ($480$ evaluaciones) fueron suficientes para alcanzar el equilibrio del frente.
\end{enumerate}

\begin{table}[H]
\centering
\caption{Traza Generacional del Algoritmo NSGA-II en Modo Tesis ($N_{\text{pop}}=24, G=20$).}
\label{tab:convergencia_gen}
\resizebox{\linewidth}{!}{%
\begin{tabular}{c c cc cc c c c}
\toprule
\textbf{Gen.} & \textbf{Evals.} & $f_1^{\min}$ [mm] & $\bar{f}_1 \pm \sigma_{f1}$ [mm] & $f_2^{\min}$ [$\text{N}^2\text{m}^2\text{s}$] & $\bar{f}_2$ [$\text{N}^2\text{m}^2\text{s}$] & \textbf{Hipervolumen $HV$} & $|\mathcal{F}_1|$ & \textbf{Diversidad $\sigma_\theta$} \\
\midrule
1  & 24  & 8.3163 & $26.39 \pm 17.24$ & 0.499867 & 0.528223 & 0.4834 & 4  & 0.2737 \\
2  & 48  & 8.1420 & $15.63 \pm 6.15$  & 0.499867 & 0.508585 & 0.4998 & 9  & 0.2364 \\
3  & 72  & 8.0642 & $18.08 \pm 5.76$  & 0.497047 & 0.504851 & 0.5149 & 17 & 0.2236 \\
4  & 96  & 7.8731 & $18.23 \pm 6.75$  & 0.495707 & 0.504126 & 0.5241 & 18 & 0.2258 \\
5  & 120 & 7.8731 & $18.99 \pm 7.96$  & 0.490851 & 0.503031 & 0.5366 & 24 & 0.2181 \\
6  & 144 & 7.8731 & $20.98 \pm 8.70$  & 0.484615 & 0.500915 & 0.5444 & 24 & 0.1916 \\
8  & 192 & 7.7283 & $27.48 \pm 12.12$ & 0.468266 & 0.494398 & 0.5467 & 24 & 0.1550 \\
10 & 240 & 7.6942 & $29.22 \pm 13.35$ & 0.468266 & 0.492342 & 0.5489 & 24 & 0.1634 \\
12 & 288 & 7.6937 & $27.81 \pm 12.87$ & 0.468266 & 0.493812 & 0.5487 & 24 & 0.1585 \\
15 & 360 & 7.6937 & $36.31 \pm 18.11$ & 0.450821 & 0.484403 & 0.5448 & 24 & 0.1527 \\
18 & 432 & 7.6418 & $38.18 \pm 19.81$ & 0.446691 & 0.482070 & 0.5413 & 24 & 0.1633 \\
20 & 480 & \textbf{7.6404} & $38.63 \pm 19.50$ & \textbf{0.446574} & 0.481427 & \textbf{0.5489} & \textbf{24} & 0.1529 \\
\bottomrule
\end{tabular}%
}
\end{table}

\subsection{Impacto de los Parámetros (Estudio de Sensibilidad $S_1$ a $S_9$)}
Para justificar rigurosamente la configuración hiperparamétrica elegida, se ejecutaron nueve configuraciones experimentales ($S_1$ a $S_9$) variando el tamaño de población $N_{\text{pop}}$, el número de generaciones $G$, la probabilidad de cruce $p_c$, el índice de distribución de cruce $\eta_c$, la probabilidad de mutación $p_m$ y el índice de mutación $\eta_m$ (Tabla~\ref{tab:sensibilidad} y Figura~\ref{fig:sens_comp}a).

\begin{table}[H]
\centering
\caption{Resultados del Estudio de Sensibilidad Paramétrica sobre NSGA-II en el Gemelo Digital MuJoCo.}
\label{tab:sensibilidad}
\resizebox{\linewidth}{!}{%
\begin{tabular}{l ccccccc cccc}
\toprule
\textbf{Configuración} & $N_{\text{pop}}$ & $G$ & \textbf{Evals} & $p_c$ & $\eta_c$ & $p_m$ & $\eta_m$ & $\text{RMSE}_{\text{sel}}$ [mm] & Esfuerzo [$\text{N}^2\text{m}^2\text{s}$] & \textbf{HV} & $t_{\text{ejec}}$ [s] \\
\midrule
$S_1$: Modo Rápido           & 8  & 4  & 32  & 0.90 & 15 & 0.167 & 20 & 8.9464 & 0.514169 & 0.4855 & 2.34  \\
$S_2$: Población Media       & 16 & 10 & 160 & 0.90 & 15 & 0.167 & 20 & 9.3395 & 0.514982 & 0.5094 & 12.32 \\
\textbf{$S_3$: Tesis Nominal (Elegida)} & \textbf{24} & \textbf{20} & \textbf{480} & \textbf{0.90} & \textbf{15} & \textbf{0.167} & \textbf{20} & \textbf{7.8267} & \textbf{0.511447} & \textbf{0.5489} & \textbf{29.88} \\
$S_4$: Población Alta        & 36 & 15 & 540 & 0.90 & 15 & 0.167 & 20 & 7.9124 & 0.512285 & 0.5422 & 37.84 \\
$S_5$: Cruce Bajo ($p_c=0.6$) & 16 & 12 & 192 & 0.60 & 15 & 0.167 & 20 & 8.5714 & 0.513152 & 0.5280 & 14.71 \\
$S_6$: Cruce Difuso ($\eta_c=5$) & 16 & 12 & 192 & 0.90 & 5  & 0.167 & 20 & 8.6023 & 0.512995 & 0.5222 & 15.04 \\
$S_7$: Mutación Baja ($p_m=0.05$) & 16 & 12 & 192 & 0.90 & 15 & 0.050 & 20 & 8.9757 & 0.513733 & 0.5126 & 15.05 \\
$S_8$: Mutación Alta ($p_m=0.45$) & 16 & 12 & 192 & 0.90 & 15 & 0.450 & 20 & 8.1281 & 0.512346 & 0.5318 & 14.59 \\
$S_9$: Mutación Agresiva ($\eta_m=5$) & 16 & 12 & 192 & 0.90 & 15 & 0.600 & 5  & 8.8040 & 0.513581 & 0.5119 & 15.45 \\
\bottomrule
\end{tabular}%
}
\end{table}

\begin{figure}[H]
    \centering
    \includegraphics[width=1.0\linewidth]{figuras_ce/fig_ce_03_sensibilidad_y_comparacion.png}
    \caption{(a) Sensibilidad del RMSE compromiso y del Hipervolumen Dominado $HV$ ante las 9 configuraciones hiperparamétricas ($S_1$ a $S_9$). (b) Comparación del rendimiento entre los 7 paradigmas de optimización evaluados sobre el mismo Gemelo Digital MuJoCo.}
    \label{fig:sens_comp}
\end{figure}

\paragraph{Hallazgos del Estudio de Sensibilidad:}
\begin{enumerate}
    \item \textbf{Relación entre Tamaño de Población $N_{\text{pop}}$ y Profundidad Generacional $G$ ($S_1 \dots S_4$):} Comparando $S_3$ ($N_{\text{pop}}=24, G=20$, 480 evals) con $S_4$ ($N_{\text{pop}}=36, G=15$, 540 evals), se observa que $S_3$ alcanza un mayor Hipervolumen ($0.5489$ vs. $0.5422$), menor RMSE ($7.827\text{ mm}$ vs. $7.912\text{ mm}$) y un $21\%$ menos de tiempo de cómputo ($29.88\text{ s}$ vs. $37.84\text{ s}$). Esto demuestra que, en espacios continuos de dimensión moderada ($D=6$), profundizar hasta $G=20$ generaciones con $N_{\text{pop}}=24$ es más eficiente para refinar la frontera de Pareto mediante SBX que dispersar el presupuesto en una población excesivamente ancha con pocas generaciones.
    \item \textbf{Sensibilidad a la Tasa y Distribución de Mutación ($S_7, S_8, S_9$):} Con mutación demasiado baja ($S_7$, $p_m = 0.05$, equivalente a mutar solo 0.3 genes por individuo), la población pierde capacidad de ajuste local y se estanca en $\text{RMSE} = 8.976\text{ mm}$ ($HV = 0.5126$). Elevar la probabilidad de mutación a un régimen moderado-alto con paso fino ($S_8$, $p_m = 0.45, \eta_m = 20$) mejora significativamente la exploración de los bordes del espacio ($K_{p3} \to 250$, $K_{d3} \to 4.2$), reduciendo el RMSE a $8.128\text{ mm}$ ($HV = 0.5318$). Sin embargo, cuando se combina alta tasa de mutación con saltos largos ($S_9$, $p_m = 0.60, \eta_m = 5$), la mutación se vuelve destructiva y el desempeño retrocede a $8.804\text{ mm}$ ($HV = 0.5119$).
    \item \textbf{Sensibilidad al Operador de Cruce SBX ($S_5, S_6$):} Reducir $p_c$ a $0.60$ ($S_5$) o usar un índice difuso $\eta_c = 5$ ($S_6$) ralentiza la explotación sinérgica de los pares $(K_{p,i}, K_{d,i})$ frente al valor canónico $p_c = 0.90, \eta_c = 15$ ($S_3$).
\end{enumerate}

\subsection{Interpretación Física y Biomecánica de la Solución Óptima $\theta^*$}
La Figura~\ref{fig:ganancias} compara la estructura interna del cromosoma clásico $\theta_{\text{base}} = [169, 169, 169, 28.6, 28.6, 28.6]^T$ frente a las soluciones Pareto-óptimas halladas por NSGA-II en Modo Rápido ($\theta^*_{\text{rap}} = [239.88,\, 191.92,\, 215.83,\, 50.22,\, 29.67,\, 9.00]^T$) y en Modo Tesis ($\theta^*_{\text{tesis}} = [203.94,\, 195.39,\, 249.58,\, 49.65,\, 17.33,\, 4.20]^T$).

\begin{figure}[H]
    \centering
    \includegraphics[width=0.96\linewidth]{figuras_ce/fig_ce_04_ganancias_cromosoma.png}
    \caption{Comparación física de los genes del controlador CTC Base frente a las soluciones evolucionadas por NSGA-II: (a) Ganancias proporcionales $K_{p,i}$ y (b) Ganancias derivativas $K_{d,i}$ para las tres articulaciones del brazo.}
    \label{fig:ganancias}
\end{figure}

Ambas ejecuciones de NSGA-II descubrieron de forma emergente —sin habérselo programado explícitamente— la misma ley estructural biomecánica para el manipulador antropomórfico RRR:
\begin{enumerate}
    \item \textbf{Rigidización Proporcional Progresiva ($K_{p,i} > 169\text{ s}^{-2}$):} El algoritmo incrementa $K_{p1} \in [204, 240]\text{ s}^{-2}$, $K_{p2} \in [192, 195]\text{ s}^{-2}$ y lleva $K_{p3}$ cerca del límite superior ($215.8 - 249.6\text{ s}^{-2}$). Esto contrarresta directamente el error estacionario inducido cuando la masa real de los eslabones aumenta en $+25\%/+15\%$ respecto al modelo nominal $\hat{M}(q)$ y rechaza el pulso de perturbación externa en $t \in [6.0, 6.6]\text{ s}$.
    \item \textbf{Gradiente Decreciente de Amortiguamiento Derivativo ($K_{d1} \gg K_{d2} \gg K_{d3}$):} Este es el hallazgo más notable del algoritmo evolutivo. Mientras el diseño clásico asigna $K_{d1} = K_{d2} = K_{d3} = 28.6\text{ s}^{-1}$, NSGA-II descubre que debe \textbf{elevar fuertemente el amortiguamiento de la cintura} ($K_{d1} \approx 50.0\text{ s}^{-1}$, razón de amortiguamiento efectiva $\zeta_1 = K_{d1}/(2\sqrt{K_{p1}}) \approx 1.74$, régimen sobreamortiguado que frena la inercia rotacional de todo el brazo en el plano XY), mantener un amortiguamiento moderado en el hombro ($K_{d2} \in [17.3, 29.7]\text{ s}^{-1}$, $\zeta_2 \approx 0.62 - 1.07$) y \textbf{abatir drásticamente la ganancia derivativa del codo distal} hasta $K_{d3} \in [4.20, 9.00]\text{ s}^{-1}$ ($\zeta_3 \approx 0.13 - 0.31$). Dado que el eslabón distal $q_3$ posee baja inercia pero el menor límite de par ($\tau_{\max,3} = 0.45\text{ N}\cdot\text{m}$) y sufre amortiguamiento viscoso físico $+30\%$ en el escenario incierto, reducir $K_{d3}$ elimina el gasto inútil de torque derivativo $\hat{M}_{33} K_{d3}(\dot{q}_{r3} - \dot{q}_3)$, reduciendo el objetivo $f_2$ sin sacrificar precisión cartesiana.
\end{enumerate}

\subsection{Comparación y Validación contra Métodos Clásicos y Otros Algoritmos Evolutivos}
Para cumplir con la exigencia de validación comparativa del documento guía, se contrastó NSGA-II en dos niveles: (A) en el problema de optimización de ganancias frente a otros 5 optimizadores (Tabla~\ref{tab:comp_optimizadores} y Figura~\ref{fig:sens_comp}b), y (B) en validación operativa a $100\text{ Hz}$ frente a otras 4 arquitecturas de control robótico (Tabla~\ref{tab:validacion_controladores} y Figura~\ref{fig:curvas_planta}).

\begin{table}[H]
\centering
\caption{Comparación Multi-Paradigma de Algoritmos de Optimización sobre la Planta Robusta (4 Escenarios).}
\label{tab:comp_optimizadores}
\resizebox{\linewidth}{!}{%
\begin{tabular}{l l c cc cc c}
\toprule
\textbf{Método de Optimización} & \textbf{Paradigma} & \textbf{Evals} & $\text{RMSE}$ [mm] & Esfuerzo [$\text{N}^2\text{m}^2\text{s}$] & $\Delta \text{RMSE}$ & $\Delta \text{Esf.}$ & \textbf{Entrega PF} \\
\midrule
1. CTC Base (Asignación de Polos) & Clásico Analítico & 1 & 11.3533 & 0.515921 & $0.00\%$ & $0.00\%$ & No \\
2. Random Search (Monte Carlo) & Heurística Básica & 160 & 7.7897 & 0.513671 & $-31.39\%$ & $-0.44\%$ & No \\
3. Nelder-Mead Simplex & Clásico sin Derivadas & 60 & 7.6265 & 0.512901 & $-32.83\%$ & $-0.59\%$ & No \\
4. Evolución Diferencial (\texttt{DE/rand/1/bin}) & Evolutivo Monobjetivo & 160 & 7.5830 & 0.512321 & $-33.21\%$ & $-0.70\%$ & No \\
5. Estrategia Evolutiva \textbf{CMA-ES} & Evolutivo Covarianza & 160 & \textbf{7.5763} & 0.513552 & $\mathbf{-33.27\%}$ & $-0.46\%$ & No \\
6. \textbf{NSGA-II (Modo Rápido)} & Evolutivo Multiobjetivo & 32 & 8.8082 & 0.512962 & $-22.42\%$ & $-0.57\%$ & \textbf{Sí (8 sol.)} \\
7. \textbf{NSGA-II (Modo Tesis — Compromiso)} & Evolutivo Multiobjetivo & 480 & 7.8267 & \textbf{0.511447} & $-31.06\%$ & $\mathbf{-0.87\%}$ & \textbf{Sí (24 sol.)} \\
8. \textbf{NSGA-II (Modo Tesis — Extremo $\#1$)} & Evolutivo Multiobjetivo & 480 & \textbf{7.6404} & 0.512791 & $-32.70\%$ & $-0.61\%$ & \textbf{Sí (24 sol.)} \\
\bottomrule
\end{tabular}%
}
\end{table}

\paragraph{Discusión de la Comparación Multi-Paradigma:}
\begin{itemize}
    \item Los algoritmos evolutivos especializados en optimización continua monobjetivo (\textbf{DE} con $7.583\text{ mm}$ y \textbf{CMA-ES} con $7.576\text{ mm}$) y el simplex local de \textbf{Nelder-Mead} ($7.627\text{ mm}$, que converge rápido hacia la frontera superior $K_{p1}=K_{p3}=250.0$) alcanzan el mínimo error cuando se les suministra una función escalarizada fija $J(\theta) = 0.5(f_1/f_{1,\text{base}}) + 0.5(f_2/f_{2,\text{base}})$.
    \item Sin embargo, \textbf{NSGA-II Modo Tesis} iguala virtualmente dicha precisión en su extremo $\#1$ ($7.640\text{ mm}$) y logra en su solución compromiso $\#2$ ($7.827\text{ mm}$) el \textbf{menor consumo energético de todos los métodos de alta precisión} ($0.511447\text{ N}^2\text{m}^2\text{s}$, $-0.87\%$), entregando además 24 controladores óptimos para distintos presupuestos de batería/temperatura sin necesidad de re-ejecutar el optimizador.
\end{itemize}

\begin{table}[H]
\centering
\caption{Validación Operativa Cruzada a $100\text{ Hz}$ de los 5 Controladores del Proyecto (\texttt{evaluar\_controladores.py}).}
\label{tab:validacion_controladores}
\resizebox{\linewidth}{!}{%
\begin{tabular}{l l cccccccc}
\toprule
\textbf{Controlador} & \textbf{Régimen} & $\text{RMSE}_{\text{seg}}$ [mm] & $\text{RMSE}_{2\text{s}}$ [mm] & $e_{\text{final}}$ [mm] & Esfuerzo [$\text{N}^2\text{m}^2\text{s}$] & $t_{\text{cap}}$ [s] & $t_{\text{rec}}$ [s] & Sat. [\%] & $t_{\text{calc}}$ [ms] \\
\midrule
\multirow{2}{*}{\textbf{PD (CTC Base)}} & Nominal & 6.879 & 1.070 & 0.892 & 0.4861 & 5.84 & --- & 0.0\% & 0.018 \\
 & Perturbado & 8.651 & 1.832 & 1.441 & 0.4921 & 5.84 & 0.582 & 0.0\% & 0.018 \\
\midrule
\multirow{2}{*}{\textbf{EVO (CTC NSGA-II)}} & \textbf{Nominal} & \textbf{4.573} & \textbf{0.636} & \textbf{0.516} & \textbf{0.4817} & \textbf{4.50} & --- & \textbf{0.0\%} & \textbf{0.018} \\
 & \textbf{Perturbado} & \textbf{6.557} & \textbf{1.132} & \textbf{0.887} & \textbf{0.4902} & \textbf{4.50} & \textbf{0.382} & \textbf{0.0\%} & \textbf{0.018} \\
\midrule
\multirow{2}{*}{\textbf{MPC ($N=8$)}} & Nominal & 7.137 & 1.144 & 0.961 & 0.4865 & 6.04 & --- & 0.0\% & 14.60 \\
 & Perturbado & 8.947 & 1.928 & 1.531 & 0.4922 & 6.02 & 0.622 & 0.0\% & 14.80 \\
\midrule
\multirow{2}{*}{\textbf{HYB (CTC-MPC)}} & Nominal & 6.815 & 1.145 & 0.961 & 0.4863 & 6.06 & --- & 0.0\% & 14.70 \\
 & Perturbado & 8.693 & 1.928 & 1.532 & 0.4920 & 6.02 & 0.622 & 0.0\% & 14.90 \\
\midrule
\multirow{2}{*}{\textbf{KIN (CLIK-DLS)}} & Nominal & 4.236 & 0.365 & 0.273 & 0.4816 & 4.38 & --- & 0.0\% & 0.042 \\
 & Perturbado & 5.507 & 0.794 & 0.565 & 0.4911 & 4.38 & 0.242 & 0.0\% & 0.042 \\
\bottomrule
\end{tabular}%
}
\end{table}

\begin{figure}[H]
    \centering
    \includegraphics[width=0.98\linewidth]{figuras_ce/fig_curvas_simulacion.png}
    \caption{Validación en el Gemelo Digital MuJoCo: (a) Evolución temporal del error cartesiano ante la perturbación en $t \in [6.0, 6.6]\text{ s}$ mostrando la superioridad de EVO (NSGA-II) frente a CTC Base y MPC; (b) Rapidez cartesiana $\|\dot{p}\|$; (c) Índice de manipulabilidad de Yoshikawa $w(q)$; y (d) Desempeño del controlador evolutivo en el Circuito de 5 Aros (Estándar, Rápido y Turbo, logrando 5/5 encestes).}
    \label{fig:curvas_planta}
\end{figure}

Como evidencia la Tabla~\ref{tab:validacion_controladores}, una vez desplegadas las ganancias evolucionadas en línea, el controlador \textbf{EVO (CTC NSGA-II)} tiene un costo computacional de inferencia idéntico al controlador PD clásico (\textbf{$0.018\text{ ms}$} por ciclo, $810$ veces más rápido que el MPC de $14.60\text{ ms}$), pero reduce el RMSE nominal en un \textbf{33.5\%} ($4.573\text{ mm}$ vs. $6.879\text{ mm}$), el error estacionario final en un \textbf{42.2\%} ($0.516\text{ mm}$ vs. $0.892\text{ mm}$), el tiempo de captura en \textbf{1.34 segundos} ($4.50\text{ s}$ vs. $5.84\text{ s}$) y el tiempo de recuperación tras perturbación en un \textbf{34.4\%} ($0.382\text{ s}$ vs. $0.582\text{ s}$).

\subsection{Confrontación Adversarial en Gemelo Digital MuJoCo Hostil (Reparametrización Modal Tri-Objetivo, Sustituto RBF y Protocolo de $N=30$ Semillas con Wilcoxon-Holm)}

Un análisis crítico de primeros principios sobre la solución del gemelo idealizado ($\theta^*_{\text{tesis}}$ en la Tabla~\ref{tab:frente_pareto}, donde $K_{p3} \to 249.58\text{ s}^{-2}$ y $K_{d3} \to 4.20\text{ s}^{-1} \implies \zeta_3 \approx 0.133$) plantea una pregunta científica fundamental: \textit{¿es el bajo amortiguamiento distal $\zeta_3 = 0.133$ una ventaja biomecánica genuina o explota la ausencia de ruido de cuantización y retardo en el simulador ideal?} Asimismo, cruzar directamente $\theta = [K_{p,i}, K_{d,i}]$ mediante SBX puede alterar la razón de amortiguamiento $\zeta_i = K_{d,i}/(2\sqrt{K_{p,i}})$.

Para responder de manera definitiva e irrefutable con datos sobre el motor físico 3D \textbf{MuJoCo} (\texttt{Computacion Evolutiva/Mejoras/confrontacion\_definitiva\_mujoco\_30semillas.py}), se implementó una extensión unificada con cinco mejoras estructurales:
\begin{enumerate}
    \item \textbf{Gemelo Digital 3D MuJoCo Hostil con Números Aleatorios Comunes (CRN):} Se inyectó ruido gaussiano realista en los encoders ($\sigma_q = 0.002\text{ rad}, \sigma_{\dot{q}} = 0.020\text{ rad/s}$) y un retardo de transporte en lazo cerrado de 1 ciclo de control ($\Delta t_{\text{ctrl}} = 10\text{ ms}$), evaluando cada individuo sobre un \textbf{Mini-Batch Espacial de Trayectorias 3D} generado por \textit{Latin Hypercube Sampling} (LHS) bajo los 4 escenarios dinámicos ($s_1 \dots s_4$).
    \item \textbf{Formulación Tri-Objetivo con Tasa de Par Anti-Chattering ($f_3$) y Agregación Robusta $\mathcal{H}_\infty$:} Se incorporó el tercer objetivo $f_3(\theta) = \int_0^T \|\dot{\tau}(t)\|_2^2 dt$ $[\text{N}^2\text{m}^2/\text{s}]$ para penalizar la vibración de alta frecuencia (\textit{chattering}), agregando los episodios mediante el funcional robusto media-peor caso ($\lambda = 0.5$):
    \begin{equation}
        f_m^{\text{rob}}(\theta) = (1-\lambda)\,\frac{1}{|\mathcal{B}|}\sum_{b \in \mathcal{B}} f_m^{(b)}(\theta) + \lambda \max_{b \in \mathcal{B}} f_m^{(b)}(\theta), \quad m \in \{1, 2, 3\}
        \label{eq:triobjetivo_rob}
    \end{equation}
    \item \textbf{Reparametrización Genotípica Modal ($\tilde{\theta} \in \mathbb{R}^6$):} El cromosoma se codifica en coordenadas modales físicas $\tilde{\theta} = [\omega_{n1}, \omega_{n2}, \omega_{n3}, \zeta_1, \zeta_2, \zeta_3]^T \in [4.472, 15.811]^3 \times [0.35, 2.00]^3$ y se decodifica exactamente mediante $K_{p,i} = \omega_{n,i}^2$, $K_{d,i} = 2\zeta_i\omega_{n,i}$. Esto alinea las variedades de amortiguamiento constante con los ejes de SBX e impone el prior físico $\zeta_i \ge 0.35$ como una simple cota de caja.
    \item \textbf{Controlador CTC con Ganancia Programada (\textit{Gain-Scheduling}, $D=12$ genes) + Sustituto RBF (LIE) + Búsqueda Local Memética:} Se evolucionó la ley no lineal dependiente del error $\omega_{n,i}(e_i) = \min(\omega_{n0,i} + \Delta\omega_{n,i}\exp(-|e_i|/\sigma_i), \omega_{\max})$ asistida por un ensamble sustituto de Base Radial (RBF) con criterio \textit{Lower Confidence Bound} (LIE) y refinamiento direccional memético en la rodilla del frente.
    \item \textbf{Protocolo Estadístico de $N = 30$ Semillas Independientes con Pruebas de Friedman y Wilcoxon-Holm:} Se confrontaron 5 controladores a lo largo de 30 lotes LHS y realizaciones de ruido independientes en MuJoCo 3D ($1,800$ simulaciones dinámicas tridimensionales): (\textbf{C1}) CTC Base Analítico ($\zeta=1.10$), (\textbf{C2}) $\theta^*_{\text{ideal}}$ de la Tabla~\ref{tab:frente_pareto} ($\zeta_3=0.133$), (\textbf{C3}) NSGA-II Directo $[K_p,K_d]$ evolucionado en gemelo hostil, (\textbf{C4}) NSGA-II Modal $[\omega_n,\zeta]$ Tri-Objetivo ($\omega_n = [13.85, 12.88, 15.22]\text{ rad/s}, \zeta = [1.015, 0.806, 1.281]$) y (\textbf{C5}) NSGA-II Gain-Scheduled + RBF + Memético.
\end{enumerate}

\begin{table}[H]
\centering
\caption{Confrontación Estadística en el Gemelo Digital 3D MuJoCo Hostil ($N=30$ Semillas Independientes, Ruido $\sigma_q=0.002\text{ rad}$, Retardo $10\text{ ms}$, Mini-Batch LHS, Test de Friedman y Wilcoxon con Corrección de Holm-Bonferroni).}
\label{tab:confrontacion_30semillas}
\resizebox{\linewidth}{!}{%
\begin{tabular}{l c c c c c c c}
\toprule
\textbf{Controlador Evaluado ($N=30$)} & \textbf{$f_1^{\text{rob}}$ Mediana (IQR)} & \textbf{$f_2^{\text{rob}}$ Mediana (IQR)} & \textbf{$f_3^{\text{rob}}$ Chattering Med. (IQR)} & \textbf{Rango} & \textbf{Rango} & \textbf{$p_{\text{Holm}}$ ($f_1$)} & \textbf{$p_{\text{Holm}}$ ($f_3$)} \\
 & $[\text{mm}]$ ($\downarrow$) & $[\text{N}^2\text{m}^2\text{s}]$ ($\downarrow$) & $[\text{N}^2\text{m}^2/\text{s}]$ ($\downarrow$) & \textbf{Friedman $f_1$} & \textbf{Friedman $f_3$} & \textbf{vs. C1 Base} & \textbf{vs. C2 Ideal} \\
\midrule
\textbf{C1: CTC Base Analítico} ($\omega_n=13, \zeta=1.10$) & $13.235\ (0.324)$ & $0.4422\ (0.0040)$ & $2.247\ (0.258)$ & $5.00$ & $2.33$ & --- & $7.45 \times 10^{-9}$ \\
\textbf{C2: $\theta^*_{\text{ideal}}$ Tabla 2} ($\zeta_3=0.133$, borde sin ruido) & $\mathbf{9.713\ (0.165)}$ & $\mathbf{0.4404\ (0.0038)}$ & $3.684\ (0.499)$ & $\mathbf{1.00}$ & $5.00$ (Peor) & $7.45 \times 10^{-9}$ & --- \\
\textbf{C3: NSGA-II Directo Hostil} ($[K_p,K_d] \in \mathbb{R}^6$) & $9.916\ (0.267)$ & $0.4406\ (0.0040)$ & $3.334\ (0.468)$ & $2.00$ & $4.00$ & $7.45 \times 10^{-9}$ & $7.45 \times 10^{-9}$ \\
\textbf{C4: NSGA-II Modal Tri-Obj.} ($[\omega_n,\zeta], \zeta \in [0.81, 1.28]$) & $11.302\ (0.330)$ & $0.4415\ (0.0041)$ & $2.337\ (0.315)$ & $3.50$ & $2.67$ & $7.45 \times 10^{-9}$ & $7.45 \times 10^{-9}$ \\
\textbf{C5: NSGA-II Gain-Sched + RBF + Memético} ($D=12$) & $11.364\ (0.172)$ & $0.4409\ (0.0040)$ & $\mathbf{1.922\ (0.289)}$ & $3.50$ & $\mathbf{1.00}$ (Mejor) & $7.45 \times 10^{-9}$ & $7.45 \times 10^{-9}$ \\
\bottomrule
\multicolumn{8}{l}{\footnotesize \textbf{Test Omnibus de Friedman ($N=30$ bloques, $k=5$):} Precisión $f_1$: $\chi^2_F = 114.00,\ p = 1.02 \times 10^{-23}$. Chattering $f_3$: $\chi^2_F = 114.96,\ p = 6.36 \times 10^{-24}$.}
\end{tabular}%
}
\end{table}

\begin{figure}[H]
    \centering
    \includegraphics[width=0.99\linewidth]{figuras_ce/fig_ce_05_confrontacion_hostil_30semillas.png}
    \caption{Confrontación experimental en el Gemelo Digital 3D MuJoCo Hostil: (a) Frente de Pareto tri-objetivo verificando con Números Aleatorios Comunes (CRN) que tanto el óptimo Modal ($\theta^*_{\text{mod}}$) como el Gain-Scheduled ($\theta^*_{\text{GS}}$) dominan estrictamente al CTC Base analítico; (b) Convergencia del Hipervolumen $HV(g)$ y métrica $IGD^+$; (c) Distribución de $f_1^{\text{rob}}$ en $N=30$ semillas independientes; y (d) Demostración estadística del incremento de \textit{chattering} $f_3^{\text{rob}}$ en $\theta^*_{\text{ideal}}$ ($\zeta_3=0.133$) y su supresión mediante la reparametrización modal y el \textit{Gain-Scheduling}.}
    \label{fig:confrontacion_30semillas}
\end{figure}

\paragraph{Hallazgos Científicos de la Confrontación en 30 Semillas (Tabla~\ref{tab:confrontacion_30semillas} y Figura~\ref{fig:confrontacion_30semillas}):}
\begin{enumerate}
    \item \textbf{Veredicto Experimental sobre $\theta^*_{\text{ideal}}$ ($\zeta_3 = 0.133$):} Al someter el cromosoma ganador del gemelo idealizado (\textbf{C2}) al gemelo MuJoCo hostil con ruido y retardo de $10\text{ ms}$, su precisión de seguimiento se mantiene excelente (mediana $f_1 = 9.713\text{ mm}$, superando a \textbf{C1} en las 30 semillas con $p_{\text{Holm}} = 7.45 \times 10^{-9}$ gracias a la fricción viscosa física de MuJoCo), \textbf{pero su índice de vibración de par $f_3 = \int \|\dot{\tau}\|^2 dt$ se dispara a una mediana de $3.684\text{ N}^2\text{m}^2/\text{s}$} ($+64.0\%$ más \textit{chattering} que el CTC Base de $2.247$, Rango de Friedman $5.00/5.00$). Esto demuestra cuantitativamente que abatir $K_{d3} \to 4.20\text{ s}^{-1}$ con $K_{p3} \to 249.58\text{ s}^{-2}$ amplifica el ruido de encoder a través del término proporcional no amortiguado.
    \item \textbf{Superioridad del Genotipo Modal (\textbf{C4}) y del \textit{Gain-Scheduling} con Sustituto RBF (\textbf{C5}):} La reparametrización modal \textbf{C4} descubre razones de amortiguamiento físicamente bien acondicionadas ($\zeta = [1.015, 0.806, 1.281]$, lejos de los bordes de subamortiguamiento), logrando dominar estrictamente al CTC Base en el lote de entrenamiento ($f_1 = 11.15\text{ mm} < 14.38\text{ mm}$, $f_2 = 0.3651 < 0.3679$, $f_3 = 1.815 < 1.930$) y reduciendo el error mediano en 30 semillas a $11.302\text{ mm}$ ($p_{\text{Holm}} = 7.45 \times 10^{-9}$) mientras recorta el \textit{chattering} en un \textbf{36.6\%} respecto a \textbf{C2}. Aún mejor, \textbf{C5} (\textit{Gain-Scheduling} + RBF + búsqueda memética) alcanza la menor métrica de distancia generacional invertida ($IGD^+ = 0.0277$) y el \textbf{menor \textit{chattering} de todos los controladores evaluados} ($f_3 = 1.922\text{ N}^2\text{m}^2/\text{s}$, Rango $1.00$, un \textbf{47.8\%} menos vibración de par que \textbf{C2} y un \textbf{14.5\%} menos que el propio CTC Base analítico, con $p_{\text{Holm}} = 7.45 \times 10^{-9}$).
\end{enumerate}

\subsection{Conclusiones, Limitaciones y Trabajos Futuros}

\paragraph{1. Logros Alcanzados:}
\begin{itemize}
    \item Se formuló y resolvió exitosamente el problema de sintonización óptima robusta de ganancias CTC ($\theta \in \mathbb{R}^6$) como un problema multiobjetivo con restricciones físicas sobre un Gemelo Digital 3D en MuJoCo bajo incertidumbre paramétrica y perturbaciones externas.
    \item El algoritmo evolutivo \textbf{NSGA-II} halló frentes de Pareto con hasta 24 soluciones no dominadas, descubriendo soluciones compromiso que dominan estrictamente al diseño analítico clásico tanto en precisión ($\Delta\text{RMSE} = -31.1\%$ en evaluación robusta de 4 escenarios y $-33.5\%$ en régimen nominal) como en ahorro energético ($\Delta\text{Esfuerzo} = -0.87\%$), manteniendo $0.0\%$ de saturación en los servomotores Dynamixel.
    \item Se demostró desde primeros principios biomecánicos que la evolución desacopla de manera inteligente las razones de amortiguamiento ($\zeta_1 \approx 1.74$ sobreamortiguado en la base inercial frente a $\zeta_3 \approx 0.13-0.31$ en el codo ligero).
\end{itemize}

\paragraph{2. Limitaciones Identificadas:}
\begin{itemize}
    \item \textbf{Costo computacional de la evaluación física en bucle (\textit{Simulation-in-the-Loop}):} Cada evaluación de aptitud requiere integrar $4 \times 4,500 = 18,000$ pasos de dinámica de cuerpo rígido en MuJoCo. Aunque la paralelización en 12 procesos redujo el tiempo de 480 evaluaciones a $29.88\text{ s}$, escalar a manipuladores de 6 o 7 GDL con miles de generaciones requeriría modelos subrogados (\textit{Surrogate-Assisted Evolutionary Algorithms}).
    \item \textbf{Estructura de ganancias constantes:} El cromosoma $\theta \in \mathbb{R}^6$ optimiza ganancias fijas en el espacio articular, las cuales son óptimas para la envolvente estudiada pero no se adaptan dinámicamente cuando el índice de manipulabilidad de Yoshikawa $w(q)$ se aproxima a cero en singularidades.
\end{itemize}

\paragraph{3. Trabajos Futuros y Proyección a Tesis de Maestría:}
\begin{itemize}
    \item \textbf{Optimización de Muchos Objetivos (NSGA-III / MOEA/D en $\mathbb{R}^4$):} Extender el vector objetivo a cuatro criterios: $\mathbf{F}(\theta) = [\text{RMSE}_{\text{seg}},\, \int \|\tau\|^2 dt,\, \int \|\dot{\tau}\|^2 dt \text{ (suavidad/sacudida)},\, -w_{\min}(q)]^T$, integrando la optimización de ganancias con la planificación evolutiva de trayectorias libres de singularidades.
    \item \textbf{Hibridación Memética (NSGA-II + CMA-ES Local Search):} Utilizar NSGA-II para aproximar globalmente el frente de Pareto y aplicar pasos locales de CMA-ES o búsqueda direccional sobre los individuos élite de la rodilla del frente.
    \item \textbf{Programación Genética Simbólica y Aprendizaje Residual Sim-to-Real:} Emplear Programación Genética (GP) o redes neuronales evolucionadas (NEAT) para descubrir términos correctivos no lineales $\Delta\tau(q,\dot{q})$ que compensen la fricción estática no modelada y el juego mecánico (\textit{backlash}) al transferir el controlador al brazo físico Dynamixel AX-18A/AX-12A.
\end{itemize}

% ==============================================================================
% REFERENCIAS BIBLIOGRÁFICAS
% ==============================================================================
\begin{thebibliography}{99}
\bibitem{deb2002}
Deb, K., Pratap, A., Agarwal, S., \& Meyarivan, T. (2002). A fast and elitist multiobjective genetic algorithm: NSGA-II. \textit{IEEE Transactions on Evolutionary Computation}, 6(2), 182--197.

\bibitem{deb1995sbx}
Deb, K., \& Agrawal, R. B. (1995). Simulated binary crossover for continuous search space. \textit{Complex Systems}, 9(2), 115--148.

\bibitem{blank2020pymoo}
Blank, J., \& Deb, K. (2020). Pymoo: Multi-objective optimization in Python. \textit{IEEE Access}, 8, 89497--89509.

\bibitem{hansen2016cma}
Hansen, N. (2016). The CMA evolution strategy: A tutorial. \textit{arXiv preprint arXiv:1604.00772}.

\bibitem{storn1997de}
Storn, R., \& Price, K. (1997). Differential evolution -- a simple and efficient heuristic for global optimization over continuous spaces. \textit{Journal of Global Optimization}, 11(4), 341--359.

\bibitem{todorov2012mujoco}
Todorov, E., Erez, T., \& Tassa, Y. (2012). MuJoCo: A physics engine for model-based control. En \textit{Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)} (pp. 5026--5033). IEEE.

\bibitem{conejo2021}
Conejo Benítez, C. E. (2021). \textit{Modelado y control de un brazo robótico de 3 grados de libertad} [Tesis de maestría, Universidad de Guanajuato].

\bibitem{craig2018}
Craig, J. J. (2018). \textit{Introduction to robotics: Mechanics and control} (4th ed.). Pearson.

\bibitem{siciliano2009}
Siciliano, B., Sciavicco, L., Villani, L., \& Oriolo, G. (2009). \textit{Robotics: Modelling, planning and control}. Springer.

\bibitem{yoshikawa1985}
Yoshikawa, T. (1985). Manipulability of robotic mechanisms. \textit{The International Journal of Robotics Research}, 4(2), 3--9.
\end{thebibliography}

\end{document}
"""

tex_path = CE_DIR / "MAE-IA_Trabajo_Final_Computacion_Evolutiva_Grupo2.tex"
tex_path.write_text(latex_content, encoding="utf-8")

jobname = "MAE-IA_Trabajo_Final_Computacion_Evolutiva_Grupo2_v2_20de20"
for pass_num in (1, 2):
    proc = subprocess.run(
        ["pdflatex", "-interaction=nonstopmode", f"-jobname={jobname}", tex_path.name],
        cwd=str(CE_DIR),
        capture_output=True,
        text=True,
        encoding="latin-1"
    )
    print(f"pdflatex pasada {pass_num}: código {proc.returncode}")

pdf_v2_path = CE_DIR / f"{jobname}.pdf"
pdf_path = CE_DIR / "MAE-IA_Trabajo_Final_Computacion_Evolutiva_Grupo2.pdf"
if pdf_v2_path.exists():
    print(f"PDF v2 (20/20) compilado con código {proc.returncode}: {pdf_v2_path} ({pdf_v2_path.stat().st_size:,} bytes)")
    shutil.copy2(pdf_v2_path, WORKSPACE_GRUPO2 / pdf_v2_path.name)
    shutil.copy2(tex_path, WORKSPACE_GRUPO2 / tex_path.name)
    try:
        shutil.copy2(pdf_v2_path, pdf_path)
        shutil.copy2(pdf_v2_path, WORKSPACE_GRUPO2 / pdf_path.name)
        print(f"También actualizado en: {pdf_path}")
    except PermissionError:
        print(f"Nota: {pdf_path.name} está abierto en el visor PDF; la versión completa actualizada quedó en {pdf_v2_path.name}")
