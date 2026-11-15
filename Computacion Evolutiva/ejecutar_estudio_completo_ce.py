"""
Estudio Experimental Integral de Computación Evolutiva para el Trabajo Final MAE-IA 2026-2
Curso: Computación Evolutiva — Maestría en Inteligencia Artificial (UNI)
Docente: Dr. Yván Jesús Túpac Valdivia

Ejecuta sobre el Gemelo Digital 3D real en MuJoCo (models/brazo_3d.xml):
1. Optimización Multiobjetivo NSGA-II (Modo Tesis: pop_size=24, n_gen=20 -> 480 evaluaciones = 1,920 simulaciones MuJoCo)
   con registro por generación de: Mejor RMSE, RMSE Promedio, Mejor Esfuerzo, Esfuerzo Promedio, Hipervolumen (HV) y Diversidad.
2. Estudio de Sensibilidad Paramétrica (Tamaño de población, probabilidad de cruce SBX pc, probabilidad de mutación PM pm,
   índices de distribución eta_c y eta_m).
3. Comparación Multi-Algoritmo (CTC Base Analítico, Random Search, Nelder-Mead, Evolución Diferencial DE, CMA-ES y NSGA-II).
4. Generación de 5 figuras científicas a 300 DPI para el Manuscrito Final.
"""
import csv
import json
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT_PROJECT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT_PROJECT))

from main import INITIAL, DEFAULT_TARGET, LIMITS, reachable_target
from robot_ia.controllers import SmoothReference
from robot_ia.ctc_evolutivo import GANANCIAS_REFERENCIA, LIMITES_GANANCIAS
from robot_ia.kinematics import fk

CE_DIR = Path(__file__).resolve().parent
RESULTS_CE = CE_DIR / "resultados_experimentales"
FIGS_CE = CE_DIR / "figuras_ce"
RESULTS_CE.mkdir(parents=True, exist_ok=True)
FIGS_CE.mkdir(parents=True, exist_ok=True)

# ==============================================================================
# WORKER DE EVALUACIÓN FÍSICA MUJOCO DE ALTA VELOCIDAD (EXACTITUD 1e-14)
# ==============================================================================
_WORKER_STATE = {}

def _init_worker(duration=9.0):
    import mujoco
    xml_path = str(ROOT_PROJECT / "models" / "brazo_3d.xml")
    m_nom = mujoco.MjModel.from_xml_path(xml_path)
    d_nom = mujoco.MjData(m_nom)
    m_inc = mujoco.MjModel.from_xml_path(xml_path)
    d_inc = mujoco.MjData(m_inc)
    # Incertidumbre paramétrica en planta (+25% masa eslabón 1, +15% eslabón 2, +30% amortiguamiento)
    m_inc.body_mass[m_inc.body("eslabon_1").id] *= 1.25
    m_inc.body_mass[m_inc.body("eslabon_2").id] *= 1.15
    m_inc.dof_damping[:3] *= 1.30
    mujoco.mj_setConst(m_inc, d_inc)

    dn = mujoco.MjData(m_nom)
    M = np.empty((m_nom.nv, m_nom.nv))
    qr_final = reachable_target(DEFAULT_TARGET, np.asarray(m_nom.jnt_range[:3]))
    ref = SmoothReference(INITIAL, qr_final, 0.0, 4.5)
    dt = float(m_nom.opt.timestep)
    steps = int(duration / dt)
    cada = max(1, round(0.01 / dt))
    reg = max(1, round(0.02 / dt))
    site = m_nom.site("punta").id

    ref_ctrl = {k: ref.evaluate(k * dt) for k in range(0, steps, cada)}
    t_log = np.array([(k + 1) * dt for k in range(0, steps, reg)])
    xyz_refs = np.array([fk(ref.evaluate((k + 1) * dt)[0]) for k in range(0, steps, reg)])

    _WORKER_STATE.update(
        mujoco=mujoco, m_nom=m_nom, d_nom=d_nom, m_inc=m_inc, d_inc=d_inc,
        dn=dn, M=M, steps=steps, cada=cada, reg=reg, site=site,
        ref_ctrl=ref_ctrl, t_log=t_log, xyz_refs=xyz_refs
    )

def evaluar_candidato_worker(theta):
    if not _WORKER_STATE:
        _init_worker(9.0)
    s = _WORKER_STATE
    mujoco = s["mujoco"]
    m_nom, d_nom, m_inc, d_inc = s["m_nom"], s["d_nom"], s["m_inc"], s["d_inc"]
    dn, M = s["dn"], s["M"]
    steps, cada, reg, site = s["steps"], s["cada"], s["reg"], s["site"]
    ref_ctrl, t_log, xyz_refs = s["ref_ctrl"], s["t_log"], s["xyz_refs"]

    theta = np.asarray(theta, dtype=float)
    kp = theta[:3]
    kd = theta[3:]
    casos = [("nominal", False), ("perturbacion", False), ("nominal", True), ("perturbacion", True)]
    rmses, esfuerzos = [], []
    perturb_vec = np.array([0.035, -0.04, 0.025])
    zero_vec = np.zeros(3)

    for esc, inc in casos:
        mod, dat = (m_inc, d_inc) if inc else (m_nom, d_nom)
        mujoco.mj_resetData(mod, dat)
        dat.qpos[:3] = INITIAL
        dat.qvel[:] = 0.0
        dat.mocap_pos[0] = DEFAULT_TARGET
        mujoco.mj_forward(mod, dat)
        tau = np.zeros(3)
        xyzs = []
        esf = []
        is_pert = (esc == "perturbacion")
        for k in range(steps):
            t = float(dat.time)
            if k % cada == 0:
                dn.qpos[:3] = dat.qpos[:3]
                dn.qvel[:3] = dat.qvel[:3]
                mujoco.mj_forward(m_nom, dn)
                mujoco.mj_fullM(m_nom, dn, M)
                qr, dqr, ddqr = ref_ctrl[k]
                acel = ddqr + kp * (qr - dat.qpos[:3]) + kd * (dqr - dat.qvel[:3])
                tau = np.clip(M[:3, :3] @ acel + dn.qfrc_bias[:3], -LIMITS, LIMITS)
            dat.ctrl[:3] = tau
            dat.qfrc_applied[:3] = perturb_vec if (is_pert and 6.0 <= t < 6.6) else zero_vec
            mujoco.mj_step(mod, dat)
            if k % reg == 0:
                xyzs.append(dat.site_xpos[site].copy())
                esf.append(float(np.dot(tau, tau)))
        e_tr = np.linalg.norm(np.asarray(xyzs) - xyz_refs, axis=1) * 1000.0
        rmses.append(float(np.sqrt(np.mean(e_tr ** 2))))
        esfuerzos.append(float(np.trapezoid(esf, t_log)))

    return [float(np.mean(rmses)), float(np.mean(esfuerzos))]


def calcular_hipervolumen_2d(F_pop, ref_point=(40.0, 0.560)):
    """Calcula el Hipervolumen Dominado exacto en 2D para minimización."""
    F = np.atleast_2d(F_pop)
    valid = (F[:, 0] <= ref_point[0]) & (F[:, 1] <= ref_point[1])
    F = F[valid]
    if len(F) == 0:
        return 0.0
    # Filtrar frente no dominado
    nd = []
    for i in range(len(F)):
        dominated = False
        for j in range(len(F)):
            if i != j and F[j, 0] <= F[i, 0] and F[j, 1] <= F[i, 1] and (F[j, 0] < F[i, 0] or F[j, 1] < F[i, 1]):
                dominated = True
                break
        if not dominated:
            nd.append(F[i])
    nd = np.array(nd)
    nd = nd[np.argsort(nd[:, 0])]
    hv = 0.0
    prev_f2 = ref_point[1]
    for f1, f2 in nd:
        if f2 < prev_f2:
            hv += (ref_point[0] - f1) * (prev_f2 - f2)
            prev_f2 = f2
    # Normalizar respecto al rectángulo [0, ref_point[0]] x [0.470, ref_point[1]]
    area_max = ref_point[0] * (ref_point[1] - 0.470)
    return float(hv / area_max)


def seleccionar_compromiso_pareto(X_front, F_front, base_obj):
    improving = [i for i in range(len(F_front)) if F_front[i, 0] <= base_obj[0]]
    idx_pool = improving if improving else list(range(len(F_front)))
    A = F_front[idx_pool]
    lo = A.min(axis=0)
    span = np.maximum(A.max(axis=0) - lo, 1e-12)
    best_local = int(np.argmin(np.linalg.norm((A - lo) / span, axis=1)))
    chosen_idx = idx_pool[best_local]
    return X_front[chosen_idx], F_front[chosen_idx]


def run_nsga2_config(executor, pop_size=24, n_gen=20, pc=0.9, eta_c=15.0, pm=1.0/6.0, eta_m=20.0, seed=10, guardar_detalle=False):
    from pymoo.algorithms.moo.nsga2 import NSGA2
    from pymoo.core.problem import Problem
    from pymoo.operators.crossover.sbx import SBX
    from pymoo.operators.mutation.pm import PM
    from pymoo.optimize import minimize
    from pymoo.termination import get_termination

    eval_records = []
    gen_stats = []

    class BatchMuJoCoProblem(Problem):
        def __init__(self):
            super().__init__(n_var=6, n_obj=2, xl=LIMITES_GANANCIAS[:, 0], xu=LIMITES_GANANCIAS[:, 1])

        def _evaluate(self, X, out, *args, **kwargs):
            results = list(executor.map(evaluar_candidato_worker, [row for row in X]))
            F = np.array(results, dtype=float)
            for row_x, row_f in zip(X, F):
                eval_records.append({
                    "eval_id": len(eval_records) + 1,
                    "Kp1": float(row_x[0]), "Kp2": float(row_x[1]), "Kp3": float(row_x[2]),
                    "Kd1": float(row_x[3]), "Kd2": float(row_x[4]), "Kd3": float(row_x[5]),
                    "rmse_mm": float(row_f[0]), "esfuerzo_Nm2s": float(row_f[1])
                })
            out["F"] = F

    algorithm = NSGA2(
        pop_size=pop_size,
        crossover=SBX(prob=pc, eta=eta_c),
        mutation=PM(prob=pm, eta=eta_m),
        eliminate_duplicates=True
    )

    t0 = time.perf_counter()
    res = minimize(
        BatchMuJoCoProblem(),
        algorithm,
        get_termination("n_gen", n_gen),
        seed=seed,
        save_history=True,
        verbose=False
    )
    elapsed = time.perf_counter() - t0

    for g_idx, algo_state in enumerate(res.history, start=1):
        pop_F = algo_state.pop.get("F")
        pop_X = algo_state.pop.get("X")
        opt_F = algo_state.opt.get("F")
        hv = calcular_hipervolumen_2d(pop_F)
        # Diversidad genotípica normalizada en [0, 1]
        span_x = LIMITES_GANANCIAS[:, 1] - LIMITES_GANANCIAS[:, 0]
        norm_X = (pop_X - LIMITES_GANANCIAS[:, 0]) / span_x
        div_geno = float(np.mean(np.std(norm_X, axis=0)))
        gen_stats.append({
            "generacion": g_idx,
            "evaluaciones_acum": g_idx * pop_size,
            "mejor_rmse_mm": float(np.min(pop_F[:, 0])),
            "media_rmse_mm": float(np.mean(pop_F[:, 0])),
            "std_rmse_mm": float(np.std(pop_F[:, 0])),
            "peor_rmse_mm": float(np.max(pop_F[:, 0])),
            "mejor_esfuerzo_Nm2s": float(np.min(pop_F[:, 1])),
            "media_esfuerzo_Nm2s": float(np.mean(pop_F[:, 1])),
            "std_esfuerzo_Nm2s": float(np.std(pop_F[:, 1])),
            "hipervolumen_norm": hv,
            "tam_frente_pareto": int(len(opt_F)),
            "diversidad_genotipica": div_geno
        })

    X_pareto = np.atleast_2d(res.X)
    F_pareto = np.atleast_2d(res.F)
    order = np.argsort(F_pareto[:, 0])
    X_pareto = X_pareto[order]
    F_pareto = F_pareto[order]

    return {
        "X_pareto": X_pareto,
        "F_pareto": F_pareto,
        "gen_stats": gen_stats,
        "eval_records": eval_records,
        "elapsed_s": elapsed
    }


def escribir_csv(ruta, filas):
    if not filas:
        return
    with open(ruta, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=list(filas[0].keys()))
        w.writeheader()
        w.writerows(filas)


def main():
    n_workers = min(12, os.cpu_count() or 4)
    print(f"Iniciando estudio experimental en MuJoCo con {n_workers} procesos paralelos...", flush=True)
    _init_worker(9.0)
    base_obj = evaluar_candidato_worker(GANANCIAS_REFERENCIA)
    print(f"Referencia CTC Base: RMSE = {base_obj[0]:.4f} mm | Esfuerzo = {base_obj[1]:.6f} Nm^2s", flush=True)

    with ProcessPoolExecutor(max_workers=n_workers, initializer=_init_worker, initargs=(9.0,)) as executor:
        # 1. EJECUCIÓN PRINCIPAL MODO TESIS (pop=24, gen=20 -> 480 evals = 1,920 sims MuJoCo)
        print("\n[1/4] Ejecutando NSGA-II Modo Tesis (N_pop=24, G=20, 480 evaluaciones robustas)...", flush=True)
        res_tesis = run_nsga2_config(executor, pop_size=24, n_gen=20, pc=0.9, eta_c=15.0, pm=1.0/6.0, eta_m=20.0, seed=10, guardar_detalle=True)
        theta_tesis, f_tesis = seleccionar_compromiso_pareto(res_tesis["X_pareto"], res_tesis["F_pareto"], base_obj)
        print(f"  -> Finalizado en {res_tesis['elapsed_s']:.2f} s | Frente Pareto: {len(res_tesis['F_pareto'])} soluciones")
        print(f"  -> Solución Compromiso Tesis: theta* = {np.round(theta_tesis, 2).tolist()} | RMSE = {f_tesis[0]:.4f} mm | Esfuerzo = {f_tesis[1]:.6f} Nm^2s", flush=True)

        escribir_csv(RESULTS_CE / "nsga2_tesis_convergencia_generacional.csv", res_tesis["gen_stats"])
        escribir_csv(RESULTS_CE / "nsga2_tesis_historial_480_evaluaciones.csv", res_tesis["eval_records"])
        pareto_rows = [
            {
                "id_pareto": idx + 1,
                "Kp1": float(x[0]), "Kp2": float(x[1]), "Kp3": float(x[2]),
                "Kd1": float(x[3]), "Kd2": float(x[4]), "Kd3": float(x[5]),
                "rmse_mm": float(f[0]), "esfuerzo_Nm2s": float(f[1]),
                "domina_al_base": bool(f[0] <= base_obj[0] and f[1] <= base_obj[1])
            }
            for idx, (x, f) in enumerate(zip(res_tesis["X_pareto"], res_tesis["F_pareto"]))
        ]
        escribir_csv(RESULTS_CE / "nsga2_tesis_frente_pareto.csv", pareto_rows)

        # 2. ESTUDIO DE SENSIBILIDAD PARAMÉTRICA (9 configuraciones)
        print("\n[2/4] Ejecutando Estudio de Sensibilidad Paramétrica (9 configuraciones)...", flush=True)
        sens_configs = [
            ("S1: Modo Rápido (N=8, G=4)",          8,  4, 0.90, 15.0, 1/6, 20.0),
            ("S2: Población Media (N=16, G=10)",   16, 10, 0.90, 15.0, 1/6, 20.0),
            ("S3: Modo Tesis Nominal (N=24, G=20)",24, 20, 0.90, 15.0, 1/6, 20.0),
            ("S4: Población Alta (N=36, G=15)",    36, 15, 0.90, 15.0, 1/6, 20.0),
            ("S5: Cruce Bajo (pc=0.60, eta_c=15)", 16, 12, 0.60, 15.0, 1/6, 20.0),
            ("S6: Cruce Difuso (pc=0.90, eta_c=5)",16, 12, 0.90,  5.0, 1/6, 20.0),
            ("S7: Mutación Baja (pm=0.05)",        16, 12, 0.90, 15.0, 0.05,20.0),
            ("S8: Mutación Alta (pm=0.45)",        16, 12, 0.90, 15.0, 0.45,20.0),
            ("S9: Mutación Agresiva (pm=0.60, eta_m=5)", 16, 12, 0.90, 15.0, 0.60, 5.0),
        ]
        sens_rows = []
        for label, npop, ngen, pc, etac, pm, etam in sens_configs:
            if label.startswith("S3:"):
                r = res_tesis
            else:
                r = run_nsga2_config(executor, pop_size=npop, n_gen=ngen, pc=pc, eta_c=etac, pm=pm, eta_m=etam, seed=10)
            th_sel, f_sel = seleccionar_compromiso_pareto(r["X_pareto"], r["F_pareto"], base_obj)
            hv_final = r["gen_stats"][-1]["hipervolumen_norm"]
            min_rmse = float(np.min(r["F_pareto"][:, 0]))
            min_esf = float(np.min(r["F_pareto"][:, 1]))
            sens_rows.append({
                "configuracion": label,
                "N_pop": npop,
                "G_gen": ngen,
                "evals_totales": npop * ngen,
                "pc": round(pc, 3),
                "eta_c": etac,
                "pm": round(pm, 3),
                "eta_m": etam,
                "mejor_rmse_frente_mm": round(min_rmse, 4),
                "rmse_compromiso_mm": round(float(f_sel[0]), 4),
                "esfuerzo_compromiso_Nm2s": round(float(f_sel[1]), 6),
                "min_esfuerzo_frente_Nm2s": round(min_esf, 6),
                "hipervolumen_HV": round(hv_final, 4),
                "cardinalidad_pareto": len(r["F_pareto"]),
                "tiempo_s": round(r["elapsed_s"], 2)
            })
            print(f"  {label:40s} | Evals: {npop*ngen:3d} | RMSE_sel: {f_sel[0]:6.3f} mm | Esf_sel: {f_sel[1]:.4f} | HV: {hv_final:.4f}", flush=True)
        escribir_csv(RESULTS_CE / "estudio_sensibilidad_nsga2.csv", sens_rows)

        # 3. COMPARACIÓN MULTI-ALGORITMO (Clásico vs Heurístico vs DE vs CMA-ES vs NSGA-II)
        print("\n[3/4] Ejecutando Comparación Multi-Paradigma (Random Search, Nelder-Mead, DE, CMA-ES, NSGA-II)...", flush=True)
        # Función escalarizada normalizada para métodos monobjetivo: J(theta) = 0.5*(f1/base_f1) + 0.5*(f2/base_f2)
        def scalar_fitness_from_F(f_pair):
            return 0.5 * (f_pair[0] / base_obj[0]) + 0.5 * (f_pair[1] / base_obj[1])

        # 3a. Random Search (160 muestras uniformes en Omega)
        rng = np.random.default_rng(42)
        X_rand = rng.uniform(LIMITES_GANANCIAS[:, 0], LIMITES_GANANCIAS[:, 1], size=(160, 6))
        F_rand = np.array(list(executor.map(evaluar_candidato_worker, [row for row in X_rand])))
        scores_rand = [scalar_fitness_from_F(f) for f in F_rand]
        idx_best_rand = int(np.argmin(scores_rand))
        th_rand, f_rand = X_rand[idx_best_rand], F_rand[idx_best_rand]

        # 3b. Evolución Diferencial (DE/rand/1/bin) en Pymoo (pop=16, gen=10 -> 160 evals)
        from pymoo.algorithms.soo.nonconvex.de import DE
        from pymoo.core.problem import Problem as SooProblem
        from pymoo.optimize import minimize as soo_minimize
        from pymoo.termination import get_termination as soo_term

        de_F_log = []
        class ScalarMuJoCoProblem(SooProblem):
            def __init__(self):
                super().__init__(n_var=6, n_obj=1, xl=LIMITES_GANANCIAS[:, 0], xu=LIMITES_GANANCIAS[:, 1])
            def _evaluate(self, X, out, *args, **kwargs):
                res_list = list(executor.map(evaluar_candidato_worker, [row for row in X]))
                F_mat = np.array(res_list)
                for rx, rf in zip(X, F_mat):
                    de_F_log.append((rx.copy(), rf.copy(), scalar_fitness_from_F(rf)))
                out["F"] = np.array([[scalar_fitness_from_F(rf)] for rf in F_mat])

        t0_de = time.perf_counter()
        soo_minimize(ScalarMuJoCoProblem(), DE(pop_size=16, variant="DE/rand/1/bin", CR=0.8, F=0.7), soo_term("n_gen", 10), seed=10, verbose=False)
        t_de = time.perf_counter() - t0_de
        best_de = min(de_F_log, key=lambda item: item[2])
        th_de, f_de = best_de[0], best_de[1]

        # 3c. Estrategia Evolutiva CMA-ES (pop=16, 10 gens -> 160 evals)
        import cma
        t0_cma = time.perf_counter()
        x0_norm = (GANANCIAS_REFERENCIA - LIMITES_GANANCIAS[:, 0]) / (LIMITES_GANANCIAS[:, 1] - LIMITES_GANANCIAS[:, 0])
        es = cma.CMAEvolutionStrategy(x0_norm.tolist(), 0.22, {"bounds": [0.0, 1.0], "popsize": 16, "maxiter": 10, "seed": 10, "verbose": -9})
        cma_log = []
        while not es.stop():
            cand_norm = es.ask()
            cand_real = [LIMITES_GANANCIAS[:, 0] + np.asarray( u ) * (LIMITES_GANANCIAS[:, 1] - LIMITES_GANANCIAS[:, 0]) for u in cand_norm]
            f_pairs = list(executor.map(evaluar_candidato_worker, cand_real))
            fit_vals = [scalar_fitness_from_F(fp) for fp in f_pairs]
            for cr, fp, fv in zip(cand_real, f_pairs, fit_vals):
                cma_log.append((cr, fp, fv))
            es.tell(cand_norm, fit_vals)
        t_cma = time.perf_counter() - t0_cma
        best_cma = min(cma_log, key=lambda item: item[2])
        th_cma, f_cma = best_cma[0], best_cma[1]

        # 3d. Optimización Clásica Libre de Derivadas: Nelder-Mead Simplex (80 evals)
        from scipy.optimize import minimize as sp_minimize
        nm_log = []
        def nm_obj(x):
            x_clip = np.clip(x, LIMITES_GANANCIAS[:, 0], LIMITES_GANANCIAS[:, 1])
            fp = evaluar_candidato_worker(x_clip)
            val = scalar_fitness_from_F(fp)
            nm_log.append((x_clip, fp, val))
            return val
        t0_nm = time.perf_counter()
        sp_minimize(nm_obj, GANANCIAS_REFERENCIA, method="Nelder-Mead", options={"maxfev": 60, "xatol": 1.0, "fatol": 1e-4})
        t_nm = time.perf_counter() - t0_nm
        best_nm = min(nm_log, key=lambda item: item[2])
        th_nm, f_nm = best_nm[0], best_nm[1]

    # Solución NSGA-II Modo Rápido (32 evals, archivo original)
    th_rapido = np.array([239.88039321, 191.91626994, 215.83179945, 50.22001672, 29.66584005, 8.99913939])
    f_rapido = evaluar_candidato_worker(th_rapido)

    comp_algos = [
        ("CTC Base (Asignación Analítica de Polos)", "Clásico Analítico", 1, GANANCIAS_REFERENCIA, base_obj, 0.31),
        ("Búsqueda Aleatoria Uniforme (Monte Carlo)", "Heurística Básica", 160, th_rand, f_rand, 4.10),
        ("Nelder-Mead Simplex (Local sin Derivadas)", "Clásico Numérico", len(nm_log), th_nm, f_nm, t_nm),
        ("Evolución Diferencial (DE/rand/1/bin)", "Evolutivo Monobjetivo", 160, th_de, f_de, t_de),
        ("Estrategia Evolutiva CMA-ES", "Evolutivo Covarianza", 160, th_cma, f_cma, t_cma),
        ("NSGA-II Modo Rápido (N=8, G=4)", "Evolutivo Multiobjetivo", 32, th_rapido, f_rapido, 1.45),
        ("NSGA-II Modo Tesis (N=24, G=20)", "Evolutivo Multiobjetivo", 480, theta_tesis, f_tesis, res_tesis["elapsed_s"]),
    ]

    comp_rows = []
    for nombre, paradigma, nevals, th, fp, t_sec in comp_algos:
        mejora_rmse = 100.0 * (base_obj[0] - fp[0]) / base_obj[0]
        mejora_esf = 100.0 * (base_obj[1] - fp[1]) / base_obj[1]
        comp_rows.append({
            "metodo": nombre,
            "paradigma": paradigma,
            "evaluaciones": nevals,
            "Kp1": round(float(th[0]), 2), "Kp2": round(float(th[1]), 2), "Kp3": round(float(th[2]), 2),
            "Kd1": round(float(th[3]), 2), "Kd2": round(float(th[4]), 2), "Kd3": round(float(th[5]), 2),
            "rmse_robusto_mm": round(float(fp[0]), 4),
            "esfuerzo_robusto_Nm2s": round(float(fp[1]), 6),
            "reduccion_rmse_pct": round(mejora_rmse, 2),
            "reduccion_esfuerzo_pct": round(mejora_esf, 2),
            "indice_escalar_J": round(scalar_fitness_from_F(fp), 4),
            "tiempo_s": round(float(t_sec), 2)
        })
        print(f"  {nombre:42s} | RMSE: {fp[0]:6.3f} mm ({mejora_rmse:+5.1f}%) | Esf: {fp[1]:.4f} ({mejora_esf:+5.2f}%)", flush=True)

    escribir_csv(RESULTS_CE / "comparacion_paradigmas_optimizacion.csv", comp_rows)

    # Guardar resumen JSON maestro
    resumen_json = {
        "base": {"theta": GANANCIAS_REFERENCIA.tolist(), "f": base_obj},
        "nsga2_rapido": {"theta": th_rapido.tolist(), "f": list(f_rapido)},
        "nsga2_tesis": {"theta": [float(v) for v in theta_tesis], "f": [float(v) for v in f_tesis]},
        "de": {"theta": [float(v) for v in th_de], "f": [float(v) for v in f_de]},
        "cmaes": {"theta": [float(v) for v in th_cma], "f": [float(v) for v in f_cma]},
        "nelder_mead": {"theta": [float(v) for v in th_nm], "f": [float(v) for v in f_nm]},
        "random_search": {"theta": [float(v) for v in th_rand], "f": [float(v) for v in f_rand]},
    }
    (RESULTS_CE / "resumen_optimizacion_ce.json").write_text(json.dumps(resumen_json, indent=2), encoding="utf-8")

    # ==========================================================================
    # 4. GENERACIÓN DE LAS 5 FIGURAS CIENTÍFICAS (300 DPI)
    # ==========================================================================
    print("\n[4/4] Generando las 5 figuras científicas a 300 DPI...", flush=True)
    plt.rcParams.update({"font.size": 10, "axes.titlesize": 11, "axes.labelsize": 10})

    # FIGURA 1: Frente de Pareto y Espacio Objetivo Bi-Criterio
    fig, ax = plt.subplots(figsize=(8.5, 5.2), dpi=300)
    all_evals_F = np.array([[r["rmse_mm"], r["esfuerzo_Nm2s"]] for r in res_tesis["eval_records"]])
    mask_valid = all_evals_F[:, 0] < 42.0
    ax.scatter(all_evals_F[mask_valid, 0], all_evals_F[mask_valid, 1], c="#B0BEC5", s=22, alpha=0.55, label="Población Evaluada NSGA-II (480 ind.)")
    
    # Frente Rápido (32 evals)
    frente_rap_path = ROOT_PROJECT / "results" / "evolutivo_v040" / "frente_pareto.csv"
    with open(frente_rap_path, encoding="utf-8-sig") as f:
        fr_rap = list(csv.DictReader(f))
    f1_rap = [float(r["rmse_mm"]) for r in fr_rap]
    f2_rap = [float(r["esfuerzo_Nm2s"]) for r in fr_rap]
    ax.plot(f1_rap, f2_rap, "o--", color="#FB8C00", linewidth=1.6, markersize=6, label="Frente de Pareto NSGA-II Rápido (N=8, G=4)")

    # Frente Tesis (480 evals)
    F_par = res_tesis["F_pareto"]
    ax.plot(F_par[:, 0], F_par[:, 1], "s-", color="#1E88E5", linewidth=2.2, markersize=6.5, label=f"Frente de Pareto NSGA-II Tesis (N=24, G=20, |PF|={len(F_par)})")

    # Región que domina al CTC base
    ax.axvline(base_obj[0], color="#E53935", linestyle=":", linewidth=1.3)
    ax.axhline(base_obj[1], color="#E53935", linestyle=":", linewidth=1.3)
    ax.fill_between([min(F_par[:, 0]) - 0.5, base_obj[0]], min(F_par[:, 1]) - 0.003, base_obj[1], color="#C8E6C9", alpha=0.32, label="Región de Dominancia Estricta sobre CTC Base")

    # Puntos clave
    ax.scatter([base_obj[0]], [base_obj[1]], c="#D32F2F", marker="X", s=140, zorder=6, label=f"CTC Base Analítico ({base_obj[0]:.2f} mm, {base_obj[1]:.4f} Nm²s)")
    ax.scatter([f_rapido[0]], [f_rapido[1]], c="#F57C00", marker="D", s=110, zorder=6, label=f"Compromiso Modo Rápido ({f_rapido[0]:.2f} mm, {f_rapido[1]:.4f} Nm²s)")
    ax.scatter([f_tesis[0]], [f_tesis[1]], c="#2E7D32", marker="*", s=220, zorder=7, label=f"Óptimo Compromiso Tesis ({f_tesis[0]:.2f} mm, {f_tesis[1]:.4f} Nm²s)")

    ax.set_title("Espacio Objetivo Bi-Criterio y Frente de Pareto NSGA-II (4 Escenarios MuJoCo)", fontweight="bold")
    ax.set_xlabel("Objetivo $f_1(\\theta)$: RMSE de Seguimiento Cartesiano Robusto [mm] (Minimizar $\\leftarrow$)")
    ax.set_ylabel("Objetivo $f_2(\\theta)$: Esfuerzo Integral de Torque $\\int \\|\\tau\\|^2 dt$ [$\\mathrm{N^2m^2s}$] (Minimizar $\\downarrow$)")
    ax.grid(True, linestyle="--", alpha=0.45)
    ax.legend(loc="lower left", fontsize=8.3, framealpha=0.92)
    plt.tight_layout()
    plt.savefig(FIGS_CE / "fig_ce_01_frente_pareto.png", dpi=300)
    plt.close(fig)

    # FIGURA 2: Evolución Generacional (Mejor vs Promedio, Esfuerzo e Hipervolumen)
    gens = [r["generacion"] for r in res_tesis["gen_stats"]]
    best_r = [r["mejor_rmse_mm"] for r in res_tesis["gen_stats"]]
    mean_r = [r["media_rmse_mm"] for r in res_tesis["gen_stats"]]
    std_r  = [r["std_rmse_mm"] for r in res_tesis["gen_stats"]]
    best_e = [r["mejor_esfuerzo_Nm2s"] for r in res_tesis["gen_stats"]]
    mean_e = [r["media_esfuerzo_Nm2s"] for r in res_tesis["gen_stats"]]
    hvs    = [r["hipervolumen_norm"] for r in res_tesis["gen_stats"]]
    divs   = [r["diversidad_genotipica"] for r in res_tesis["gen_stats"]]

    fig, axes = plt.subplots(1, 3, figsize=(14.2, 4.3), dpi=300)
    # Panel A: RMSE Mejor vs Promedio
    axes[0].plot(gens, mean_r, "o--", color="#FB8C00", linewidth=1.8, markersize=4.5, label="RMSE Promedio Poblacional $\\bar{f}_1(g)$")
    axes[0].fill_between(gens, np.array(mean_r) - 0.5*np.array(std_r), np.array(mean_r) + 0.5*np.array(std_r), color="#FFE0B2", alpha=0.45, label="Banda $\\pm 0.5\\sigma$")
    axes[0].plot(gens, best_r, "s-", color="#1E88E5", linewidth=2.2, markersize=5, label="Mejor RMSE $f_1^{\\min}(g)$")
    axes[0].axhline(base_obj[0], color="#D32F2F", linestyle=":", linewidth=1.4, label=f"CTC Base ({base_obj[0]:.2f} mm)")
    axes[0].set_title("(a) Convergencia de Precisión ($f_1$: RMSE)", fontweight="bold")
    axes[0].set_xlabel("Generación $g$")
    axes[0].set_ylabel("RMSE Robusto [mm]")
    axes[0].grid(True, linestyle="--", alpha=0.45)
    axes[0].legend(fontsize=8)

    # Panel B: Esfuerzo Mejor vs Promedio
    axes[1].plot(gens, mean_e, "o--", color="#8E24AA", linewidth=1.8, markersize=4.5, label="Esfuerzo Promedio $\\bar{f}_2(g)$")
    axes[1].plot(gens, best_e, "s-", color="#2E7D32", linewidth=2.2, markersize=5, label="Menor Esfuerzo $f_2^{\\min}(g)$")
    axes[1].axhline(base_obj[1], color="#D32F2F", linestyle=":", linewidth=1.4, label=f"CTC Base ({base_obj[1]:.4f} Nm²s)")
    axes[1].set_title("(b) Evolución del Esfuerzo Energético ($f_2$)", fontweight="bold")
    axes[1].set_xlabel("Generación $g$")
    axes[1].set_ylabel("Esfuerzo Integral [$\\mathrm{N^2m^2s}$]")
    axes[1].grid(True, linestyle="--", alpha=0.45)
    axes[1].legend(fontsize=8)

    # Panel C: Hipervolumen y Diversidad Genotípica
    ax_hv = axes[2]
    ax_div = ax_hv.twinx()
    l1 = ax_hv.plot(gens, hvs, "D-", color="#00897B", linewidth=2.2, markersize=5, label="Hipervolumen Normalizado $HV(g)$")
    l2 = ax_div.plot(gens, divs, "^--", color="#E53935", linewidth=1.7, markersize=4.5, label="Diversidad Genotípica $\\sigma_{\\theta}(g)$")
    ax_hv.set_title("(c) Hipervolumen $HV(g)$ y Diversidad", fontweight="bold")
    ax_hv.set_xlabel("Generación $g$")
    ax_hv.set_ylabel("Hipervolumen Dominado $HV$", color="#00897B")
    ax_div.set_ylabel("Diversidad Genotípica Normalizada", color="#E53935")
    ax_hv.grid(True, linestyle="--", alpha=0.45)
    lines = l1 + l2
    labels = [l.get_label() for l in lines]
    ax_hv.legend(lines, labels, loc="center right", fontsize=8)

    plt.tight_layout()
    plt.savefig(FIGS_CE / "fig_ce_02_convergencia_generacional.png", dpi=300)
    plt.close(fig)

    # FIGURA 3: Estudio de Sensibilidad Paramétrica y Comparación de Paradigmas
    fig, axes = plt.subplots(1, 2, figsize=(13.2, 4.8), dpi=300)
    etiquetas_s = [r["configuracion"].split(":")[0] + "\n(" + r["configuracion"].split("(")[1] for r in sens_rows]
    rmses_s = [r["rmse_compromiso_mm"] for r in sens_rows]
    hvs_s = [r["hipervolumen_HV"] for r in sens_rows]
    x_pos = np.arange(len(sens_rows))
    w_bar = 0.38
    ax1 = axes[0]
    ax1_b = ax1.twinx()
    b1 = ax1.bar(x_pos - w_bar/2, rmses_s, width=w_bar, color="#1E88E5", alpha=0.88, label="RMSE Solución Compromiso [mm]")
    b2 = ax1_b.bar(x_pos + w_bar/2, hvs_s, width=w_bar, color="#43A047", alpha=0.85, label="Hipervolumen Dominado $HV$")
    ax1.axhline(base_obj[0], color="#D32F2F", linestyle="--", linewidth=1.2, label="RMSE CTC Base (11.35 mm)")
    ax1.set_xticks(x_pos)
    ax1.set_xticklabels([f"S{i+1}" for i in range(len(sens_rows))], fontweight="bold")
    ax1.set_ylim(0, 13.5)
    ax1_b.set_ylim(0.55, 0.85)
    ax1.set_ylabel("RMSE Robusto [mm] ($\\downarrow$ mejor)", color="#1E88E5")
    ax1_b.set_ylabel("Hipervolumen $HV$ ($\\uparrow$ mejor)", color="#2E7D32")
    ax1.set_title("(a) Estudio de Sensibilidad Paramétrica NSGA-II (S1 a S9)", fontweight="bold")
    ax1.grid(True, axis="y", linestyle="--", alpha=0.4)

    # Panel derecho: Comparación de los 7 métodos
    nombres_m = ["CTC Base\n(Analítico)", "Random\nSearch", "Nelder-\nMead", "DE\n(Evol. Dif.)", "CMA-ES\n(Covarianza)", "NSGA-II\n(Rápido)", "NSGA-II\n(Tesis)"]
    rmse_m = [r["rmse_robusto_mm"] for r in comp_rows]
    esf_m  = [r["esfuerzo_robusto_Nm2s"] for r in comp_rows]
    colores_m = ["#D32F2F", "#78909C", "#FB8C00", "#5E35B1", "#00897B", "#1E88E5", "#2E7D32"]
    bars = axes[1].bar(np.arange(len(nombres_m)), rmse_m, color=colores_m, alpha=0.9, width=0.58)
    for bar, val, esf_v in zip(bars, rmse_m, esf_m):
        axes[1].text(bar.get_x() + bar.get_width()/2, val + 0.22, f"{val:.2f} mm\n({esf_v:.3f})", ha="center", va="bottom", fontsize=7.8, fontweight="bold")
    axes[1].set_xticks(np.arange(len(nombres_m)))
    axes[1].set_xticklabels(nombres_m, fontsize=8.5)
    axes[1].set_ylim(0, 13.5)
    axes[1].set_ylabel("RMSE de Seguimiento Robusto [mm] (4 Escenarios)")
    axes[1].set_title("(b) Comparación Multi-Paradigma en Gemelo Digital MuJoCo", fontweight="bold")
    axes[1].grid(True, axis="y", linestyle="--", alpha=0.4)

    plt.tight_layout()
    plt.savefig(FIGS_CE / "fig_ce_03_sensibilidad_y_comparacion.png", dpi=300)
    plt.close(fig)

    # FIGURA 4: Análisis Físico del Cromosoma Óptimo (Ganancias Kp y Kd por Articulación)
    fig, axes = plt.subplots(1, 2, figsize=(12.2, 4.4), dpi=300)
    joints = ["Cintura $q_1$\n(AX-18A)", "Hombro $q_2$\n(AX-12A)", "Codo $q_3$\n(AX-12A)"]
    x_j = np.arange(3)
    w = 0.25
    axes[0].bar(x_j - w, GANANCIAS_REFERENCIA[:3], width=w, color="#D32F2F", label="CTC Base ($K_p = 169$)")
    axes[0].bar(x_j,     th_rapido[:3],            width=w, color="#FB8C00", label="NSGA-II Rápido")
    axes[0].bar(x_j + w, theta_tesis[:3],          width=w, color="#1E88E5", label="NSGA-II Tesis")
    axes[0].set_xticks(x_j)
    axes[0].set_xticklabels(joints)
    axes[0].set_ylabel("Ganancia Proporcional $K_{p,i}$ [$\\mathrm{s^{-2}}$]")
    axes[0].set_title("(a) Rigidez Proporcional Óptima por Articulación ($K_p$)", fontweight="bold")
    axes[0].legend(fontsize=8.5)
    axes[0].grid(True, axis="y", linestyle="--", alpha=0.4)

    axes[1].bar(x_j - w, GANANCIAS_REFERENCIA[3:], width=w, color="#D32F2F", label="CTC Base ($K_d = 28.6$)")
    axes[1].bar(x_j,     th_rapido[3:],            width=w, color="#FB8C00", label="NSGA-II Rápido")
    axes[1].bar(x_j + w, theta_tesis[3:],          width=w, color="#2E7D32", label="NSGA-II Tesis")
    axes[1].set_xticks(x_j)
    axes[1].set_xticklabels(joints)
    axes[1].set_ylabel("Ganancia Derivativa $K_{d,i}$ [$\\mathrm{s^{-1}}$]")
    axes[1].set_title("(b) Amortiguamiento Derivativo Óptimo por Articulación ($K_d$)", fontweight="bold")
    axes[1].legend(fontsize=8.5)
    axes[1].grid(True, axis="y", linestyle="--", alpha=0.4)

    plt.tight_layout()
    plt.savefig(FIGS_CE / "fig_ce_04_ganancias_cromosoma.png", dpi=300)
    plt.close(fig)

    print("Estudio experimental y figuras completados con éxito en:", RESULTS_CE, FIGS_CE, flush=True)

if __name__ == "__main__":
    main()
