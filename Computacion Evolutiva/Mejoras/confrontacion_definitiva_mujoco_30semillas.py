#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
CONFRONTACION_DEFINITIVA_MUJOCO_30SEMILLAS.PY
================================================================================
Unificación Suprema de las Mejoras 1 (mejoras_evolutivas_ctc.py + Auditoría
Adversarial Kimi/Argon/3ra Opinión) y Mejora 2 (Proyecto MATLAB Extendido:
Gain-Scheduling, Sustituto RBF con criterio LIE, Test de Friedman/Wilcoxon-Holm)
ejecutada directamente sobre el GEMELO DIGITAL 3D REAL EN MUJOCO.

Resuelve y demuestra experimentalmente:
  1. Corrección del Bug de Semilla Cruzada (seed=0 vs seed=42) de
     mejoras_evolutivas_ctc.py usando Números Aleatorios Comunes (CRN).
  2. Gemelo Digital 3D MuJoCo Hostil:
       - Ruido gaussiano en encoders: sigma_q = 0.002 rad, sigma_qd = 0.02 rad/s
       - Retardo de transporte en lazo cerrado: 1 paso de control (10 ms)
       - Mini-batch espacial de trayectorias 3D por Latin Hypercube Sampling (LHS)
       - 4 escenarios dinámicos (nominal, perturbación 3D, masa +25%/+15% y
         fricción +30%, combinado)
       - Agregación robusta tipo H_inf: 0.5 * media + 0.5 * peor_caso
       - Tercer objetivo anti-chattering: f3 = int ||d(tau)/dt||^2 dt [N^2 m^2 / s]
  3. Reparametrización Modal Física:
       theta_modal = [wn1, wn2, wn3, z1, z2, z3] con z_i in [0.35, 2.00]
  4. Ley de Ganancia Programada (Gain-Scheduled CTC, 12 genes) + Sustituto RBF
     con adquisición LIE + Refinamiento Memético Local (Nelder-Mead).
  5. Experimento de Confrontación en N = 30 Semillas Independientes con
     Test de Friedman y Test de Rangos con Signo de Wilcoxon + Holm-Bonferroni,
     enfrentando:
       (C1) CTC Base Analítico (wn=13, z=1.10)
       (C2) theta*_ideal de Tabla 2 (z3=0.133, entrenado sin ruido) -> Prueba
            experimental de la hipótesis de la 3ra Opinión sobre chattering f3
       (C3) NSGA-II Directo [Kp, Kd] evolucionado en Gemelo Hostil
       (C4) NSGA-II Modal Tri-Objetivo [wn, z] evolucionado en Gemelo Hostil
       (C5) NSGA-II Gain-Scheduled Memético + RBF (Unificación Mejora 1 + 2)
================================================================================
"""

from __future__ import annotations

import csv
import os
import sys
import json
import time
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from scipy.stats import wilcoxon, friedmanchisquare, rankdata
import mujoco
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
CE_DIR = Path(__file__).resolve().parent.parent
MEJORAS_DIR = Path(__file__).resolve().parent
FIG_DIR = CE_DIR / "figuras_ce"
RES_DIR = CE_DIR / "resultados_experimentales"
FIG_DIR.mkdir(parents=True, exist_ok=True)
RES_DIR.mkdir(parents=True, exist_ok=True)

if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from main import INITIAL, DEFAULT_TARGET, LIMITS, reachable_target
from robot_ia.controllers import SmoothReference
from robot_ia.kinematics import fk

# ==============================================================================
# PARÁMETROS DEL GEMELO HOSTIL MUJOCO 3D Y ESPACIO MODAL
# ==============================================================================
XML_PATH = PROJECT_ROOT / "models" / "brazo_3d.xml"
DT_SIM = 0.002
DT_CTRL = 0.010
TAU_MAX = np.asarray(LIMITS, dtype=float)
Q_INICIO = np.asarray(INITIAL, dtype=float)
T_EPISODIO = 6.0          # Duración por episodio en lote [s]
SIGMA_Q = 0.002           # Ruido encoder posición [rad] (~0.115 deg)
SIGMA_QD = 0.020          # Ruido velocidad articular [rad/s]
RETARDO_PASOS = 1         # Retardo de transporte: 1 ciclo de control (10 ms)
LAMBDA_RIESGO = 0.5       # Agregación H_inf: 0.5 * media + 0.5 * max

KP_MIN, KP_MAX = 20.0, 250.0
KD_MIN, KD_MAX = 3.0, 55.0
WN_MIN, WN_MAX = float(np.sqrt(KP_MIN)), float(np.sqrt(KP_MAX))  # [4.472, 15.811]
Z_MIN, Z_MAX = 0.35, 2.00

ESCENARIOS_NOMBRES = ("nominal", "perturbado", "incierto", "combinado")


# ==============================================================================
# MUESTREO LHS DE TRAYECTORIAS 3D Y SEMILLAS DETERMINISTAS (CRN)
# ==============================================================================
def generar_lote_lhs_trayectorias(n_tray: int, seed_lote: int):
    """
    Genera n_tray pares (q0, qf) mediante Latin Hypercube Sampling estratificado
    incluyendo siempre en el índice 0 el corredor nominal de enceste y en los
    demás índices trayectorias espaciales diversas (alta extensión, plegado y
    cambio de codo) dentro del espacio de trabajo libre de colisiones.
    """
    rng = np.random.default_rng(seed_lote)
    q_meta_nom = reachable_target(DEFAULT_TARGET, np.array([[-2.8, 2.8], [-1.25, 1.35], [-2.4, 2.4]]))
    pares = [(Q_INICIO.copy(), q_meta_nom.copy())]
    if n_tray <= 1:
        return pares

    q0_lo = np.array([-1.40, -0.55, -1.55], dtype=float)
    q0_hi = np.array([+1.40, +0.35, -0.35], dtype=float)
    qf_lo = np.array([-1.40, -0.25, -1.45], dtype=float)
    qf_hi = np.array([+1.40, +0.75, +1.05], dtype=float)

    m = n_tray - 1
    u0 = np.empty((m, 3), dtype=float)
    uf = np.empty((m, 3), dtype=float)
    for j in range(3):
        p0 = rng.permutation(m)
        pf = rng.permutation(m)
        u0[:, j] = (p0 + rng.random(m)) / m
        uf[:, j] = (pf + rng.random(m)) / m

    Q0 = q0_lo + u0 * (q0_hi - q0_lo)
    QF = qf_lo + uf * (qf_hi - qf_lo)
    for i in range(m):
        pares.append((Q0[i].copy(), QF[i].copy()))
    return pares


# ==============================================================================
# WORKER MULTIPROCESO CON MUJOCO 3D PRE-CARGADO EN RAM
# ==============================================================================
_W_MODELS = {}
_W_DATAS = {}
_W_NOM_MODEL = None
_W_NOM_DATA = None
_W_SITE_ID = None


def _init_mujoco_hostile_worker():
    global _W_MODELS, _W_DATAS, _W_NOM_MODEL, _W_NOM_DATA, _W_SITE_ID
    _W_NOM_MODEL = mujoco.MjModel.from_xml_path(str(XML_PATH))
    _W_NOM_DATA = mujoco.MjData(_W_NOM_MODEL)
    _W_SITE_ID = _W_NOM_MODEL.site("punta").id
    for esc in ESCENARIOS_NOMBRES:
        m = mujoco.MjModel.from_xml_path(str(XML_PATH))
        d = mujoco.MjData(m)
        if esc in ("incierto", "combinado"):
            m.body_mass[m.body("eslabon_1").id] *= 1.25
            m.body_mass[m.body("eslabon_2").id] *= 1.15
            m.dof_damping[:3] *= 1.30
            mujoco.mj_setConst(m, d)
        _W_MODELS[esc] = m
        _W_DATAS[esc] = d


def simular_episodio_mujoco_hostil(
    feno: dict,
    q0: np.ndarray,
    qf: np.ndarray,
    esc: str,
    seed_ruido: int,
    hostil: bool = True,
    duracion: float = T_EPISODIO,
):
    """
    Ejecuta 1 episodio en el Gemelo Digital 3D MuJoCo:
      - Si hostil=True: inyecta ruido gaussiano en sensores (SIGMA_Q, SIGMA_QD)
        con semilla común seed_ruido (CRN) y retardo de transporte de 1 paso
        de control (10 ms).
      - Soporta fenotipo estático (Kp, Kd fijos) o Gain-Scheduled (wn(e)).
      - Devuelve (rmse_mm, energia_tau2, tasa_par_dtau2, sat_pct).
    """
    global _W_MODELS, _W_DATAS, _W_NOM_MODEL, _W_NOM_DATA, _W_SITE_ID
    if _W_NOM_MODEL is None:
        _init_mujoco_hostile_worker()

    model = _W_MODELS[esc]
    data = _W_DATAS[esc]
    nominal = _W_NOM_MODEL
    dn = _W_NOM_DATA

    mujoco.mj_resetData(model, data)
    mujoco.mj_resetData(nominal, dn)
    data.qpos[:3] = q0
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)

    traj = SmoothReference(q0, qf, 0.0, 0.65 * duracion)
    pasos_ctrl = max(1, int(round(DT_CTRL / DT_SIM)))
    n_steps = int(round(duracion / DT_SIM))

    rng = np.random.default_rng(seed_ruido)

    n_delay = (RETARDO_PASOS + 1) if hostil else 1
    buf_q = [q0.copy() for _ in range(n_delay)]
    buf_dq = [np.zeros(3, dtype=float) for _ in range(n_delay)]

    M_full = np.zeros((nominal.nv, nominal.nv), dtype=float)
    tau_cmd = np.zeros(3, dtype=float)
    tau_prev = np.zeros(3, dtype=float)

    err2_sum = 0.0
    n_err = 0
    energia_tau2 = 0.0
    tasa_par_dtau2 = 0.0
    sat_count = 0
    n_ctrl_steps = 0

    t_pert_ini = 0.67 * duracion
    t_pert_fin = t_pert_ini + 0.50
    perturb_vec = np.array([0.035, -0.040, 0.025], dtype=float)
    zero_vec = np.zeros(3, dtype=float)

    for k in range(n_steps):
        t = k * DT_SIM

        if k % pasos_ctrl == 0:
            if hostil:
                q_meas = data.qpos[:3].copy() + rng.normal(0.0, SIGMA_Q, 3)
                dq_meas = data.qvel[:3].copy() + rng.normal(0.0, SIGMA_QD, 3)
            else:
                q_meas = data.qpos[:3].copy()
                dq_meas = data.qvel[:3].copy()

            buf_q.append(q_meas)
            buf_dq.append(dq_meas)
            q_s = buf_q.pop(0)
            dq_s = buf_dq.pop(0)

            qr, dqr, ddqr = traj.evaluate(t)
            e = qr - q_s
            ed = dqr - dq_s

            if feno["modo"] == "estatico":
                Kp = feno["Kp"]
                Kd = feno["Kd"]
            else:
                wn_e = np.minimum(
                    feno["wn0"] + feno["Dwn"] * np.exp(-np.abs(e) / feno["sigma"]),
                    WN_MAX
                )
                Kp = wn_e ** 2
                Kd = 2.0 * feno["z"] * wn_e

            dn.qpos[:3] = q_s
            dn.qvel[:3] = dq_s
            mujoco.mj_forward(nominal, dn)
            mujoco.mj_fullM(nominal, dn, M_full)
            M_hat = M_full[:3, :3]
            bias_hat = dn.qfrc_bias[:3].copy()

            v = ddqr + Kp * e + Kd * ed
            tau_raw = M_hat @ v + bias_hat
            tau_cmd = np.clip(tau_raw, -TAU_MAX, TAU_MAX)

            if n_ctrl_steps > 0:
                dtau_dt = (tau_cmd - tau_prev) / DT_CTRL
                tasa_par_dtau2 += float(np.dot(dtau_dt, dtau_dt)) * DT_CTRL
            tau_prev = tau_cmd.copy()

            if np.any(np.abs(tau_cmd) >= 0.995 * TAU_MAX):
                sat_count += 1
            n_ctrl_steps += 1

        data.ctrl[:3] = tau_cmd
        if esc in ("perturbado", "combinado") and (t_pert_ini <= t <= t_pert_fin):
            data.qfrc_applied[:3] = perturb_vec
        else:
            data.qfrc_applied[:3] = zero_vec

        mujoco.mj_step(model, data)
        energia_tau2 += float(np.dot(tau_cmd, tau_cmd)) * DT_SIM

        if not (np.all(np.isfinite(data.qpos)) and np.all(np.isfinite(data.qvel))):
            return 1e8, 1e8, 1e8, 100.0

        if k % (2 * pasos_ctrl) == 0:
            p_real = data.site_xpos[_W_SITE_ID].copy()
            qr_next, _, _ = traj.evaluate((k + 1) * DT_SIM)
            p_ref = fk(qr_next)
            diff_mm = (p_real - p_ref) * 1000.0
            err2_sum += float(np.dot(diff_mm, diff_mm))
            n_err += 1

    rmse_mm = float(np.sqrt(err2_sum / max(n_err, 1)))
    sat_pct = 100.0 * sat_count / max(n_ctrl_steps, 1)
    return rmse_mm, float(energia_tau2), float(tasa_par_dtau2), float(sat_pct)


def decodificar_fenotipo(x: np.ndarray, tipo_genotipo: str) -> dict:
    x = np.asarray(x, dtype=float)
    if tipo_genotipo == "directo":
        Kp = x[:3]
        Kd = x[3:6]
        wn = np.sqrt(np.maximum(Kp, 1e-6))
        z = Kd / (2.0 * wn)
        return {"modo": "estatico", "Kp": Kp, "Kd": Kd, "wn": wn, "z": z}
    elif tipo_genotipo == "modal":
        wn = x[:3]
        z = x[3:6]
        Kp = wn ** 2
        Kd = 2.0 * z * wn
        return {"modo": "estatico", "Kp": Kp, "Kd": Kd, "wn": wn, "z": z}
    elif tipo_genotipo == "programado":
        g = x.reshape(3, 4)
        wn0 = g[:, 0]
        Dwn = g[:, 1]
        z = g[:, 2]
        sigma = np.maximum(g[:, 3], 0.02)
        return {
            "modo": "programado",
            "wn0": wn0,
            "Dwn": Dwn,
            "z": z,
            "sigma": sigma,
            "Kp": (wn0 + Dwn) ** 2,
            "Kd": 2.0 * z * (wn0 + Dwn),
        }
    else:
        raise ValueError(f"Tipo de genotipo desconocido: {tipo_genotipo}")


def _eval_candidato_worker(args):
    """Función top-level para evaluación multiproceso en MuJoCo."""
    x, tipo_genotipo, lote_tray, seed_base, hostil, lambda_riesgo = args
    feno = decodificar_fenotipo(x, tipo_genotipo)
    f1s, f2s, f3s, sats = [], [], [], []
    idx_ep = 0
    for (q0, qf) in lote_tray:
        for esc in ESCENARIOS_NOMBRES:
            # Semilla CRN determinista por (seed_base, episodio)
            seed_ep = int(seed_base * 1000 + idx_ep * 17 + 7)
            r, en, tp, sat = simular_episodio_mujoco_hostil(
                feno, q0, qf, esc, seed_ruido=seed_ep, hostil=hostil
            )
            f1s.append(r)
            f2s.append(en)
            f3s.append(tp)
            sats.append(sat)
            idx_ep += 1

    f1s = np.asarray(f1s, dtype=float)
    f2s = np.asarray(f2s, dtype=float)
    f3s = np.asarray(f3s, dtype=float)

    f1_rob = (1.0 - lambda_riesgo) * float(np.mean(f1s)) + lambda_riesgo * float(np.max(f1s))
    f2_rob = (1.0 - lambda_riesgo) * float(np.mean(f2s)) + lambda_riesgo * float(np.max(f2s))
    f3_rob = (1.0 - lambda_riesgo) * float(np.mean(f3s)) + lambda_riesgo * float(np.max(f3s))
    return np.array([f1_rob, f2_rob, f3_rob, float(np.mean(sats))], dtype=float)


# ==============================================================================
# OPERADORES NSGA-II TRI-OBJETIVO + SUSTITUTO RBF (MEJORA 2) + MEMÉTICO LOCAL
# ==============================================================================
def fast_non_dominated_sort(F: np.ndarray):
    N = len(F)
    S = [[] for _ in range(N)]
    n_dom = np.zeros(N, dtype=int)
    frentes = [[]]
    for p in range(N):
        for q_ in range(N):
            if p == q_:
                continue
            if np.all(F[p] <= F[q_]) and np.any(F[p] < F[q_]):
                S[p].append(q_)
            elif np.all(F[q_] <= F[p]) and np.any(F[q_] < F[p]):
                n_dom[p] += 1
        if n_dom[p] == 0:
            frentes[0].append(p)
    i = 0
    while frentes[i]:
        nxt = []
        for p in frentes[i]:
            for q_ in S[p]:
                n_dom[q_] -= 1
                if n_dom[q_] == 0:
                    nxt.append(q_)
        i += 1
        frentes.append(nxt)
    frentes.pop()
    return frentes


def crowding_distance(F: np.ndarray, idx):
    m = F.shape[1]
    d = np.zeros(len(idx), dtype=float)
    if len(idx) <= 2:
        d[:] = np.inf
        return d
    for j in range(m):
        orden = np.argsort(F[idx, j])
        d[orden[0]] = d[orden[-1]] = np.inf
        rango = F[idx[orden[-1]], j] - F[idx[orden[0]], j]
        if rango < 1e-14:
            continue
        for k in range(1, len(idx) - 1):
            d[orden[k]] += (F[idx[orden[k + 1]], j] - F[idx[orden[k - 1]], j]) / rango
    return d


def torneo_binario(rango, crowd, rng):
    a, b = rng.integers(0, len(rango), 2)
    if rango[a] < rango[b]:
        return a
    if rango[b] < rango[a]:
        return b
    return a if crowd[a] >= crowd[b] else b


def sbx_acotado(p1, p2, lo, hi, pc, eta_c, rng):
    c1, c2 = p1.copy(), p2.copy()
    if rng.random() > pc:
        return c1, c2
    for j in range(len(p1)):
        if rng.random() > 0.5 or abs(p1[j] - p2[j]) < 1e-14:
            continue
        y1, y2 = min(p1[j], p2[j]), max(p1[j], p2[j])
        rand = rng.random()
        beta = 1.0 + 2.0 * (y1 - lo[j]) / (y2 - y1)
        alpha = 2.0 - beta ** -(eta_c + 1.0)
        betaq = (rand * alpha) ** (1.0 / (eta_c + 1.0)) if rand <= 1.0 / alpha \
            else (1.0 / (2.0 - rand * alpha)) ** (1.0 / (eta_c + 1.0))
        c1[j] = 0.5 * ((y1 + y2) - betaq * (y2 - y1))

        beta = 1.0 + 2.0 * (hi[j] - y2) / (y2 - y1)
        alpha = 2.0 - beta ** -(eta_c + 1.0)
        betaq = (rand * alpha) ** (1.0 / (eta_c + 1.0)) if rand <= 1.0 / alpha \
            else (1.0 / (2.0 - rand * alpha)) ** (1.0 / (eta_c + 1.0))
        c2[j] = 0.5 * ((y1 + y2) + betaq * (y2 - y1))
    return np.clip(c1, lo, hi), np.clip(c2, lo, hi)


def mutacion_polinomial(x, lo, hi, pm, eta_m, rng):
    y = x.copy()
    for j in range(len(x)):
        if rng.random() > pm:
            continue
        d1 = (x[j] - lo[j]) / (hi[j] - lo[j])
        d2 = (hi[j] - x[j]) / (hi[j] - lo[j])
        rand = rng.random()
        pot = 1.0 / (eta_m + 1.0)
        if rand < 0.5:
            xy = 1.0 - d1
            val = 2.0 * rand + (1.0 - 2.0 * rand) * (xy ** (eta_m + 1.0))
            dq = val ** pot - 1.0
        else:
            xy = 1.0 - d2
            val = 2.0 * (1.0 - rand) + 2.0 * (rand - 0.5) * (xy ** (eta_m + 1.0))
            dq = 1.0 - val ** pot
        y[j] = np.clip(x[j] + dq * (hi[j] - lo[j]), lo[j], hi[j])
    return y


class SurrogateRBFEnsemble:
    """
    Modelo Sustituto de Base Radial (RBF Gaussiano con regularización Ridge
    e incertidumbre epistémica basada en distancia al archivo), inspirado en
    SurrogateManager.m / RBFNet.m de la Mejora 2, para pre-filtrar candidatos
    mediante Lower Confidence Bound (LIE).
    """
    def __init__(self, lo: np.ndarray, hi: np.ndarray, reg: float = 1e-3):
        self.lo = lo
        self.hi = hi
        self.reg = reg
        self.X_db = []
        self.Y_db = []
        self.W = None
        self.C = None
        self.gamma = 1.0

    def add_batch(self, X: np.ndarray, Y: np.ndarray):
        for x, y in zip(X, Y):
            if np.all(np.isfinite(y)) and y[0] < 1e6:
                self.X_db.append(x.copy())
                self.Y_db.append(y[:3].copy())
        if len(self.X_db) >= 8:
            self._fit()

    def _norm(self, X: np.ndarray) -> np.ndarray:
        return (X - self.lo) / np.maximum(self.hi - self.lo, 1e-12)

    def _fit(self):
        Xn = self._norm(np.asarray(self.X_db))
        Y = np.asarray(self.Y_db)
        # Usar hasta los últimos 160 puntos como centros RBF
        if len(Xn) > 160:
            Xn = Xn[-160:]
            Y = Y[-160:]
        self.C = Xn
        d2 = np.sum((Xn[:, None, :] - Xn[None, :, :]) ** 2, axis=2)
        med_d2 = max(float(np.median(d2[d2 > 0])), 0.05) if np.any(d2 > 0) else 0.25
        self.gamma = 1.0 / (2.0 * med_d2)
        Phi = np.exp(-self.gamma * d2)
        Phi_aug = np.hstack([Phi, np.ones((len(Xn), 1))])
        A = Phi_aug.T @ Phi_aug + self.reg * np.eye(Phi_aug.shape[1])
        self.W = np.linalg.solve(A, Phi_aug.T @ Y)

    def predict_lcb(self, X: np.ndarray, kappa: float = 0.65) -> np.ndarray:
        if self.W is None or self.C is None:
            return np.zeros((len(X), 3))
        Xn = self._norm(np.asarray(X))
        d2 = np.sum((Xn[:, None, :] - self.C[None, :, :]) ** 2, axis=2)
        Phi = np.exp(-self.gamma * d2)
        Phi_aug = np.hstack([Phi, np.ones((len(Xn), 1))])
        Y_hat = Phi_aug @ self.W
        # Incertidumbre epistémica: 1 - max activación RBF
        sigma_epist = np.sqrt(np.clip(1.0 - np.max(Phi, axis=1, keepdims=True), 0.0, 1.0))
        std_y = np.std(np.asarray(self.Y_db), axis=0, keepdims=True) + 1e-9
        return Y_hat - kappa * sigma_epist * std_y


def hipervolumen_2d_norm(F_2d: np.ndarray, ref: np.ndarray, ideal: np.ndarray) -> float:
    pts = np.asarray(F_2d, dtype=float)
    mask = np.all(pts <= ref, axis=1)
    pts = pts[mask]
    if len(pts) == 0:
        return 0.0
    frentes = fast_non_dominated_sort(pts)
    nd = pts[frentes[0]]
    nd = nd[np.argsort(nd[:, 0])]
    hv = 0.0
    x_prev = ref[0]
    for x, y in reversed(nd):
        if x < x_prev and y < ref[1]:
            hv += (x_prev - x) * (ref[1] - y)
            x_prev = x
    area_max = float((ref[0] - ideal[0]) * (ref[1] - ideal[1]))
    return float(hv / max(area_max, 1e-12))


def calcular_igd_plus(F_aprox: np.ndarray, F_ref: np.ndarray) -> float:
    """
    Calcula la Distancia Generacional Invertida Modificada (IGD+, Ishibuchi 2015),
    débilmente Pareto-compatible en R^3, normalizada por el rango de F_ref.
    """
    lo = F_ref.min(axis=0)
    hi = F_ref.max(axis=0)
    rango = np.maximum(hi - lo, 1e-12)
    R = (F_ref - lo) / rango
    A = (F_aprox - lo) / rango
    dists = []
    for r in R:
        # d+(r, a) = || max(a - r, 0) ||_2
        diff_pos = np.maximum(A - r[None, :], 0.0)
        d_min = np.min(np.linalg.norm(diff_pos, axis=1))
        dists.append(d_min)
    return float(np.mean(dists))


def selector_post_pareto_3d(F: np.ndarray, X: np.ndarray, f_base: np.ndarray):
    """
    Selecciona la solución compromiso del frente de Pareto en R^3 dentro del
    cono de dominancia F_dom = {theta in F1 | F(theta) <= F(theta_base)},
    normalizando [z_ideal, z_nadir] estrictamente sobre F_dom (Ecuación 12).
    """
    cono_estricto = np.all(F[:, :3] <= f_base[:3], axis=1)
    if np.any(cono_estricto):
        idx = np.where(cono_estricto)[0]
    else:
        cono_f1 = F[:, 0] <= f_base[0]
        idx = np.where(cono_f1)[0] if np.any(cono_f1) else np.arange(len(F))

    F_sub = F[idx, :3]
    z_ideal = F_sub.min(axis=0)
    z_nadir = F_sub.max(axis=0)
    rango = np.maximum(z_nadir - z_ideal, 1e-12)
    Fn = (F_sub - z_ideal) / rango

    pesos = np.array([0.75, 0.10, 0.15])
    d = np.sqrt(np.sum(pesos * (Fn ** 2), axis=1))
    mejor = idx[int(np.argmin(d))]
    return X[mejor].copy(), F[mejor].copy(), int(mejor)


def evolucionar_nsga2_mujoco(
    pool: ProcessPoolExecutor,
    tipo_genotipo: str,
    n_pop: int,
    n_gen: int,
    lote_tray: list,
    seed_evol: int,
    seed_eval_crn: int,
    hostil: bool = True,
    usar_surrogate_rbf: bool = False,
    usar_memetico: bool = False,
):
    rng = np.random.default_rng(seed_evol)
    if tipo_genotipo == "directo":
        lo = np.array([KP_MIN] * 3 + [KD_MIN] * 3, dtype=float)
        hi = np.array([KP_MAX] * 3 + [KD_MAX] * 3, dtype=float)
    elif tipo_genotipo == "modal":
        lo = np.array([WN_MIN] * 3 + [Z_MIN] * 3, dtype=float)
        hi = np.array([WN_MAX] * 3 + [Z_MAX] * 3, dtype=float)
    elif tipo_genotipo == "programado":
        lo = np.array([WN_MIN, 0.0, Z_MIN, 0.05] * 3, dtype=float)
        hi = np.array([WN_MAX, 4.0, Z_MAX, 0.65] * 3, dtype=float)
    else:
        raise ValueError(tipo_genotipo)

    D = len(lo)
    X = lo + rng.random((n_pop, D)) * (hi - lo)

    # Inyectar semilla analítica equivalente en individuo 0 para garantizar piso
    if tipo_genotipo == "directo":
        X[0] = np.array([169.0, 169.0, 169.0, 28.6, 28.6, 28.6])
    elif tipo_genotipo == "modal":
        X[0] = np.array([13.0, 13.0, 13.0, 1.10, 1.10, 1.10])
    elif tipo_genotipo == "programado":
        X[0] = np.array([12.5, 2.5, 1.05, 0.18, 12.8, 2.2, 0.85, 0.18, 13.5, 2.2, 0.95, 0.18])
        if n_pop > 1:
            X[1] = np.array([13.0, 0.0, 1.10, 0.25] * 3)

    def eval_poblacion(P_mat):
        tareas = [
            (ind, tipo_genotipo, lote_tray, seed_eval_crn, hostil, LAMBDA_RIESGO)
            for ind in P_mat
        ]
        res = list(pool.map(_eval_candidato_worker, tareas))
        return np.asarray(res, dtype=float)

    F_full = eval_poblacion(X)
    F = F_full[:, :3]

    surrogate = SurrogateRBFEnsemble(lo, hi) if usar_surrogate_rbf else None
    if surrogate is not None:
        surrogate.add_batch(X, F)

    ref_hv = np.array([45.0, 0.45])
    ideal_hv = np.array([5.0, 0.20])
    hist_hv = []
    hist_best_f1 = []
    hist_best_f3 = []

    for g in range(n_gen):
        frentes = fast_non_dominated_sort(F)
        rango = np.empty(n_pop, dtype=int)
        crowd = np.zeros(n_pop, dtype=float)
        for r, fr in enumerate(frentes):
            rango[fr] = r
            crowd[fr] = crowding_distance(F, fr)

        # Si usa Surrogate RBF (Mejora 2), genera 2*n_pop candidatos y filtra los mejores n_pop por LCB
        n_cand = (2 * n_pop) if (surrogate is not None and surrogate.W is not None) else n_pop
        cand_X = []
        while len(cand_X) < n_cand:
            p1 = X[torneo_binario(rango, crowd, rng)]
            p2 = X[torneo_binario(rango, crowd, rng)]
            c1, c2 = sbx_acotado(p1, p2, lo, hi, 0.90, 15.0, rng)
            for c in (c1, c2):
                if len(cand_X) < n_cand:
                    cand_X.append(mutacion_polinomial(c, lo, hi, 1.0 / D, 20.0, rng))
        cand_X = np.asarray(cand_X, dtype=float)

        if surrogate is not None and surrogate.W is not None and len(cand_X) > n_pop:
            F_lcb = surrogate.predict_lcb(cand_X, kappa=0.65)
            frentes_s = fast_non_dominated_sort(F_lcb)
            sel = []
            for fr in frentes_s:
                if len(sel) + len(fr) <= n_pop:
                    sel.extend(fr)
                else:
                    d = crowding_distance(F_lcb, fr)
                    sel.extend([fr[i] for i in np.argsort(-d)[: n_pop - len(sel)]])
                    break
            hijos_X = cand_X[sel]
        else:
            hijos_X = cand_X[:n_pop]

        hijos_F_full = eval_poblacion(hijos_X)
        hijos_F = hijos_F_full[:, :3]
        if surrogate is not None:
            surrogate.add_batch(hijos_X, hijos_F)

        RX = np.vstack([X, hijos_X])
        RF_full = np.vstack([F_full, hijos_F_full])
        RF = RF_full[:, :3]

        frentes_R = fast_non_dominated_sort(RF)
        keep = []
        for fr in frentes_R:
            if len(keep) + len(fr) <= n_pop:
                keep.extend(fr)
            else:
                d = crowding_distance(RF, fr)
                orden = np.argsort(-d)
                keep.extend([fr[i] for i in orden[: n_pop - len(keep)]])
                break

        X = RX[keep]
        F_full = RF_full[keep]
        F = F_full[:, :3]

        hv = hipervolumen_2d_norm(F[:, :2], ref_hv, ideal_hv)
        hist_hv.append(hv)
        hist_best_f1.append(float(F[:, 0].min()))
        hist_best_f3.append(float(F[:, 2].min()))

    # Búsqueda Local Memética sobre el mejor individuo compromiso (Paso de refinamiento direccional)
    if usar_memetico:
        idx_best = int(np.argmin(F[:, 0] / np.median(F[:, 0]) + 0.25 * F[:, 1] / np.median(F[:, 1]) + 0.25 * F[:, 2] / np.median(F[:, 2])))
        x0 = X[idx_best].copy()
        deltas = 0.04 * (hi - lo)
        cand_local = []
        for j in range(D):
            xp = np.clip(x0.copy(), lo, hi)
            xm = np.clip(x0.copy(), lo, hi)
            xp[j] = min(hi[j], x0[j] + deltas[j])
            xm[j] = max(lo[j], x0[j] - deltas[j])
            cand_local.extend([xp, xm])
        cand_local = np.asarray(cand_local, dtype=float)
        F_loc_full = eval_poblacion(cand_local)
        RX = np.vstack([X, cand_local])
        RF_full = np.vstack([F_full, F_loc_full])
        RF = RF_full[:, :3]
        frentes_R = fast_non_dominated_sort(RF)
        keep = []
        for fr in frentes_R:
            if len(keep) + len(fr) <= n_pop:
                keep.extend(fr)
            else:
                d = crowding_distance(RF, fr)
                orden = np.argsort(-d)
                keep.extend([fr[i] for i in orden[: n_pop - len(keep)]])
                break
        X = RX[keep]
        F_full = RF_full[keep]
        F = F_full[:, :3]

    frente1 = fast_non_dominated_sort(F)[0]
    return X[frente1], F_full[frente1], hist_hv, hist_best_f1, hist_best_f3, X, F_full


def holm_bonferroni_adjust(p_vals: dict) -> dict:
    """Corrección de Holm-Bonferroni para comparaciones múltiples."""
    items = sorted(p_vals.items(), key=lambda kv: kv[1])
    m = len(items)
    adj = {}
    prev = 0.0
    for i, (k, p) in enumerate(items):
        p_corr = min(1.0, max(prev, (m - i) * p))
        prev = p_corr
        adj[k] = p_corr
    return adj


def main():
    t_global = time.perf_counter()
    print("=" * 84)
    print(" EJECUTANDO UNIFICACIÓN SUPREMA EN MUJOCO 3D (MEJORA 1 + MEJORA 2 + 30 SEMILLAS)")
    print("=" * 84)

    # Ganancias fijas a confrontar:
    # 1. CTC Base Analítico (wn = 13, z = 1.10)
    theta_base = np.array([169.0, 169.0, 169.0, 28.6, 28.6, 28.6], dtype=float)
    # 2. Ganador Idealizado de Tabla 2 (entrenado sin ruido, z3 = 0.133, Kd3 = 4.20)
    theta_ideal_tabla2 = np.array([203.94, 195.39, 249.58, 49.65, 17.33, 4.20], dtype=float)

    lote_entrenamiento = generar_lote_lhs_trayectorias(n_tray=2, seed_lote=42)

    with ProcessPoolExecutor(max_workers=min(12, os.cpu_count() or 8), initializer=_init_mujoco_hostile_worker) as pool:
        # ------------------------------------------------------------------
        # FASE 1: Demostración del Bug seed=0 vs seed=42 y Evolución en Gemelo Hostil MuJoCo
        # ------------------------------------------------------------------
        print("\n[1/3] Evaluando CTC Base y Ganador Idealizado (Tabla 2) en Gemelo MuJoCo Hostil...")
        F_base_hostil = _eval_candidato_worker(
            (theta_base, "directo", lote_entrenamiento, 42, True, LAMBDA_RIESGO)
        )
        F_ideal_en_hostil = _eval_candidato_worker(
            (theta_ideal_tabla2, "directo", lote_entrenamiento, 42, True, LAMBDA_RIESGO)
        )
        print(f"  -> CTC Base Analítico (z=1.10) en Gemelo Hostil: "
              f"f1={F_base_hostil[0]:.3f} mm | f2={F_base_hostil[1]:.4f} | f3(chattering)={F_base_hostil[2]:.3f}")
        print(f"  -> Ganador Idealizado Tabla 2 (z3=0.133) en Gemelo Hostil: "
              f"f1={F_ideal_en_hostil[0]:.3f} mm | f2={F_ideal_en_hostil[1]:.4f} | f3(chattering)={F_ideal_en_hostil[2]:.3f}")

        print("\n[2/3] Evolucionando 3 Arquitecturas Evolutivas sobre el Gemelo Digital 3D MuJoCo Hostil...")
        t0 = time.perf_counter()
        X_dir, F_dir, hv_dir, bf1_dir, bf3_dir, popX_dir, popF_dir = evolucionar_nsga2_mujoco(
            pool, "directo", n_pop=24, n_gen=15, lote_tray=lote_entrenamiento,
            seed_evol=42, seed_eval_crn=42, hostil=True, usar_surrogate_rbf=False, usar_memetico=False
        )
        print(f"  [a] NSGA-II Directo [Kp,Kd] completado en {time.perf_counter()-t0:.2f} s (|F1|={len(F_dir)})")

        t0 = time.perf_counter()
        X_mod, F_mod, hv_mod, bf1_mod, bf3_mod, popX_mod, popF_mod = evolucionar_nsga2_mujoco(
            pool, "modal", n_pop=24, n_gen=15, lote_tray=lote_entrenamiento,
            seed_evol=42, seed_eval_crn=42, hostil=True, usar_surrogate_rbf=False, usar_memetico=True
        )
        print(f"  [b] NSGA-II Modal Reparametrizado [wn,z] completado en {time.perf_counter()-t0:.2f} s (|F1|={len(F_mod)})")

        t0 = time.perf_counter()
        X_gs, F_gs, hv_gs, bf1_gs, bf3_gs, popX_gs, popF_gs = evolucionar_nsga2_mujoco(
            pool, "programado", n_pop=24, n_gen=15, lote_tray=lote_entrenamiento,
            seed_evol=42, seed_eval_crn=42, hostil=True, usar_surrogate_rbf=True, usar_memetico=True
        )
        print(f"  [c] NSGA-II Gain-Scheduled + RBF + Memético (12 genes) completado en {time.perf_counter()-t0:.2f} s (|F1|={len(F_gs)})")

        theta_dir_star, F_dir_star, _ = selector_post_pareto_3d(F_dir[:, :3], X_dir, F_base_hostil[:3])
        theta_mod_star, F_mod_star, _ = selector_post_pareto_3d(F_mod[:, :3], X_mod, F_base_hostil[:3])
        theta_gs_star, F_gs_star, _ = selector_post_pareto_3d(F_gs[:, :3], X_gs, F_base_hostil[:3])

        feno_mod_star = decodificar_fenotipo(theta_mod_star, "modal")
        feno_gs_star = decodificar_fenotipo(theta_gs_star, "programado")

        # Frente de referencia combinado para IGD+
        F_union = np.vstack([F_dir[:, :3], F_mod[:, :3], F_gs[:, :3]])
        frente_ref_idx = fast_non_dominated_sort(F_union)[0]
        F_ref_igd = F_union[frente_ref_idx]

        igd_dir = calcular_igd_plus(F_dir[:, :3], F_ref_igd)
        igd_mod = calcular_igd_plus(F_mod[:, :3], F_ref_igd)
        igd_gs = calcular_igd_plus(F_gs[:, :3], F_ref_igd)

        print(f"\n  -> Solución Compromiso Modal [wn, z]:")
        print(f"     wn = {np.round(feno_mod_star['wn'], 2)} rad/s | zeta = {np.round(feno_mod_star['z'], 3)}")
        print(f"     Kp = {np.round(feno_mod_star['Kp'], 2)} s^-2  | Kd   = {np.round(feno_mod_star['Kd'], 2)} s^-1")
        print(f"     F* = [f1={F_mod_star[0]:.3f} mm, f2={F_mod_star[1]:.4f}, f3={F_mod_star[2]:.3f}] | IGD+={igd_mod:.4f}")
        print(f"  -> Solución Compromiso Gain-Scheduled + RBF + Memético:")
        print(f"     F* = [f1={F_gs_star[0]:.3f} mm, f2={F_gs_star[1]:.4f}, f3={F_gs_star[2]:.3f}] | IGD+={igd_gs:.4f}")

        # ------------------------------------------------------------------
        # FASE 2: PROTOCOLO ESTADÍSTICO DE 30 SEMILLAS INDEPENDIENTES (CRN)
        #         SOBRE EL GEMELO DIGITAL 3D MUJOCO HOSTIL
        # ------------------------------------------------------------------
        print("\n[3/3] Ejecutando Confrontación Estadística de N = 30 Semillas Independientes en MuJoCo 3D...")
        N_SEEDS = 30
        registros_30 = []

        # En cada semilla s = 1..30 se genera un lote LHS espacial distinto de trayectorias 3D
        # y realizaciones independientes de ruido de encoder (CRN entre los 5 controladores)
        tareas_30 = []
        meta_tareas = []
        for s in range(1, N_SEEDS + 1):
            lote_s = generar_lote_lhs_trayectorias(n_tray=3, seed_lote=1000 + s)
            candidatos_s = [
                ("C1_CTC_Base", theta_base, "directo"),
                ("C2_Ideal_Tabla2", theta_ideal_tabla2, "directo"),
                ("C3_NSGA2_Directo_Hostil", theta_dir_star, "directo"),
                ("C4_NSGA2_Modal_TriObj", theta_mod_star, "modal"),
                ("C5_NSGA2_GainSched_RBF", theta_gs_star, "programado"),
            ]
            for nombre_c, th_c, tipo_c in candidatos_s:
                tareas_30.append((th_c, tipo_c, lote_s, 5000 + s, True, LAMBDA_RIESGO))
                meta_tareas.append((s, nombre_c))

        t0_30 = time.perf_counter()
        resultados_30 = list(pool.map(_eval_candidato_worker, tareas_30))
        print(f"  -> 30 semillas x 5 controladores x 12 episodios MuJoCo (1,800 simulaciones 3D) "
              f"completadas en {time.perf_counter()-t0_30:.2f} s!")

        for (s, nombre_c), vec_f in zip(meta_tareas, resultados_30):
            registros_30.append({
                "seed": s,
                "controlador": nombre_c,
                "f1_rmse_rob_mm": float(vec_f[0]),
                "f2_energia_rob": float(vec_f[1]),
                "f3_chattering_rob": float(vec_f[2]),
                "sat_pct": float(vec_f[3]),
            })

    with open(RES_DIR / "confrontacion_30semillas_mujoco_hostil.csv", "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(registros_30[0].keys()))
        writer.writeheader()
        writer.writerows(registros_30)

    # Guardar frentes de Pareto del gemelo hostil
    orden_mod = np.argsort(F_mod[:, 0])
    cols_mod = ["wn1", "wn2", "wn3", "zeta1", "zeta2", "zeta3",
                "Kp1", "Kp2", "Kp3", "Kd1", "Kd2", "Kd3",
                "f1_rmse_rob_mm", "f2_energia_rob", "f3_chattering_rob", "sat_pct"]
    mat_mod = np.hstack([
        X_mod,
        np.array([decodificar_fenotipo(x, "modal")["Kp"] for x in X_mod]),
        np.array([decodificar_fenotipo(x, "modal")["Kd"] for x in X_mod]),
        F_mod,
    ])[orden_mod]
    with open(RES_DIR / "frente_pareto_modal_hostil_mujoco.csv", "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(cols_mod)
        for row in mat_mod:
            writer.writerow([float(v) for v in row])

    # ------------------------------------------------------------------
    # ANÁLISIS ESTADÍSTICO NO PARAMÉTRICO: MEDIANA, IQR, FRIEDMAN Y WILCOXON-HOLM
    # ------------------------------------------------------------------
    orden_ctrl = [
        "C1_CTC_Base",
        "C2_Ideal_Tabla2",
        "C3_NSGA2_Directo_Hostil",
        "C4_NSGA2_Modal_TriObj",
        "C5_NSGA2_GainSched_RBF",
    ]

    pivot_f1 = {c: np.zeros(N_SEEDS, dtype=float) for c in orden_ctrl}
    pivot_f2 = {c: np.zeros(N_SEEDS, dtype=float) for c in orden_ctrl}
    pivot_f3 = {c: np.zeros(N_SEEDS, dtype=float) for c in orden_ctrl}
    for row in registros_30:
        idx_s = int(row["seed"]) - 1
        c = row["controlador"]
        pivot_f1[c][idx_s] = row["f1_rmse_rob_mm"]
        pivot_f2[c][idx_s] = row["f2_energia_rob"]
        pivot_f3[c][idx_s] = row["f3_chattering_rob"]

    # Test global de Friedman sobre los 5 métodos (N=30 bloques)
    stat_friedman_f1, p_friedman_f1 = friedmanchisquare(*[pivot_f1[c] for c in orden_ctrl])
    stat_friedman_f3, p_friedman_f3 = friedmanchisquare(*[pivot_f3[c] for c in orden_ctrl])

    # Rangos medios de Friedman por fila (semilla)
    mat_f1 = np.column_stack([pivot_f1[c] for c in orden_ctrl])
    mat_f3 = np.column_stack([pivot_f3[c] for c in orden_ctrl])
    ranks_f1_mat = np.array([rankdata(row) for row in mat_f1])
    ranks_f3_mat = np.array([rankdata(row) for row in mat_f3])
    rangos_f1 = {c: float(ranks_f1_mat[:, i].mean()) for i, c in enumerate(orden_ctrl)}
    rangos_f3 = {c: float(ranks_f3_mat[:, i].mean()) for i, c in enumerate(orden_ctrl)}

    # Wilcoxon pareado contra C1_CTC_Base y contra C2_Ideal_Tabla2
    p_raw_vs_base_f1 = {}
    p_raw_vs_ideal_f3 = {}
    w_stat_vs_base_f1 = {}
    for c in orden_ctrl:
        if c != "C1_CTC_Base":
            res_w = wilcoxon(pivot_f1[c], pivot_f1["C1_CTC_Base"], alternative="two-sided")
            p_raw_vs_base_f1[c] = float(res_w.pvalue)
            w_stat_vs_base_f1[c] = float(res_w.statistic)
        if c != "C2_Ideal_Tabla2":
            res_w3 = wilcoxon(pivot_f3[c], pivot_f3["C2_Ideal_Tabla2"], alternative="two-sided")
            p_raw_vs_ideal_f3[c] = float(res_w3.pvalue)

    p_holm_vs_base_f1 = holm_bonferroni_adjust(p_raw_vs_base_f1)
    p_holm_vs_ideal_f3 = holm_bonferroni_adjust(p_raw_vs_ideal_f3)

    resumen_estadistico = []
    for c in orden_ctrl:
        v1 = pivot_f1[c]
        v2 = pivot_f2[c]
        v3 = pivot_f3[c]
        resumen_estadistico.append({
            "controlador": c,
            "f1_mediana_mm": float(np.median(v1)),
            "f1_iqr_mm": float(np.percentile(v1, 75) - np.percentile(v1, 25)),
            "f1_q25_mm": float(np.percentile(v1, 25)),
            "f1_q75_mm": float(np.percentile(v1, 75)),
            "f1_media_mm": float(np.mean(v1)),
            "f1_std_mm": float(np.std(v1, ddof=1)),
            "f2_mediana": float(np.median(v2)),
            "f2_iqr": float(np.percentile(v2, 75) - np.percentile(v2, 25)),
            "f3_chattering_mediana": float(np.median(v3)),
            "f3_chattering_iqr": float(np.percentile(v3, 75) - np.percentile(v3, 25)),
            "rango_friedman_f1": float(rangos_f1[c]),
            "rango_friedman_f3": float(rangos_f3[c]),
            "wilcoxon_W_vs_base_f1": float(w_stat_vs_base_f1.get(c, -1.0)),
            "p_raw_vs_base_f1": float(p_raw_vs_base_f1.get(c, 1.0)),
            "p_holm_vs_base_f1": float(p_holm_vs_base_f1.get(c, 1.0)),
            "p_holm_vs_ideal_f3": float(p_holm_vs_ideal_f3.get(c, 1.0)),
        })

    with open(RES_DIR / "resumen_estadistico_30semillas_wilcoxon_holm.csv", "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(resumen_estadistico[0].keys()))
        writer.writeheader()
        writer.writerows(resumen_estadistico)

    print("\n================ TABLA ESTADÍSTICA (N = 30 SEMILLAS, MUJOCO 3D HOSTIL) ================")
    for r in resumen_estadistico:
        print(f"  {r['controlador']:25s} | f1_med={r['f1_mediana_mm']:7.3f} (IQR={r['f1_iqr_mm']:5.3f}) mm | "
              f"f2_med={r['f2_mediana']:6.4f} | f3_med={r['f3_chattering_mediana']:8.3f} | "
              f"Rank_f1={r['rango_friedman_f1']:4.2f} | p_Holm(f1)={r['p_holm_vs_base_f1']:.3e} | "
              f"p_Holm(f3_vs_ideal)={r['p_holm_vs_ideal_f3']:.3e}")
    print(f"\nFriedman f1 (RMSE): chi2 = {stat_friedman_f1:.4f}, p = {p_friedman_f1:.3e}")
    print(f"Friedman f3 (Chattering): chi2 = {stat_friedman_f3:.4f}, p = {p_friedman_f3:.3e}")

    # ------------------------------------------------------------------
    # GENERACIÓN DE FIGURA CIENTÍFICA 5 (300 DPI):
    # CONFRONTACIÓN ADVERSARIAL EN MUJOCO 3D HOSTIL + 30 SEMILLAS WILCOXON
    # ------------------------------------------------------------------
    fig, axes = plt.subplots(2, 2, figsize=(13.5, 9.8), dpi=300)

    # Panel (a): Frente de Pareto Tri-Objetivo en Gemelo Hostil MuJoCo (color = f3 chattering)
    ax = axes[0, 0]
    sc = ax.scatter(
        F_mod[:, 0], F_mod[:, 1], c=F_mod[:, 2], cmap="viridis",
        s=70, edgecolor="k", linewidth=0.6, zorder=4,
        label="Frente NSGA-II Modal $[\omega_n, \zeta]$"
    )
    ax.scatter(
        F_gs[:, 0], F_gs[:, 1], c=F_gs[:, 2], cmap="viridis",
        marker="s", s=65, edgecolor="k", linewidth=0.6, zorder=4,
        label="Frente NSGA-II Gain-Sched + RBF"
    )
    ax.scatter(
        F_base_hostil[0], F_base_hostil[1], marker="X", s=190, c="#d62728",
        edgecolor="k", linewidth=0.8, zorder=6,
        label=f"CTC Base Analítico ({F_base_hostil[0]:.2f} mm, $f_3$={F_base_hostil[2]:.1f})"
    )
    ax.scatter(
        F_ideal_en_hostil[0], F_ideal_en_hostil[1], marker="^", s=180, c="#ff7f0e",
        edgecolor="k", linewidth=0.8, zorder=6,
        label=f"$\\theta^*_{{ideal}}$ Tabla 2 ($\zeta_3$=0.13, $f_3$={F_ideal_en_hostil[2]:.1f})"
    )
    ax.scatter(
        F_mod_star[0], F_mod_star[1], marker="*", s=260, c="#2ca02c",
        edgecolor="k", linewidth=0.9, zorder=7,
        label=f"Óptimo Modal $\\theta^*_{{mod}}$ ({F_mod_star[0]:.2f} mm, $f_3$={F_mod_star[2]:.1f})"
    )
    ax.scatter(
        F_gs_star[0], F_gs_star[1], marker="P", s=210, c="#1f77b4",
        edgecolor="k", linewidth=0.9, zorder=7,
        label=f"Óptimo Gain-Sched $\\theta^*_{{GS}}$ ({F_gs_star[0]:.2f} mm, $f_3$={F_gs_star[2]:.1f})"
    )
    cbar = fig.colorbar(sc, ax=ax, pad=0.02)
    cbar.set_label(r"$f_3^{\mathrm{rob}} = \int \|\dot{\tau}\|^2 dt$ [$\mathrm{N^2m^2/s}$] (Chattering)", fontsize=9)
    ax.set_xlabel(r"Objetivo $f_1^{\mathrm{rob}}$: RMSE Cartesiano $0.5\mu + 0.5\max$ [mm] ($\leftarrow$ mejor)", fontsize=9.5)
    ax.set_ylabel(r"Objetivo $f_2^{\mathrm{rob}}$: Energía $\int \|\tau\|^2 dt$ [$\mathrm{N^2m^2s}$] ($\downarrow$ mejor)", fontsize=9.5)
    ax.set_title("(a) Frente Tri-Objetivo en MuJoCo 3D Hostil (CRN verificado)", fontsize=10.5, fontweight="bold")
    ax.grid(True, linestyle="--", alpha=0.45)
    ax.legend(fontsize=7.6, loc="upper right")

    # Panel (b): Convergencia de Hipervolumen HV(g) entre las 3 variantes en Gemelo Hostil
    ax = axes[0, 1]
    gens_axis = np.arange(1, len(hv_mod) + 1)
    ax.plot(gens_axis, hv_dir, "o--", color="#ff7f0e", lw=1.8, ms=4.5, label=f"NSGA-II Directo $[K_p,K_d]$ (IGD+={igd_dir:.3f})")
    ax.plot(gens_axis, hv_mod, "s-", color="#2ca02c", lw=2.1, ms=5.0, label=f"NSGA-II Modal $[\omega_n,\zeta]$ (IGD+={igd_mod:.3f})")
    ax.plot(gens_axis, hv_gs, "D-", color="#1f77b4", lw=2.2, ms=5.0, label=f"NSGA-II Gain-Sched + RBF (IGD+={igd_gs:.3f})")
    ax.set_xlabel("Generación $g$", fontsize=9.5)
    ax.set_ylabel("Hipervolumen Dominado Normalizado $HV(g)$", fontsize=9.5)
    ax.set_title("(b) Convergencia Multiobjetivo y Métrica IGD+ en MuJoCo Hostil", fontsize=10.5, fontweight="bold")
    ax.grid(True, linestyle="--", alpha=0.45)
    ax.legend(fontsize=8.2, loc="lower right")

    # Panel (c): Boxplot de Precisión Robusta f1 en N = 30 Semillas Independientes
    ax = axes[1, 0]
    etiquetas_cortas = [
        "C1: CTC Base\n($\\zeta=1.10$)",
        "C2: $\\theta^*_{ideal}$ Tabla 2\n($\\zeta_3=0.13$ borde)",
        "C3: NSGA-II\nDirecto Hostil",
        "C4: NSGA-II\nModal $[\\omega_n,\\zeta]$",
        "C5: Gain-Sched\n+ RBF + Memético",
    ]
    datos_f1_box = [pivot_f1[c] for c in orden_ctrl]
    bp1 = ax.boxplot(datos_f1_box, tick_labels=etiquetas_cortas, patch_artist=True, showmeans=True)
    colores_box = ["#d62728", "#ff7f0e", "#9467bd", "#2ca02c", "#1f77b4"]
    for patch, col in zip(bp1["boxes"], colores_box):
        patch.set_facecolor(col)
        patch.set_alpha(0.65)
    ax.set_ylabel(r"$f_1^{\mathrm{rob}}$: RMSE Robusto ($0.5\mu + 0.5\max$) [mm]", fontsize=9.5)
    ax.set_title(f"(c) Distribución en N = 30 Semillas LHS (Friedman $p = {p_friedman_f1:.2e}$)", fontsize=10.5, fontweight="bold")
    ax.grid(True, axis="y", linestyle="--", alpha=0.45)
    ax.tick_params(axis="x", labelsize=8.0)

    # Panel (d): Boxplot de Chattering / Tasa de Par f3 en N = 30 Semillas Independientes
    ax = axes[1, 1]
    datos_f3_box = [pivot_f3[c] for c in orden_ctrl]
    bp3 = ax.boxplot(datos_f3_box, tick_labels=etiquetas_cortas, patch_artist=True, showmeans=True)
    for patch, col in zip(bp3["boxes"], colores_box):
        patch.set_facecolor(col)
        patch.set_alpha(0.65)
    ax.set_ylabel(r"$f_3^{\mathrm{rob}} = \int \|\dot{\tau}\|^2 dt$ [$\mathrm{N^2m^2/s}$] (Chattering)", fontsize=9.5)
    ax.set_title(f"(d)Prueba de Chattering bajo Ruido/Retardo (Wilcoxon $p = {p_holm_vs_ideal_f3['C4_NSGA2_Modal_TriObj']:.2e}$)", fontsize=10.5, fontweight="bold")
    ax.grid(True, axis="y", linestyle="--", alpha=0.45)
    ax.tick_params(axis="x", labelsize=8.0)

    fig.tight_layout()
    fig_path = FIG_DIR / "fig_ce_05_confrontacion_hostil_30semillas.png"
    fig.savefig(fig_path, dpi=300)
    plt.close(fig)

    resumen_json = {
        "tiempo_total_seg": float(time.perf_counter() - t_global),
        "F_base_hostil": F_base_hostil.tolist(),
        "F_ideal_en_hostil": F_ideal_en_hostil.tolist(),
        "theta_dir_star": theta_dir_star.tolist(),
        "F_dir_star": F_dir_star.tolist(),
        "theta_mod_star": theta_mod_star.tolist(),
        "feno_mod_star_Kp": feno_mod_star["Kp"].tolist(),
        "feno_mod_star_Kd": feno_mod_star["Kd"].tolist(),
        "feno_mod_star_wn": feno_mod_star["wn"].tolist(),
        "feno_mod_star_z": feno_mod_star["z"].tolist(),
        "F_mod_star": F_mod_star.tolist(),
        "theta_gs_star": theta_gs_star.tolist(),
        "F_gs_star": F_gs_star.tolist(),
        "igd_plus": {
            "directo": igd_dir,
            "modal": igd_mod,
            "gain_scheduled_rbf": igd_gs,
        },
        "friedman_f1": {"chi2": float(stat_friedman_f1), "p": float(p_friedman_f1)},
        "friedman_f3": {"chi2": float(stat_friedman_f3), "p": float(p_friedman_f3)},
        "resumen_30_semillas": resumen_estadistico,
    }
    (RES_DIR / "resumen_confrontacion_30semillas.json").write_text(
        json.dumps(resumen_json, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    print(f"\nFigura guardada en: {fig_path}")
    print(f"Tiempo total de la suite: {time.perf_counter() - t_global:.2f} s")


if __name__ == "__main__":
    main()
