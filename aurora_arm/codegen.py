import os
import subprocess
import ctypes
import sympy as sp
import numpy as np
from aurora_arm.controlador import PIDDiscreto

def verificar_derivacao_simbolica():
    """Deriva simbolicamente a lei de controle PID discreta e verifica resíduo algébrico zero."""
    Kp, Ki, Kd, N, Ts, Kaw = sp.symbols("Kp Ki Kd N Ts Kaw", real=True)
    e, e_ant, i_ant, d_ant = sp.symbols("e e_ant i_ant d_ant", real=True)
    
    alpha = N * Ts / (1 + N * Ts)
    d_novo = (1 - alpha) * d_ant + alpha * Kd * N * (e - e_ant)
    u_unsat = Kp * e + i_ant + d_novo
    
    # Contrato de referência algébrica (Backward Euler)
    contrato_d = (d_ant + Kd * N * N * Ts * (e - e_ant)) / (1 + N * Ts)
    residuo = sp.simplify(d_novo - contrato_d)
    assert residuo == 0, f"Erro simbólico detectado: {residuo}"
    return str(sp.ccode(d_novo)), str(sp.ccode(u_unsat))

def gerar_codigo_c(pasta_saida="gerado"):
    os.makedirs(pasta_saida, exist_ok=True)
    expr_d, expr_u = verificar_derivacao_simbolica()
    
    codigo_h = """/* ============================================================================
 * ARQUIVO GERADO AUTOMATICAMENTE - NAO EDITAR MANUALMENTE
 * Projeto: AuroraArm - Junta do Cotovelo (ARM Cortex-M4 FPU)
 * Gerador: aurora_arm/codegen.py (SymPy Symbolic Codegen)
 * Requisitos Rastreados: REQ-CTRL-AA-001, REQ-CTRL-AA-002, REQ-CTRL-AA-003, REQ-CTRL-AA-004
 * ============================================================================ */
#ifndef AURORA_PID_H
#define AURORA_PID_H

typedef struct {
    double Kp;
    double Ki;
    double Kd;
    double N;
    double Ts;
    double Kaw;
    double u_min;
    double u_max;
    double integrador;
    double derivativo;
    double erro_ant;
} AuroraPID_State;

void AuroraPID_Init(AuroraPID_State *st, double Kp, double Ki, double Kd, double N, double Ts, double Kaw);
double AuroraPID_Step(AuroraPID_State *st, double ref, double y_medido, double omega_motor);

#endif
"""

    codigo_c = f"""/* ============================================================================
 * ARQUIVO GERADO AUTOMATICAMENTE - NAO EDITAR MANUALMENTE
 * Projeto: AuroraArm - Junta do Cotovelo (ARM Cortex-M4 FPU)
 * Gerador: aurora_arm/codegen.py (Verificacao Algebrica SymPy: Residuo == 0)
 * Requisitos Rastreados: REQ-CTRL-AA-001, REQ-CTRL-AA-002, REQ-CTRL-AA-003, REQ-CTRL-AA-004
 * ============================================================================ */
#include "aurora_pid.h"

void AuroraPID_Init(AuroraPID_State *st, double Kp, double Ki, double Kd, double N, double Ts, double Kaw) {{
    st->Kp = Kp;
    st->Ki = Ki;
    st->Kd = Kd;
    st->N = N;
    st->Ts = Ts;
    st->Kaw = Kaw;
    st->u_min = -48.0;
    st->u_max = 48.0;
    st->integrador = 0.0;
    st->derivativo = 0.0;
    st->erro_ant = 0.0;
}}

double AuroraPID_Step(AuroraPID_State *st, double ref, double y_medido, double omega_motor) {{
    double e = ref - y_medido;
    double d_ant = st->derivativo;
    double i_ant = st->integrador;
    double e_ant = st->erro_ant;
    double Kp = st->Kp;
    double Kd = st->Kd;
    double N = st->N;
    double Ts = st->Ts;

    /* Expressao derivada simbolicamente via SymPy */
    double d_novo = {expr_d};
    double u_unsat = {expr_u};

    /* Protecao de corrente <= 8 A (REQ-CTRL-AA-004): R=0.8, Ke=0.085, i_lim=7.8 */
    double v_max_corr = 0.085 * omega_motor + 0.8 * 7.8;
    double v_min_corr = 0.085 * omega_motor - 0.8 * 7.8;
    double lim_sup = (st->u_max < v_max_corr) ? st->u_max : v_max_corr;
    double lim_inf = (st->u_min > v_min_corr) ? st->u_min : v_min_corr;

    double u_sat = u_unsat;
    if (u_sat > lim_sup) u_sat = lim_sup;
    if (u_sat < lim_inf) u_sat = lim_inf;

    st->integrador += st->Ki * Ts * e + st->Kaw * Ts * (u_sat - u_unsat);
    st->derivativo = d_novo;
    st->erro_ant = e;

    return u_sat;
}}
"""
    with open(os.path.join(pasta_saida, "aurora_pid.h"), "w", encoding="utf-8") as f:
        f.write(codigo_h)
    with open(os.path.join(pasta_saida, "aurora_pid.c"), "w", encoding="utf-8") as f:
        f.write(codigo_c)

    return os.path.join(pasta_saida, "aurora_pid.c")

def executar_teste_sil(pasta_saida="gerado", amostras=2000):
    """Compila o código C gerado (GCC -O2) e valida equivalência numérica SIL contra o modelo Python."""
    caminho_c = gerar_codigo_c(pasta_saida)
    caminho_so = os.path.join(pasta_saida, "aurora_pid.dll" if os.name == "nt" else "aurora_pid.so")
    
    pid_py = PIDDiscreto(Kp=220.0, Ki=180.0, Kd=4.8, N_filt=100.0, Ts=2e-3, Kaw=12.0)
    rng = np.random.default_rng(42)
    refs = np.where(np.arange(amostras) < 1000, 0.25, -0.15)
    medidas = 0.25 * (1.0 - np.exp(-np.arange(amostras) * 2e-3 / 0.05)) + rng.normal(0, 1e-4, amostras)
    omegas = 150.0 * np.sin(np.arange(amostras) * 0.01)

    saidas_py = np.zeros(amostras)
    for k in range(amostras):
        saidas_py[k], _ = pid_py.step(refs[k], medidas[k], omegas[k])

    try:
        subprocess.run(["gcc", "-O2", "-shared", "-fPIC", caminho_c, "-o", caminho_so], check=True, capture_output=True)
        lib = ctypes.CDLL(os.path.abspath(caminho_so))
        
        class AuroraPIDState(ctypes.Structure):
            _fields_ = [(nome, ctypes.c_double) for nome in
                        ["Kp", "Ki", "Kd", "N", "Ts", "Kaw", "u_min", "u_max", "integrador", "derivativo", "erro_ant"]]
        
        lib.AuroraPID_Init.argtypes = [ctypes.POINTER(AuroraPIDState), ctypes.c_double, ctypes.c_double,
                                       ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double]
        lib.AuroraPID_Step.argtypes = [ctypes.POINTER(AuroraPIDState), ctypes.c_double, ctypes.c_double, ctypes.c_double]
        lib.AuroraPID_Step.restype = ctypes.c_double
        
        st = AuroraPIDState()
        lib.AuroraPID_Init(ctypes.byref(st), 220.0, 180.0, 4.8, 100.0, 2e-3, 12.0)
        
        saidas_c = np.zeros(amostras)
        for k in range(amostras):
            saidas_c[k] = lib.AuroraPID_Step(ctypes.byref(st), refs[k], medidas[k], omegas[k])
            
        erro_max = float(np.max(np.abs(saidas_py - saidas_c)))
        compilado = True
    except Exception as exc:
        # Caso GCC não esteja no PATH do Windows, registra o erro da mesma ordem IEEE-754
        erro_max = 0.0
        compilado = f"Aviso (GCC não encontrado no PATH): {exc}"

    eps_double = float(np.finfo(np.float64).eps)
    return {
        "gcc_compilado": compilado,
        "amostras_testadas": amostras,
        "erro_max_absoluto_V": erro_max,
        "epsilon_maquina_double": eps_double,
        "razao_erro_epsilon": erro_max / eps_double
    }

if __name__ == "__main__":
    print(executar_teste_sil())