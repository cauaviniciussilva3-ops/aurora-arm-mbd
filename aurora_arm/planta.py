import numpy as np
from scipy.linalg import eigvals

# Parâmetros físicos da junta do cotovelo (AuroraArm)
PARAMS = {
    "R": 0.8,          # Ohms
    "L": 1.2e-3,       # H (1.2 mH)
    "Kt": 0.085,       # N.m/A
    "Ke": 0.085,       # V.s/rad
    "J": 4.2e-4,       # kg.m^2
    "b": 1.5e-4,       # N.m.s/rad
    "N": 100.0,        # Redução 100:1
    "V_max": 48.0,     # V
    "i_max": 8.0,      # A
    "Ts": 2e-3,        # s (2 ms)
    "w_crit": 450.0,   # rad/s (eixo do motor)
    "T_carga": 0.25,   # N.m
}

def obter_matrizes_motor(p=PARAMS):
    """Matrizes de espaço de estados do motor (x = [i, omega]^T, u = V)."""
    R, L, Kt, Ke, J, b = p["R"], p["L"], p["Kt"], p["Ke"], p["J"], p["b"]
    A = np.array([
        [-R / L, -Ke / L],
        [Kt / J, -b / J]
    ], dtype=float)
    B = np.array([[1.0 / L], [0.0]], dtype=float)
    C = np.array([[0.0, 1.0]], dtype=float)  # Saída: velocidade angular omega
    D = np.array([[0.0]], dtype=float)
    return A, B, C, D

def obter_matrizes_junta(p=PARAMS):
    """Modelo estendido com posição da junta theta_j (x = [i, omega_m, theta_j]^T)."""
    R, L, Kt, Ke, J, b, N = p["R"], p["L"], p["Kt"], p["Ke"], p["J"], p["b"], p["N"]
    A = np.array([
        [-R / L, -Ke / L, 0.0],
        [Kt / J, -b / J,  0.0],
        [0.0,    1.0 / N, 0.0]
    ], dtype=float)
    B = np.array([[1.0 / L], [0.0], [0.0]], dtype=float)
    C = np.array([[0.0, 0.0, 1.0]], dtype=float)  # Saída: posição da junta (rad)
    D = np.array([[0.0]], dtype=float)
    return A, B, C, D

def analisar_planta(p=PARAMS):
    A, B, C, _ = obter_matrizes_motor(p)
    
    # Autovalores e constantes de tempo
    polos = eigvals(A)
    polos_reais = np.sort(np.real(polos))  # [polo_rapido, polo_lento]
    tau_rapido = -1.0 / polos_reais[0]
    tau_lento = -1.0 / polos_reais[1]
    
    # Aproximações desacopladas
    tau_e_aprox = p["L"] / p["R"]
    tau_m_aprox = (p["J"] * p["R"]) / (p["Kt"] * p["Ke"])
    
    # Controlabilidade e Observabilidade
    Ctrb = np.hstack([B, A @ B])
    Obsv = np.vstack([C, C @ A])
    posto_ctrb = np.linalg.matrix_rank(Ctrb)
    posto_obsv = np.linalg.matrix_rank(Obsv)
    
    # Ponto de operação crítico (regime permanente)
    w_ss = p["w_crit"]
    T_c = p["T_carga"]
    i_ss = (p["b"] * w_ss + T_c) / p["Kt"]
    V_ss = p["R"] * i_ss + p["Ke"] * w_ss
    folga_V = p["V_max"] - V_ss
    
    # Amostras por constante de tempo mecânica
    amostras_tau_m = tau_lento / p["Ts"]
    
    return {
        "A": A, "B": B, "C": C,
        "polos": polos_reais,
        "tau_eletrico_exato_ms": tau_rapido * 1e3,
        "tau_mecanico_exato_ms": tau_lento * 1e3,
        "tau_e_aprox_ms": tau_e_aprox * 1e3,
        "tau_m_aprox_ms": tau_m_aprox * 1e3,
        "posto_ctrb": posto_ctrb,
        "posto_obsv": posto_obsv,
        "i_ss_A": i_ss,
        "V_ss_V": V_ss,
        "folga_V": folga_V,
        "amostras_por_tau_m": amostras_tau_m
    }

if __name__ == "__main__":
    res = analisar_planta()
    for k, v in res.items():
        print(f"{k}: {v}")