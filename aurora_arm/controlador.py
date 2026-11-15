import numpy as np
import matplotlib.pyplot as plt
from aurora_arm.planta import PARAMS, obter_matrizes_junta

class PIDDiscreto:
    def __init__(self, Kp, Ki, Kd, N_filt, Ts, u_min=-48.0, u_max=48.0, Kaw=2.5):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.N = N_filt
        self.Ts = Ts
        self.u_min = u_min
        self.u_max = u_max
        self.Kaw = Kaw
        self.reset()

    def reset(self):
        self.integrador = 0.0
        self.derivativo = 0.0
        self.erro_ant = 0.0

    def step(self, ref, y_medido, omega_motor=0.0):
        erro = ref - y_medido
        P = self.Kp * erro
        
        alpha = self.N * self.Ts / (1.0 + self.N * self.Ts)
        self.derivativo = (1.0 - alpha) * self.derivativo + alpha * self.Kd * N_val(self.N) * (erro - self.erro_ant)
        
        u_unsat = P + self.integrador + self.derivativo
        
        # Proteção dinâmica de corrente <= 8 A (REQ-CTRL-AA-004)
        R, Ke, i_lim = PARAMS["R"], PARAMS["Ke"], 7.8
        v_max_corr = Ke * omega_motor + R * i_lim
        v_min_corr = Ke * omega_motor - R * i_lim
        
        lim_sup = min(self.u_max, v_max_corr)
        lim_inf = max(self.u_min, v_min_corr)
        
        u_sat = float(np.clip(u_unsat, lim_inf, lim_sup))
        
        self.integrador += self.Ki * self.Ts * erro + self.Kaw * self.Ts * (u_sat - u_unsat)
        self.erro_ant = erro
        
        return u_sat, u_unsat

def N_val(n):
    return n

def simular_malha_fechada(ref_junta_rad=0.10, tempo_total=0.45, com_anti_windup=True, salvar_grafico=None):
    p = PARAMS
    Ts = p["Ts"]
    passos = int(tempo_total / Ts)
    t = np.arange(passos) * Ts
    
    Kaw = 2.5 if com_anti_windup else 0.0
    pid = PIDDiscreto(Kp=140.0, Ki=180.0, Kd=12.0, N_filt=100.0, Ts=Ts, u_min=-48.0, u_max=48.0, Kaw=5.0)
    
    sub_passos = 100
    dt = Ts / sub_passos
    A, B, C, _ = obter_matrizes_junta(p)
    
    x = np.zeros(3)  # [i, omega_m, theta_j]
    hist_theta = np.zeros(passos)
    hist_i = np.zeros(passos)
    hist_u = np.zeros(passos)
    hist_int = np.zeros(passos)
    
    for k in range(passos):
        theta_j = x[2]
        omega_m = x[1]
        u_sat, _ = pid.step(ref_junta_rad, theta_j, omega_m)
        
        # Perturbação de carga nominal (0.25 Nm)
        T_carga = p["T_carga"] if t[k] >= 0.02 else 0.0
        
        for _ in range(sub_passos):
            dx = A @ x + B.flatten() * u_sat
            dx[1] -= T_carga / p["J"]
            x = x + dt * dx
            
        hist_theta[k] = x[2]
        hist_i[k] = x[0]
        hist_u[k] = u_sat
        hist_int[k] = pid.integrador

    valor_final = hist_theta[-1]
    erro_regime = abs(ref_junta_rad - valor_final)
    pico = np.max(hist_theta)
    sobressinal_pct = max(0.0, (pico - ref_junta_rad) / ref_junta_rad * 100.0)
    
    fora_faixa = np.where(np.abs(hist_theta - ref_junta_rad) > 0.02 * ref_junta_rad)[0]
    t_acomodacao_ms = (t[fora_faixa[-1]] * 1000.0) if len(fora_faixa) > 0 else 0.0
    
    metricas = {
        "erro_regime_rad": float(erro_regime),
        "sobressinal_pct": float(sobressinal_pct),
        "t_acomodacao_ms": float(t_acomodacao_ms),
        "tensao_max_V": float(np.max(np.abs(hist_u))),
        "corrente_max_A": float(np.max(np.abs(hist_i))),
        "integrador_max": float(np.max(np.abs(hist_int)))
    }
    
    if salvar_grafico:
        fig, axs = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
        axs[0].plot(t * 1000, hist_theta, label="Posição Junta (rad)", lw=2)
        axs[0].axhline(ref_junta_rad, color="r", ls="--", label=f"Referência ({ref_junta_rad} rad)")
        axs[0].set_ylabel("Posição [rad]")
        axs[0].set_title("Resposta da Junta do Cotovelo (AuroraArm) - PID Discreto + Anti-Windup")
        axs[0].legend()
        axs[0].grid(True)
        
        axs[1].plot(t * 1000, hist_u, color="g", label="Tensão Aplicada [V]", lw=2)
        axs[1].axhline(48.0, color="r", ls=":", label="Limite Driver 48 V")
        axs[1].set_ylabel("Tensão [V]")
        axs[1].legend()
        axs[1].grid(True)
        
        axs[2].plot(t * 1000, hist_i, color="m", label="Corrente Armadura [A]", lw=2)
        axs[2].axhline(8.0, color="r", ls=":", label="Limite Driver 8 A")
        axs[2].set_ylabel("Corrente [A]")
        axs[2].set_xlabel("Tempo [ms]")
        axs[2].legend()
        axs[2].grid(True)
        
        plt.tight_layout()
        plt.savefig(salvar_grafico, dpi=150)
        plt.close()
        
    return metricas

if __name__ == "__main__":
    import os
    os.makedirs("resultados", exist_ok=True)
    m_aw = simular_malha_fechada(com_anti_windup=True, salvar_grafico="resultados/resposta_pid.png")
    m_sem = simular_malha_fechada(com_anti_windup=False)
    print("COM Anti-Windup:", m_aw)
    print("SEM Anti-Windup:", m_sem)