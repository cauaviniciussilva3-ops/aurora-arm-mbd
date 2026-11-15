/* ============================================================================
 * ARQUIVO GERADO AUTOMATICAMENTE - NAO EDITAR MANUALMENTE
 * Projeto: AuroraArm - Junta do Cotovelo (ARM Cortex-M4 FPU)
 * Gerador: aurora_arm/codegen.py (Verificacao Algebrica SymPy: Residuo == 0)
 * Requisitos Rastreados: REQ-CTRL-AA-001, REQ-CTRL-AA-002, REQ-CTRL-AA-003, REQ-CTRL-AA-004
 * ============================================================================ */
#include "aurora_pid.h"

void AuroraPID_Init(AuroraPID_State *st, double Kp, double Ki, double Kd, double N, double Ts, double Kaw) {
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
}

double AuroraPID_Step(AuroraPID_State *st, double ref, double y_medido, double omega_motor) {
    double e = ref - y_medido;
    double d_ant = st->derivativo;
    double i_ant = st->integrador;
    double e_ant = st->erro_ant;
    double Kp = st->Kp;
    double Kd = st->Kd;
    double N = st->N;
    double Ts = st->Ts;

    /* Expressao derivada simbolicamente via SymPy */
    double d_novo = Kd*pow(N, 2)*Ts*(e - e_ant)/(N*Ts + 1) + d_ant*(-N*Ts/(N*Ts + 1) + 1);
    double u_unsat = Kd*pow(N, 2)*Ts*(e - e_ant)/(N*Ts + 1) + Kp*e + d_ant*(-N*Ts/(N*Ts + 1) + 1) + i_ant;

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
}
