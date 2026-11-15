/* ============================================================================
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
