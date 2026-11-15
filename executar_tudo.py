import os
import json
from aurora_arm.planta import analisar_planta
from aurora_arm.controlador import simular_malha_fechada
from aurora_arm.supervisor import transicao_preliminar_com_bug, transicao_corrigida, verificar_modelo, verificar_prazos_temporais
from aurora_arm.codegen import executar_teste_sil

def main():
    os.makedirs("resultados", exist_ok=True)
    os.makedirs("gerado", exist_ok=True)

    relatorio = {
        "unidade_1_planta": {k: (v.tolist() if hasattr(v, "tolist") else float(v) if hasattr(v, "item") else v)
                             for k, v in analisar_planta().items()},
        "unidade_2_controle_com_aw": simular_malha_fechada(com_anti_windup=True, salvar_grafico="resultados/resposta_pid.png"),
        "unidade_2_controle_sem_aw": simular_malha_fechada(com_anti_windup=False),
        "unidade_3_supervisor_v0": str(verificar_modelo(transicao_preliminar_com_bug)),
        "unidade_3_supervisor_v1": verificar_modelo(transicao_corrigida),
        "unidade_3_prazos_100ms": verificar_prazos_temporais(),
        "unidade_4_sil_codegen": executar_teste_sil(pasta_saida="gerado", amostras=2000),
    }

    with open("resultados/evidencias_aurora_arm.json", "w", encoding="utf-8") as f:
        json.dump(relatorio, f, indent=2, ensure_ascii=False)

    print("=== EXECUCAO COMPLETA DO PACOTE AURORA-ARM ===")
    print("1. Controle (com AW):", relatorio["unidade_2_controle_com_aw"])
    print("2. Controle (sem AW):", relatorio["unidade_2_controle_sem_aw"])
    print("3. Supervisor v1:", relatorio["unidade_3_supervisor_v1"])
    print("4. Prazos 100ms:", relatorio["unidade_3_prazos_100ms"])
    print("5. Codegen & SIL:", relatorio["unidade_4_sil_codegen"])
    print("\nArquivos gerados em 'gerado/' e 'resultados/' com sucesso!")

if __name__ == "__main__":
    main()