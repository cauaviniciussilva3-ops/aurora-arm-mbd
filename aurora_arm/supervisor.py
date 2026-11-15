from enum import Enum
from dataclasses import dataclass
from collections import deque

class Estado(Enum):
    OCIOSO = "OCIOSO"
    MOVENDO = "MOVENDO"
    PARADO_SEGURANCA = "PARADO_SEGURANCA"
    FALHA = "FALHA"

@dataclass(frozen=True)
class Entradas:
    mao_proxima: bool       # Escâner < 500 mm
    comando_partir: bool    # Comando de ciclo
    rearme_operador: bool   # Botão/confirmação explícita
    falha_sensor: bool      # Falha de diagnóstico
    vel_excedida: bool      # Velocidade > 250 mm/s na zona colaborativa

@dataclass(frozen=True)
class Saidas:
    torque_habilitado: bool
    velocidade_limitada: bool
    alarme: bool

def transicao_preliminar_com_bug(estado: Estado, ent: Entradas):
    """Versão v0 (com bug de prioridade): comando_partir avaliado antes de mao_proxima."""
    if ent.falha_sensor:
        return Estado.FALHA, Saidas(False, True, True)
    if estado == Estado.OCIOSO:
        if ent.comando_partir and not ent.mao_proxima:
            return Estado.MOVENDO, Saidas(True, True, False)
        return Estado.OCIOSO, Saidas(False, True, False)
    if estado == Estado.MOVENDO:
        # BUG: Se comando_partir continuar ativo, ignora mao_proxima!
        if ent.comando_partir:
            return Estado.MOVENDO, Saidas(True, True, False)
        if ent.mao_proxima:
            return Estado.PARADO_SEGURANCA, Saidas(False, True, False)
        return Estado.MOVENDO, Saidas(True, True, False)
    if estado == Estado.PARADO_SEGURANCA:
        if not ent.mao_proxima and ent.rearme_operador:
            return Estado.MOVENDO, Saidas(True, True, False)
        return Estado.PARADO_SEGURANCA, Saidas(False, True, False)
    return Estado.FALHA, Saidas(False, True, True)

def transicao_corrigida(estado: Estado, ent: Entradas):
    """Versão v1 (corrigida): prioridade máxima para falha e proximidade (REQ-SAFE-AA-001..004)."""
    if ent.falha_sensor or ent.vel_excedida:
        return Estado.FALHA, Saidas(False, True, True)
    if ent.mao_proxima:
        return Estado.PARADO_SEGURANCA, Saidas(False, True, False)

    if estado == Estado.OCIOSO:
        if ent.comando_partir:
            return Estado.MOVENDO, Saidas(True, True, False)
        return Estado.OCIOSO, Saidas(False, True, False)

    if estado == Estado.MOVENDO:
        return Estado.MOVENDO, Saidas(True, True, False)

    if estado == Estado.PARADO_SEGURANCA:
        # REQ-SAFE-AA-002 e 003: só retoma sem mão próxima E com rearme explícito
        if not ent.mao_proxima and ent.rearme_operador:
            return Estado.MOVENDO, Saidas(True, True, False)
        return Estado.PARADO_SEGURANCA, Saidas(False, True, False)

    return Estado.FALHA, Saidas(False, True, True)

def todas_entradas():
    lista = []
    for m in [False, True]:
        for c in [False, True]:
            for r in [False, True]:
                for f in [False, True]:
                    for v in [False, True]:
                        lista.append(Entradas(m, c, r, f, v))
    return lista

def verificar_modelo(fn_transicao):
    """Exploração exaustiva (BFS) no espaço de estados e transições."""
    inicial = Estado.OCIOSO
    visitados = {inicial}
    fila = deque([(inicial, [])])
    transicoes_exploradas = 0
    violacoes = []
    grafo = {e: {} for e in Estado}

    while fila:
        est, caminho = fila.popleft()
        for ent in todas_entradas():
            transicoes_exploradas += 1
            prox_est, saida = fn_transicao(est, ent)
            grafo[est][ent] = prox_est
            novo_caminho = caminho + [(est.value, ent, prox_est.value, saida)]

            # Propriedade REQ-SAFE-AA-001: AG !(mao_proxima & torque_habilitado)
            if ent.mao_proxima and saida.torque_habilitado:
                violacoes.append(("REQ-SAFE-AA-001", novo_caminho))

            # Propriedade REQ-SAFE-AA-002: Não retomar movimento sem rearme após PARADO_SEGURANCA
            if est == Estado.PARADO_SEGURANCA and prox_est == Estado.MOVENDO and not ent.rearme_operador:
                violacoes.append(("REQ-SAFE-AA-002", novo_caminho))

            if prox_est not in visitados:
                visitados.add(prox_est)
                fila.append((prox_est, novo_caminho))

    # Verificação de vivacidade REQ-SAFE-AA-003: a partir de PARADO_SEGURANCA, existe caminho para MOVENDO
    ent_rearme = Entradas(mao_proxima=False, comando_partir=False, rearme_operador=True, falha_sensor=False, vel_excedida=False)
    est_apos_rearme, saida_rearme = fn_transicao(Estado.PARADO_SEGURANCA, ent_rearme)
    vivacidade_ok = (est_apos_rearme == Estado.MOVENDO and saida_rearme.torque_habilitado)

    return {
        "estados_alcancaveis": len(visitados),
        "total_estados_modelo": len(Estado),
        "transicoes_exploradas": transicoes_exploradas,
        "violacoes_count": len(violacoes),
        "primeiro_contraexemplo": violacoes[0] if violacoes else None,
        "vivacidade_req_003_ok": vivacidade_ok
    }

def verificar_prazos_temporais():
    """Varredura exaustiva dos atrasos de tempo real para REQ-SAFE-AA-001 (prazo <= 100 ms)."""
    Ts_ms = 2.0
    pior_caso_ms = 0.0
    combinacoes = 0
    violacoes_prazo = 0
    
    # Varre atraso do escâner (0 a 20 ms), atraso de amostragem (0 a 2 ms), jitter RTOS (0 ou 2 ms),
    # e tempo de decaimento elétrico da corrente até zero (~5 * tau_e = 7.76 ms)
    t_decaimento_torque_ms = 5.0 * 1.5518  # 7.76 ms
    
    for t_sensor in range(0, 21):           # 0 a 20 ms
        for t_amostragem in [0.0, Ts_ms]:   # 0 a 2 ms
            for t_jitter in [0.0, 2.0]:     # perda de 1 ciclo de 2 ms
                t_execucao = 0.2            # orçamento < 200 us (0.2 ms)
                t_total = t_sensor + t_amostragem + t_jitter + t_execucao + t_decaimento_torque_ms
                combinacoes += 1
                if t_total > pior_caso_ms:
                    pior_caso_ms = t_total
                if t_total > 100.0:
                    violacoes_prazo += 1

    return {
        "combinacoes_verificadas": combinacoes,
        "pior_caso_total_ms": round(pior_caso_ms, 2),
        "margem_seguranca_ms": round(100.0 - pior_caso_ms, 2),
        "violacoes_prazo_100ms": violacoes_prazo
    }

if __name__ == "__main__":
    print("=== MODELO PRELIMINAR (v0) ===")
    r_v0 = verificar_modelo(transicao_preliminar_com_bug)
    print("Violações:", r_v0["violacoes_count"], "| Contraexemplo:", r_v0["primeiro_contraexemplo"])
    
    print("\n=== MODELO CORRIGIDO (v1) ===")
    r_v1 = verificar_modelo(transicao_corrigida)
    print(r_v1)
    
    print("\n=== VERIFICAÇÃO DE PRAZO (REQ-SAFE-AA-001 <= 100 ms) ===")
    print(verificar_prazos_temporais())