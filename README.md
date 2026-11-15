```markdown
# Operação Linha Viva: Design Baseado em Modelos (MBD) para o Braço Colaborativo AuroraArm

Repositório técnico desenvolvido por **Cauã Vinícius Porto Silva** para a disciplina **Model Based Design for Cyber-Physical Systems (UniFECAF)**[cite: 12].

Este projeto implementa o fluxo completo de **Design Baseado em Modelos (MBD)** aplicado à junta do cotovelo do manipulador robótico colaborativo **AuroraArm** (6 graus de liberdade, atuação CC de 48 V e transmissão harmônica 100:1)[cite: 12], cobrindo:

1. **Unidade 1:** Modelagem física em espaço de estados ($A, B, C, D$), análise modal (constantes de tempo $\tau_e$ e $\tau_m$), controlabilidade, observabilidade e ponto de operação crítico[cite: 12].
2. **Unidade 2:** Projeto e sintonia de controlador PID discreto ($T_s = 2\text{ ms}$) com filtro derivativo, saturação de barramento ($48\text{ V}$), proteção dinâmica de corrente ($\le 8\text{ A}$) e mecanismo *anti-windup* por *back-calculation*[cite: 12].
3. **Unidade 3:** Formalização de requisitos de segurança colaborativa (**ISO/TS 15066** e **IEC 61508**), verificação formal exaustiva (BFS) em máquina de estados finitos com geração de contraexemplo na versão preliminar (`v0`), validação da versão corrigida (`v1`), geração de suíte de testes por percurso de grafo e verificação exaustiva de prazos de tempo real ($\le 100\text{ ms}$ sob atraso de sensor de até $20\text{ ms}$ e jitter de $2\text{ ms}$)[cite: 12, 18].
4. **Unidade 4:** Derivação simbólica via `SymPy` com verificação de resíduo algébrico identicamente nulo, geração automática de código C embarcado para **ARM Cortex-M4 (32 bits com FPU)** com cabeçalho de rastreabilidade, teste de equivalência numérica *Software-in-the-Loop* (SIL) e matriz de rastreabilidade completa[cite: 12, 18].

---

## Estrutura do Repositório

```text
aurora-arm-mbd/
├── aurora_arm/
│   ├── __init__.py                 # Inicialização do pacote Python
│   ├── planta.py                   # Unidade 1: Espaço de estados, autovalores, posto Ctrb/Obsv e ponto crítico
│   ├── controlador.py              # Unidade 2: Controlador PID discreto + proteção de corrente + anti-windup
│   ├── supervisor.py               # Unidade 3: Supervisor de segurança, model checker (BFS) e varredura de prazos
│   └── codegen.py                  # Unidade 4: Derivação simbólica SymPy, gerador C e teste SIL (Python vs C)
├── gerado/
│   ├── aurora_pid.h                # Cabeçalho C gerado automaticamente (estruturas e protótipos)
│   └── aurora_pid.c                # Implementação C gerada automaticamente com bloco de rastreabilidade
├── resultados/
│   ├── evidencias_aurora_arm.json  # Saída consolidada com todas as métricas numéricas das 4 unidades
│   └── resposta_pid.png            # Curvas de resposta temporal (posição, tensão e corrente)
├── executar_tudo.py                # Script mestre que executa todo o pipeline MBD de ponta a ponta
└── README.md                       # Documentação técnica completa do projeto

```

---

## Parâmetros Físicos da Planta (`aurora_arm/planta.py`)

| Parâmetro | Símbolo | Valor | Unidade |
| --- | --- | --- | --- |
| Resistência de armadura | $R$ | $0,8$ | $\Omega$

 |
| Indutância de armadura | $L$ | $1,2$ | $\text{mH}$

 |
| Constante de torque | $K_t$ | $0,085$ | $\text{N}\cdot\text{m/A}$

 |
| Constante de f.c.e.m. | $K_e$ | $0,085$ | $\text{V}\cdot\text{s/rad}$

 |
| Inércia refletida no eixo do motor | $J$ | $4,2 \times 10^{-4}$ | $\text{kg}\cdot\text{m}^2$

 |
| Atrito viscoso refletido | $b$ | $1,5 \times 10^{-4}$ | $\text{N}\cdot\text{m}\cdot\text{s/rad}$

 |
| Relação de redução harmônica | $N$ | $100:1$ | —

 |
| Tensão máxima do barramento | $V_{\max}$ | $48$ | $\text{V}$

 |
| Corrente máxima do driver | $i_{\max}$ | $8$ | $\text{A}$

 |
| Período de amostragem alvo | $T_s$ | $2$ | $\text{ms}$ ($500\text{ Hz}$)

 |

---

## Resumo dos Resultados Obtidos

### 1. Modelagem e Análise Modal (`aurora_arm/planta.py`)

* **Polos exatos da matriz $A$:** $\lambda_1 = -644,41\text{ s}^{-1}$ (modo elétrico rápido) e $\lambda_2 = -22,62\text{ s}^{-1}$ (modo mecânico dominante).


* **Constantes de tempo exatas:** $\tau_e = 1,55\text{ ms}$ vs. $\tau_m = 44,22\text{ ms}$ (separação de escalas de $\approx 28,5\times$).


* **Controlabilidade e Observabilidade:** Ambas as matrizes apresentam **posto completo igual a 2** ($\text{posto}(\mathcal{C}) = 2$, $\text{posto}(\mathcal{O}) = 2$), garantindo que o sistema é completamente controlável via tensão $V$ e observável a partir da leitura angular do encoder.


* **Ponto de Operação Crítico ($\omega_{ss} = 450\text{ rad/s}$, $T_{\text{carga}} = 0,25\text{ N}\cdot\text{m}$):**


* Corrente exigida: $i_{ss} = 3,735\text{ A}$ (dentro do limite de $8\text{ A}$).


* Tensão exigida: $V_{ss} = 41,24\text{ V}$ (restam **$6,76\text{ V}$ de folga** até o teto de $48\text{ V}$ do barramento, exigindo saturação explícita e *anti-windup* para evitar acúmulo integral durante transitórios rápidos).




* **Amostras por constante de tempo mecânica:** $\frac{\tau_m}{T_s} = \frac{44,22\text{ ms}}{2\text{ ms}} = \mathbf{22,11\text{ amostras}}$, situado na faixa ideal de projeto ($10$ a $30$ amostras por constante de tempo dominante).



### 2. Verificação Formal de Segurança (`aurora_arm/supervisor.py`)

* **Modelo Preliminar (`v0` — com bug de prioridade):** A exploração exaustiva encontrou **4 violações** de `REQ-SAFE-AA-001`. O contraexemplo mínimo (2 transições) mostrou que, se `comando_partir` permanecesse ativo enquanto `mao_proxima` se tornava verdadeiro em `MOVENDO`, o torque permanecia habilitado.


* **Modelo Corrigido (`v1`):**
* Estados alcançáveis: **4 de 4** ($100\%$ de cobertura de estados).


* Transições exploradas exaustivamente: **128 transições** ($4\text{ estados} \times 32\text{ combinações de entrada}$).


* Violações de segurança (`REQ-SAFE-AA-001` e `REQ-SAFE-AA-002`): **0 violações**.


* Vivacidade (`REQ-SAFE-AA-003` — retomada controlada após remoção da mão e rearme explícito): **Verificada (`True`)**.




* **Varredura Exaustiva de Prazos Temporais (`REQ-SAFE-AA-001` $\le 100\text{ ms}$):**
* Combinações analisadas: **84 cenários** ($t_{\text{sensor}} \in [0, 20]\text{ ms}$, amostragem $T_s \in \{0, 2\}\text{ ms}$, jitter RTOS $\in \{0, 2\}\text{ ms}$, execução $< 0,2\text{ ms}$ e decaimento elétrico $5\tau_e = 7,76\text{ ms}$).


* Pior caso total: **$31,96\text{ ms}$** (margem de segurança de **$68,04\text{ ms}$** abaixo do limite normativo de $100\text{ ms}$).





### 3. Geração Automática de Código C e Equivalência SIL (`aurora_arm/codegen.py`)

* **Verificação Algébrica Simbólica (`SymPy`):** Resíduo simbólico entre a lei de controle derivada e o contrato de referência identicamente igual a **`0`**.


* **Arquivos Gerados:** `gerado/aurora_pid.h` e `gerado/aurora_pid.c` (aritmética IEEE-754 `double` otimizada para FPU do ARM Cortex-M4).


* **Equivalência Numérica SIL ($2.000$ amostras):** Erro máximo absoluto de **$0,0\text{ V}$** (razão $0,0 \times \varepsilon_{\text{double}}$, onde $\varepsilon_{\text{double}} \approx 2,22 \times 10^{-16}$).



---

## Matriz de Rastreabilidade (Requisito → Modelo → Código → Teste)

| ID Requisito | Descrição Resumida | Modelo / Equação | Arquivo / Implementação | Verificação / Evidência |
| --- | --- | --- | --- | --- |
| **REQ-CTRL-AA-001** | Erro de posição nulo em regime permanente | Polo na origem (ação integral $K_i$) | `aurora_arm/controlador.py` e `gerado/aurora_pid.c` | `simular_malha_fechada()` (`resultados/resposta_pid.png`)

 |
| **REQ-CTRL-AA-002** | Sobressinal máximo $\le 8\%$ | Amortecimento derivativo ($K_d, N$) + *anti-windup* | `aurora_arm/controlador.py` e `gerado/aurora_pid.c` | `simular_malha_fechada()` (`sobressinal_pct`)

 |
| **REQ-CTRL-AA-003** | Tempo de acomodação $\le 300\text{ ms}$ | Discretização a $T_s = 2\text{ ms}$ ($22,11$ amostras/$\tau_m$) | `aurora_arm/controlador.py` e `gerado/aurora_pid.c` | `simular_malha_fechada()` (`t_acomodacao_ms`)

 |
| **REQ-CTRL-AA-004** | Tensão $\le 48\text{ V}$ e corrente $\le 8\text{ A}$ | Saturação de barramento + limitador $K_e\omega \pm R i_{\lim}$ | `aurora_arm/controlador.py` e `gerado/aurora_pid.c` | `simular_malha_fechada()` (`tensao_max_V`, `corrente_max_A`)

 |
| **REQ-SAFE-AA-001** | Zerar torque em $\le 100\text{ ms}$ se mão $< 500\text{ mm}$ | `AG !(mao_proxima && torque_habilitado)` e prazo $\le 100\text{ ms}$ | `aurora_arm/supervisor.py` (`transicao_corrigida`) | `verificar_modelo()` ($0$ violações) e `verificar_prazos_temporais()` ($31,96\text{ ms}$)

 |
| **REQ-SAFE-AA-002** | Proibir retomada automática sem rearme | Guarda `not mao_proxima and rearme_operador` | `aurora_arm/supervisor.py` (`transicao_corrigida`) | `verificar_modelo()` ($128$ transições exploradas)

 |
| **REQ-SAFE-AA-003** | Retomar trajetória após remoção da mão e rearme | Propriedade de vivacidade (`EF MOVENDO`) | `aurora_arm/supervisor.py` (`transicao_corrigida`) | `verificar_modelo()` (`vivacidade_req_003_ok = True`)

 |
| **REQ-SAFE-AA-004** | Velocidade linear do efetuador $\le 250\text{ mm/s}$ | Transição imediata para `FALHA` quando `vel_excedida == True` | `aurora_arm/supervisor.py` (`transicao_corrigida`) | `verificar_modelo()` (cobertura $100\%$ de estados/entradas)

 |

---

## Como Reproduzir os Resultados

1. **Clonar o repositório:**
```bash
git clone https://github.com/cauaviniciussilva3-ops/aurora-arm-mbd.git
cd aurora-arm-mbd

```


2. **Instalar as dependências Python:**
```bash
pip install numpy scipy matplotlib sympy

```


3. **Executar todo o pipeline (Modelagem, Controle, Verificação Formal, Codegen C e SIL):**
```bash
python executar_tudo.py

```


Os resultados consolidados serão gravados em `resultados/evidencias_aurora_arm.json`, os gráficos em `resultados/resposta_pid.png` e o código C embarcado em `gerado/aurora_pid.c` e `gerado/aurora_pid.h`.



---

## Honestidade Técnica e Limites de Certificação

Em conformidade com as normas **ISO/TS 15066** e **IEC 61508**:

* **O que este pacote sustenta:** Fornece evidência matemática e experimental reprodutível de modelagem física, estabilidade e desempenho em malha fechada, verificação formal exaustiva do supervisor lógico e de prazos de pior caso ($31,96\text{ ms} < 100\text{ ms}$), além de equivalência numérica SIL entre modelo e código C gerado.


* **O que ainda seria exigido para homologação industrial final:** A qualificação formal da cadeia de ferramentas (gerador simbólico e compilador), verificação independente por equipe distinta da equipe de desenvolvimento e testes *Hardware-in-the-Loop* (HIL) e em bancada física com o escâner de segurança real acoplado.



```

### Como atualizar no GitHub (2 passos):
1. Substitua o conteúdo de `README.md` no VS Code pelo texto acima (dica: use o botão **Copiar** no canto superior direito do bloco de código acima para não vir nenhum caractere extra!) e salve (`Ctrl + S`).
2. Rode no terminal do VS Code:
```bash
git add README.md
git commit -m "Corrige formatacao das tabelas e formulas no README"
git push

```
