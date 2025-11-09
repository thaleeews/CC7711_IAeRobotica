# Projeto de Robótica - Identificação de Caixas Leves e Pesadas

## 📋 Descrição do Projeto

Este projeto implementa um controlador para o robô E-PUCK no simulador Webots que tem como objetivo identificar e classificar caixas em leves e pesadas através de tentativas de empurrão. O robô navega pelo ambiente, encontra todas as caixas disponíveis, tenta empurrá-las e, baseado no deslocamento, determina quais são leves (movem-se) e quais são pesadas (não se movem).

## 🎯 Objetivos

- Navegar pelo ambiente de forma autônoma
- Localizar todas as caixas disponíveis (11 caixas)
- Empurrar a caixa para determinar se é leve ou pesada
- Identificar a caixa mais leve
- Navegar até a caixa leve identificada e executar uma rotação no lugar

## 🤖 Robô Utilizado

- **Modelo**: E-PUCK
- **Sensores**: 8 sensores de proximidade (ps0 a ps7)
- **Motores**: 2 motores de roda (left wheel motor, right wheel motor)
- **Modo**: Supervisor (permite acesso às posições das caixas)

## 📁 Estrutura do Código

### Constantes Principais

```python
MAX_VELOCIDADE = 6.28      # Velocidade máxima dos motores
TIME_STEP = 450            # Intervalo de tempo entre atualizações (ms)
NUM_CAIXAS = 11            # Número total de caixas no ambiente
TOLERANCIA_DISTANCIA = 0.10 # Distância mínima para considerar chegada à caixa
LIMITE_SENSOR = 80         # Limite para detecção de obstáculos
```

### Funções Principais

#### 1. **Inicialização e Configuração**

- `get_caixas(supervisor, num_caixas)`: Obtém referências para todas as caixas e registra suas posições iniciais
- `inicializar_sensores(supervisor)`: Habilita os 8 sensores de proximidade do robô

#### 2. **Navegação**

- `controlar_movimento(robo_node, caixa_node, sensores, motor_esq, motor_dir)`: 
  - Controla o movimento do robô em direção a uma caixa
  - Implementa navegação proporcional adaptativa
  - Detecta e evita obstáculos usando sensores de proximidade
  - Ajusta velocidade baseado no erro angular

- `navigate_to_box(caixa_node, robo_node, sensores, motor_esq, motor_dir)`: 
  - Navega até uma caixa específica
  - Retorna `True` quando o robô chega à caixa (dentro da tolerância)

- `encontrar_caixa_mais_proxima(robo_node, caixas_restantes)`: 
  - Encontra a caixa leve mais próxima do robô
  - Retorna o índice e a referência da caixa

#### 3. **Interação com Caixas**

- `empurrar_caixa_durante(supervisor, motor_esq, motor_dir, TIME_STEP, duracao_segundos)`: 
  - Empurra a caixa por um período determinado (padrão: 2 segundos)
  - Move o robô para frente em velocidade máxima

- `verificar_movimento_caixas(caixas, posicoes_iniciais, tolerancia)`: 
  - Compara posições finais com posições iniciais
  - Classifica cada caixa como LEVE ou PESADA baseado no deslocamento
  - Imprime relatório de classificação

#### 4. **Detecção e Finalização**

- `caixa_leve_detectada(caixas, posicoes_iniciais, tolerancia)`: 
  - Verifica se alguma caixa foi movida (é leve)
  - Retorna `True` se pelo menos uma caixa leve foi detectada

- `identificar_e_girar_frente_a_caixa_leve(supervisor, robo_node, motor_esq, motor_dir, caixas, posicoes_iniciais, TIME_STEP)`: 
  - Identifica a caixa mais leve (maior deslocamento)
  - Navega até ela
  - Executa rotação contínua no lugar quando chega

#### 5. **Utilitários**

- `ler_sensores_proximidade(sensores)`: Lê valores de todos os sensores
- `mostrar_valores_sensores(sensores)`: Exibe valores dos sensores
- `delay(supervisor, TIME_STEP, time_milisec)`: Implementa delay controlado
- `imprimir_posicoes_caixas(caixas)`: Imprime posições atuais de todas as caixas

## 🔄 Fluxo de Execução

1. **Inicialização**:
   - Cria instância do Supervisor
   - Inicializa motores e sensores
   - Obtém referências para todas as caixas (CAIXA01 a CAIXA11)
   - Registra posições iniciais de todas as caixas

2. **Loop Principal**:
   - Encontra a caixa leve mais próxima ainda não visitada
   - Navega até ela usando controle proporcional adaptativo
   - Evita obstáculos detectados pelos sensores
   - Quando chega à caixa (distância < 0.10m):
     - Empurra a caixa por 2 segundos
     - Remove a caixa da lista de caixas restantes
     - Verifica se alguma caixa leve foi detectada
     - Se sim, interrompe a busca e vai para a caixa leve

3. **Finalização**:
   - Quando todas as caixas leves foram visitadas:
     - Analisa movimento de todas as caixas
     - Classifica cada uma como LEVE ou PESADA
     - Identifica a caixa mais leve
     - Navega até ela e executa rotação contínua

## 🧠 Algoritmos Implementados

### Navegação Proporcional Adaptativa

O sistema de navegação utiliza um controlador proporcional que ajusta a velocidade baseado no erro angular:

```python
k = 0.6 * (1.0 + distancia)  # Ganho adaptativo baseado na distância
v_base = 5.0
ajuste = max(min(k * erro, v_base), -v_base)
```

- **Erro angular pequeno** (< 0.1 rad): Robô anda reto em velocidade constante
- **Erro angular grande**: Robô ajusta velocidade das rodas proporcionalmente ao erro
- **Distância grande**: Aumenta o ganho para correções mais rápidas

### Evasão de Obstáculos

- Monitora sensores frontais (ps0, ps7) e laterais (ps1-ps6)
- Quando detecta obstáculo à frente (soma dos sensores > limite):
  - Inicia contador de evasão
  - Gira na direção com mais espaço livre
  - Continua evasão por alguns ciclos antes de retomar navegação

### Detecção de Caixas Leves

- Compara posição final com posição inicial após empurrão
- Se deslocamento > tolerância (0.01m): caixa é LEVE
- Se deslocamento < tolerância: caixa é PESADA
- A caixa com maior deslocamento é considerada a mais leve

## 🛠️ Requisitos

- **Webots** (versão compatível com Python controller)
- **Python 3.x**
- **Bibliotecas**:
  - `controller` (fornecido pelo Webots)
  - `math` (padrão Python)
  - `time` (padrão Python)
  - `random` (padrão Python)

## 📝 Configuração do Ambiente Webots

1. O robô deve ter o DEF `EPUCK`
2. As caixas devem ter DEFs de `CAIXA01` a `CAIXA11`
3. O robô deve estar configurado como Supervisor
4. Os sensores de proximidade devem estar nomeados como `ps0` a `ps7`
5. Os motores devem estar nomeados como `left wheel motor` e `right wheel motor`

## 🚀 Como Executar

1. Abra o arquivo `.wbt` no Webots
2. Configure o controlador do robô para usar `my_controller.py`
3. Execute a simulação
4. Observe o console para ver:
   - Progresso da navegação
   - Detecção de obstáculos
   - Classificação das caixas
   - Identificação da caixa leve

## 📊 Saída do Programa

O programa imprime no console:
- Distâncias até as caixas durante a navegação
- Alertas de obstáculos detectados
- Confirmação de chegada a cada caixa
- Relatório final classificando todas as caixas como LEVE ou PESADA
- Identificação da caixa mais leve encontrada

## 👥 Autores

- **THALES CLEMENTE PASQUOTTO** - RA: 22.222.033-7
- **LEANDRO DE BRITO ALENCAR** - RA: 22.222.034-5
- **JOÃO PAULO PAGGI ZUANON DIAS** - RA: 22.222.058-4

---

**Nota**: Este código foi desenvolvido para o simulador Webots e requer configuração adequada do ambiente de simulação para funcionar corretamente.

