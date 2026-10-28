# grid_controller_v1

Controlador de potência para ESP8266 ESP-07.

## Arquitetura

```text
URP6000 datalogger
  └─ Modbus TCP / Wi-Fi
       └─ ESP grid_controller_v1
            └─ Modbus RTU / RS485
                 ├─ SIW500H_ST030_M3 — ID 1
                 ├─ SIW400G_T100_W0 — ID 2
                 └─ SIW400G_T100_W0 — ID 3
```

O ESP lê a potência total do espelho do URP6000, lê a potência ativa dos três inversores, calcula uma ação PI e distribui o limite total proporcionalmente à potência nominal obtida por `getRatedPower()`.

## Arquivos

- `grid_controller_v1.ino`: firmware do ESP.
- `grid_controller_web.h`: página de status e página de configuração gravadas em PROGMEM.
- `grid_controller_web_server.py`: coleta as janelas estatísticas do ESP, grava CSV e serve o painel histórico.
- `grid_controller_mobile.html`: painel responsivo do histórico.
- `iniciar_monitor.bat`: inicia o coletor no Windows.
- `VALIDACAO_BANCADA.md`: sequência recomendada de validação.
- `NOTA_INVERTERMODBUSLIB.md`: incompatibilidades e pontos da biblioteca que precisam ser confirmados.
- `PATCH_OPCIONAL_InverterControl.diff`: correção opcional de uma condição de escala suspeita, caso ainda exista na versão instalada.

## Dependências Arduino

- ESP8266 Arduino Core.
- WiFiManager, de tzapu, com portal não bloqueante e suporte a `setHttpPort()`.
- modbus-esp8266, de Alexander Emelianov.
- InverterModbusLib do projeto do usuário.

O sketch foi estruturado para a API da biblioteca que contém:

```cpp
bool begin();
bool getRatedPower(uint32_t& watts);
bool getActivePower(float& watts);
bool setPowerLimit(float watts);
```

## Hardware padrão

```text
GPIO12: DE/RE do transceptor RS485
GPIO13: seleção half/full duplex, LOW para half duplex
UART0: 9600 8N1, compartilhada pelos três inversores
LED_BUILTIN: diagnóstico
```

Não use `Serial.print()` enquanto a UART0 estiver conectada ao RS485.

## Primeira inicialização

O controle PI começa **desabilitado** na primeira gravação. Isso é intencional.

Quando não houver Wi-Fi configurado, ou quando a conexão falhar por aproximadamente 60 segundos, o ESP abre:

```text
SSID: grid_controller-XXXXXX
Senha: CHANGE_ME_SETUP
Portal: http://192.168.4.1:8080
```

O WiFiManager permite escolher a rede. As credenciais são copiadas para a EEPROM do projeto.

## Endereços

```text
Status:        http://grid-controller.local/
Configuração: http://grid-controller.local/config
OTA:           http://grid-controller.local/update
API atual:     http://grid-controller.local/api/status
Estatísticas:  http://grid-controller.local/api/stats
```

Credenciais padrão de configuração e OTA:

```text
Usuário: admin
Senha:   CHANGE_ME_ADMIN
```

Troque as duas senhas padrão durante o comissionamento.

## Leitura do URP6000

O firmware consulta:

```text
0x003C..0x003E  relações RTC/RTP
0x0100..0x0102  status, SEQ e age
0x02CD..0x02D2  Pa, Pb e Pc em int32
```

A leitura é aceita somente quando:

- o status é válido;
- `age` está dentro do limite configurado;
- o `SEQ` antes e depois da leitura de potência é igual;
- o `SEQ` continua avançando ao longo do tempo.

A entrada do relé pode ser configurada como 1 A ou 5 A. O sinal da potência também pode ser invertido pela página.

## Controle PI

A convenção implementada é:

```text
erro = potência_URP_filtrada - setpoint
alvo_total = potência_controlada_medida - (Kp × erro + termo_integral)
```

O firmware inclui:

- mediana de três amostras;
- filtro exponencial configurável;
- banda morta;
- anti-windup;
- limite do termo integral;
- limite de rampa do comando;
- limite máximo percentual da capacidade nominal;
- diferença mínima antes de reenviar comando;
- redistribuição automática quando um inversor perde feedback.

Um inversor só participa do PI quando sua potência ativa está recente. No failsafe, o firmware ainda tenta comandar equipamentos que continuam respondendo, mesmo que o último feedback de potência tenha envelhecido.

## Failsafe

Quando a leitura do URP6000 permanece inválida pelo atraso configurado, o ESP distribui entre os inversores disponíveis o percentual de failsafe.

O padrão é:

```text
failsafe = 0%
```

O termo integral é zerado ao entrar em failsafe.

## LED

```text
Apagado: Wi-Fi desconectado
Duas piscadas: AP/portal ativo
Pisca rápido: URP inválido ou failsafe
Três piscadas: nenhum inversor com feedback válido
Duas piscadas lentas: parte dos inversores indisponível
Uma piscada curta: controle PI pausado
Aceso: sistema completo e PI habilitado
Muito rápido: OTA em andamento
```

## Limitação importante da RTU

O escalonador do sketch é cooperativo e executa somente uma operação da InverterModbusLib por vez. Porém, na versão analisada, os métodos públicos da biblioteca aguardam internamente o término da transação Modbus. Portanto, o restante do firmware pode ficar suspenso durante o timeout curto de uma chamada.

Para RTU realmente assíncrona, a biblioteca precisará futuramente expor uma API do tipo:

```cpp
startGetActivePower(...);
poll();
isDone();
result();
```

A versão atual reduz o impacto com timeout curto, rodízio e uma única operação por passo, mas não deve ser descrita como totalmente não bloqueante.

## Estatísticas

O ESP agrega, por padrão, janelas de 5 minutos com:

- média, mínimo e máximo da potência no URP;
- média, mínimo e máximo da potência dos inversores controlados;
- média do setpoint, erro e comando;
- número de amostras;
- máscara dos inversores disponíveis;
- alarmes ocorridos na janela.

O buffer do ESP fica em RAM e guarda 72 janelas. Com janelas de 5 minutos, isso corresponde a 6 horas. A persistência definitiva fica no computador.

## Coletor Python

O computador **consulta o ESP**, em vez de o ESP enviar dados. Essa arquitetura é mais simples de recuperar após falhas: o ESP só mantém a fila recente, e o servidor Python coleta o que ainda não está no CSV.

Não há dependências externas de Python.

No Windows:

```bat
py grid_controller_web_server.py
```

Ou abra:

```text
iniciar_monitor.bat
```

O terminal mostrará o endereço do painel, por exemplo:

```text
http://192.0.2.10:8000
```

O CSV `grid_controller_stats.csv` usa `;` como separador e UTF-8 com BOM.

Exemplos:

```bat
py grid_controller_web_server.py --device http://grid-controller.local
py grid_controller_web_server.py --device http://192.0.2.10
py grid_controller_web_server.py --http-port 8080
```

O servidor reconstrói o horário de cada janela usando o uptime do ESP, remove duplicatas por `chip_id`, `session_id` e sequência, e preserva até 31 dias em memória para o navegador. O CSV não é apagado.

## Compilação e OTA

Escolha uma configuração de flash do ESP8266 que reserve espaço suficiente para OTA. O HTML está em PROGMEM e o firmware é relativamente grande.

Compile primeiro com as mesmas versões da InverterModbusLib, WiFiManager e modbus-esp8266 instaladas no computador. A API da InverterModbusLib ainda está em evolução; diferenças de versão podem exigir pequenos ajustes de assinatura.
