# URP6000 gateway v6

Sem data comprovada. Histórico.

- Finalidade: monitorar o relé ou experimentar transporte de medidas.
- Hardware: Solar View/ESP8266 + PEXTRON URP6000-5A.
- Entrada firmware: `firmware/urp6000TCP_v6/urp6000TCP_v6.ino`. Caminhos iniciados por `/` são relativos à raiz do repositório.
- Software correspondente: web-dashboard/urp6000_web_server.py + urp6000_mobile.html; plot-mirror e loggers de espelho v4 são candidatos por mapa, exigem teste
- Protocolo e portas: Modbus TCP/502. Espelho status 0x0100, diagnóstico ampliado; HTTP/80 e OTA /update.
- Limitações: Escalas, sequenciamento e configuração serial precisam de validação com o relé.
- Dependências: ESP8266 Core e modbus-esp8266 (ModbusRTU/ModbusIP_ESP8266); versões não identificadas

## Preparação e validação

Configuração operacional: revise o `site_private.h` do sketch e/ou `software/site_private.py` (ou o arquivo local junto ao script), ignorados pelo Git. Exemplos públicos ficam ao lado. Não copie configuração de outra geração sem conferir campos e endereços.

Build Arduino: não executado; ambiente histórico e dependências exatas não foram confirmados. Teste físico: pendente após organização. Consulte [roteiro central](../../docs/testing.md). Não foram alteradas versões de dependências nem corrigidas limitações históricas. Resultados Python estão no roteiro central.
