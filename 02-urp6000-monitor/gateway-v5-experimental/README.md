# URP6000 gateway v5

Sem data comprovada. Experimental.

- Finalidade: monitorar o relé ou experimentar transporte de medidas.
- Hardware: Solar View/ESP8266 + PEXTRON URP6000-5A.
- Entrada firmware: `firmware/urp6000TCP_v5/urp6000TCP_v5.ino`. Caminhos iniciados por `/` são relativos à raiz do repositório.
- Software correspondente: python-loggers/snapshot-v5/urp6000_log_v5.py (compatibilidade de formato parcial)
- Protocolo e portas: Modbus TCP/502. Snapshot 0x0200/32; diagnóstico 0x0100/64.
- Limitações: ENABLE_RTU=0; dados de teste, não usar como medição do relé.
- Dependências: ESP8266 Core e modbus-esp8266 (ModbusRTU/ModbusIP_ESP8266); versões não identificadas

## Preparação e validação

Configuração operacional: revise o `site_private.h` do sketch e/ou `software/site_private.py` (ou o arquivo local junto ao script), ignorados pelo Git. Exemplos públicos ficam ao lado. Não copie configuração de outra geração sem conferir campos e endereços.

Build Arduino: não executado; ambiente histórico e dependências exatas não foram confirmados. Teste físico: pendente após organização. Consulte [roteiro central](../../docs/testing.md). Não foram alteradas versões de dependências nem corrigidas limitações históricas. Resultados Python estão no roteiro central.
