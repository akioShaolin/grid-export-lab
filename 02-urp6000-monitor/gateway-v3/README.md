# URP6000 gateway v3

Sem data comprovada. Histórico.

- Finalidade: monitorar o relé ou experimentar transporte de medidas.
- Hardware: Solar View/ESP8266 + PEXTRON URP6000-5A.
- Entrada firmware: `firmware/urp6000TCP/urp6000TCP.ino`. Caminhos iniciados por `/` são relativos à raiz do repositório.
- Software correspondente: python-loggers/mirror-v3/urp6000_log.py
- Protocolo e portas: Modbus TCP/502. Status 0x3000, espelho de relações/medidas/potência.
- Limitações: Escalas, sequenciamento e configuração serial precisam de validação com o relé.
- Dependências: ESP8266 Core e modbus-esp8266 (ModbusRTU/ModbusIP_ESP8266); versões não identificadas

## Preparação e validação

Configuração operacional: revise o `site_private.h` do sketch e/ou `software/site_private.py` (ou o arquivo local junto ao script), ignorados pelo Git. Exemplos públicos ficam ao lado. Não copie configuração de outra geração sem conferir campos e endereços.

Build Arduino: não executado; ambiente histórico e dependências exatas não foram confirmados. Teste físico: pendente após organização. Consulte [roteiro central](../../docs/testing.md). Não foram alteradas versões de dependências nem corrigidas limitações históricas. Resultados Python estão no roteiro central.
