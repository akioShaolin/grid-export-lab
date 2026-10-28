# Edge v2

Sem data comprovada. Firmware 1.0.4. Histórico/experimental.

- Finalidade: integrar medição, PI, comunicação e estatísticas na borda.
- Hardware: Solar View/ESP8266, relé via gateway v6; IDs ativos 2 e 3 (0x06).
- Entrada firmware: `firmware/grid_controller_v2/grid_controller_v2.ino`. Caminhos iniciados por `/` são relativos à raiz do repositório.
- Software correspondente: software/grid_controller_web_server.py + software/grid_controller_mobile.html; batch na mesma pasta.
- Protocolo e portas: Modbus TCP/502 espelho v6; RTU/9600 8N1; HTTP/80 /api/status e /api/stats; coletor HTTP/8000 /api/history e /api/latest.
- Limitações: Falha RTU persistente relatada; causa não comprovada. ID 1 não integrado com sucesso. 
- Dependências: ESP8266 Core, WiFiManager de tzapu, modbus-esp8266 de Alexander Emelianov e InverterModbusLib maps-descriptors (ver docs/references.md). Coletor Python: biblioteca padrão.

## Preparação e validação

Configuração operacional: revise o `site_private.h` do sketch e/ou `software/site_private.py` (ou o arquivo local junto ao script), ignorados pelo Git. Exemplos públicos ficam ao lado. Não copie configuração de outra geração sem conferir campos e endereços.

Build Arduino: não executado; ambiente histórico e dependências exatas não foram confirmados. Teste físico: pendente após organização. Consulte [roteiro central](../../docs/testing.md). Não foram alteradas versões de dependências nem corrigidas limitações históricas. Resultados Python estão no roteiro central.
