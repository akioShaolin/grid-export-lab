# Túnel RTU/TCP v3

Sem data comprovada. Histórico.

- Finalidade: transportar RTU pela rede.
- Hardware: Duas placas Solar View/ESP8266, UART0/RS485 9600.
- Entrada firmware: `master_client/master_client.ino e slave_server/slave_server.ino`. Caminhos iniciados por `/` são relativos à raiz do repositório.
- Software correspondente: Não há Python associado.
- Protocolo e portas: TCP/1502 binário A5 5A; HTTP/80.
- Limitações: Necessita as duas pontas; não é Modbus TCP padrão.
- Dependências: ESP8266 Arduino Core, ESP8266WiFi, ESP8266WebServer

## Preparação e validação

Configuração operacional: revise o `site_private.h` do sketch e/ou `software/site_private.py` (ou o arquivo local junto ao script), ignorados pelo Git. Exemplos públicos ficam ao lado. Não copie configuração de outra geração sem conferir campos e endereços.

Build Arduino: não executado; ambiente histórico e dependências exatas não foram confirmados. Teste físico: pendente após organização. Consulte [roteiro central](../docs/testing.md). Não foram alteradas versões de dependências nem corrigidas limitações históricas. Resultados Python estão no roteiro central.
