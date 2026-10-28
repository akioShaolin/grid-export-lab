# Hardware

- Solar View reaproveitado com ESP8266/ESP-07 e transceptor RS485.
- GPIO12 controla DE/RE; GPIO13 seleciona half/full duplex, mantido LOW nos sketches. Confirmar revisão física antes de energizar.
- UART0 é usada no barramento: não adicionar Serial.print enquanto conectada ao RS485.
- Relé PEXTRON URP6000-5A; firmware lê identificação em 0x0088, relações em 0x003C, medidas em 0x02BC e potência em 0x02CD. Inicialização, escala e baud rate devem ser conferidos por geração; v6 tem default RTU de 115200, distinto dos inversores em 9600 8N1.
- ID 2 e ID 3: WEG SIW400G T100 W0, foco dos testes posteriores.
- ID 1: WEG SIW500H ST030 M3, tentativa histórica sem integração bem-sucedida no período descrito pelo autor. v1 espera máscara 0x07; v2/v3 ativam 0x06.
- Há fotografias externas de antena Wi-Fi e montagem magnética/neodímio. Aguardam revisão visual e não foram copiadas.

Baud, paridade, polaridade, terminação, terra de referência, identificação dos dispositivos e isolamento devem ser verificados fisicamente. Este acervo não estabelece uma instalação elétrica de referência.
