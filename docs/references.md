# Referências e proveniência

## InverterModbusLib

Dependência externa, não vendorizada: [akioShaolin/InverterModbusLib](https://github.com/akioShaolin/InverterModbusLib).

- Linha histórica informada pelo autor: `refactor/maps-descriptors`, commit [`15a89ec7efc216d9c2e5de594ac8cc693aba8aeb`](https://github.com/akioShaolin/InverterModbusLib/commit/15a89ec7efc216d9c2e5de594ac8cc693aba8aeb).
- Evolução posterior informada pelo autor: `refactor/nonblocking-modbus`, referência [`767c41668efe723af8a601e37a6befd1b1fc53a1`](https://github.com/akioShaolin/InverterModbusLib/commit/767c41668efe723af8a601e37a6befd1b1fc53a1).

A segunda linha evolui a anterior e foi motivada em parte pelas limitações reveladas no controlador integrado. Esses hashes foram fornecidos para esta organização; os fontes locais não comprovam qual hash estava instalado em cada teste. As URLs foram tentadas, mas a consulta externa não conseguiu recuperar seu conteúdo. Não instalar a linha nova para compilar ou testar a história antiga. Patches opcionais nos conjuntos v1/v2 são fragmentos históricos, não uma cópia completa da biblioteca, e não foram aplicados.

## Documentação de fabricantes

- PEXTRON URP6000/URP6001, comunicação serial: [pacote oficial indicado pelo autor](https://www.pextron.com/wp-content/uploads/2026/05/URP-6000_Manuais.zip). Fonte local inclui capítulo 20 do URP600X v9.68 r01. Usado para RTU, identificação 0x0088, medição e configuração RS485. O download externo não pôde ser verificado nesta sessão.
- WEG: `MANUAL_DE_COMUNICACAO_SIW400G_T075_T100_W0_web.pdf`, fonte local não redistribuída; cobre funções de habilitação/valor do limite de potência ativa. Não foi encontrada URL oficial direta verificável durante esta organização.
- Declaração WEG de equivalência, arquivo local `Equivalência Inversores Completa.rev27.pdf`: segundo informação do autor, SIW500H ST030 M3 corresponde a Huawei SUN2000-30KTL-M3. A validade atual da declaração não foi verificada; não é apresentada como documento vigente.
- Huawei: Solar Inverter Modbus Interface Definitions, V3.0, Issue 02, 2021-03-26, referência bibliográfica fornecida pelo autor; PDF local não redistribuído. Confirmar edição na cópia original antes de usar como especificação.

Sem licença explícita de redistribuição, nenhum PDF de fabricante foi incorporado. Versões exatas de ESP8266 Core, modbus-esp8266 e WiFiManager precisam ser recuperadas do ambiente histórico. O comentário do logger v5 e dos hosts recomenda pymodbus 3.6.9; isso não garante que todos os outros scripts usaram a mesma versão.
