# Logger/visualização snapshot-v5

Sem data comprovada; histórico. urp6000_log_v5.py → formato snapshot 0x0200/32 do v5; snapshot experimental sem RTU. Firmware de campo exato incerto.

Hardware: PC com Python e ESP8266/URP6000. Entrada é o script `.py` desta pasta; firmware na versão indicada acima. Modbus TCP/502; CSVs mantêm cabeçalhos, escalas e delimitadores históricos. Dependências: pymodbus, versão não comprovada. Recomendação 3.6.9 aparece especificamente no logger snapshot. Não atualizar dependências para uniformizar versões.

Configuração local: site_private.py; exemplo ao lado. Teste físico e de runtime pendente. Ver [validação](../../../docs/testing.md).
