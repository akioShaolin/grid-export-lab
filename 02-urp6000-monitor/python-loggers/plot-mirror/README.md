# Logger/visualização plot-mirror

Sem data comprovada; histórico. urp6000_grap_v1.py → mapa espelho v4/v6, status 0x0100 e potência 0x02CD; associação por formato, sem confirmação em hardware.

Hardware: PC com Python e ESP8266/URP6000. Entrada é o script `.py` desta pasta; firmware na versão indicada acima. Modbus TCP/502; CSVs mantêm cabeçalhos, escalas e delimitadores históricos. Dependências: pymodbus, versão não comprovada; matplotlib para gráfico. Recomendação 3.6.9 aparece especificamente no logger snapshot. Não atualizar dependências para uniformizar versões.

Configuração local: site_private.py; exemplo ao lado. Teste físico e de runtime pendente. Ver [validação](../../../docs/testing.md).
