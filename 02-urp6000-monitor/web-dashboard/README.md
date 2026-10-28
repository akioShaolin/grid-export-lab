# Painel de monitoramento do relé

Histórico, sem data comprovada. Entrada: urp6000_web_server.py, com urp6000_mobile.html ao lado. Python/pymodbus no PC lê espelho 0x0100, 0x003C e 0x02CD por TCP/502; serve histórico e estado HTTP/8000 (/api/history, /api/latest, /api/state). Usa o mapa de v4/v6, sem confirmação física nesta organização. Não requer nem interpreta a API HTTP do edge. Dependência exata de pymodbus não identificada. Configuração site_private.py; exemplo ao lado. Teste de runtime e hardware pendente; validação sintática no roteiro central.
