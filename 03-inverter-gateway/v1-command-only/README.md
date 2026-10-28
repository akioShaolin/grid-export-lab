# Conjunto host/gateway v1

Sem data de autoria comprovada; arquivos de log associados nomeados em março/2026, excluídos. Histórico.

- Finalidade: traduzir comandos de rede para RTU.
- Hardware: Solar View/ESP8266, WEG SIW400G ID 2; PC e gateway do relé.
- Entrada firmware: `/03-inverter-gateway/v1-command-only/station/station.ino`. Caminhos iniciados por `/` são relativos à raiz do repositório.
- Software correspondente: /04-host-controller/v1-proportional/software/controle.py
- Protocolo e portas: TCP/8899 linhas SET/SETPCT; medição snapshot 0x0200/32 via Modbus TCP/502.
- Limitações: Fluxo confirmado pelo autor: o computador lia os dados do logger conectado ao URP6000 e enviava comandos ao logger dos inversores. A versão exata do firmware do logger URP6000 usada com este script não está comprovada pelos fontes: ele espera snapshot 0x0200/32, presente no v5 encontrado, que está com RTU desabilitado. Conferir o mapa da versão usada ao reproduzir este conjunto. Proporcional normal 0.20; caminho de cache 0.6 preservado.
- Dependências: ESP8266 Core, modbus-esp8266; Python pymodbus (comentário recomenda 3.6.9) e zeroconf; versões restantes não identificadas

## Preparação e validação

Configuração operacional: revise o `site_private.h` do sketch e/ou `software/site_private.py` (ou o arquivo local junto ao script), ignorados pelo Git. Exemplos públicos ficam ao lado. Não copie configuração de outra geração sem conferir campos e endereços.

Build Arduino: não executado; ambiente histórico e dependências exatas não foram confirmados. Teste físico: pendente após organização. Consulte [roteiro central](../../docs/testing.md). Não foram alteradas versões de dependências nem corrigidas limitações históricas. Resultados Python estão no roteiro central.
