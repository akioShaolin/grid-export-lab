# Validação e roteiro humano

Nenhum teste físico foi executado nesta organização. O controle pode escrever limites reais; começar com barramento de inversores isolado e habilitar escrita somente após validar medição, sinal e escalas.

## Ordem recomendada

1. Revisar os arquivos privados de cada conjunto. Confirmar que `git check-ignore` cobre todos os privados; revisar EEPROM existente, hostname e credenciais persistidas. Não compartilhar saídas operacionais sem sanitizar.
2. Túnel: compilar ambas as pontas com o ambiente histórico, conferir TCP/1502, framing, CRC, reconexão, keepalive e ida/volta RS485 em bancada.
3. URP v3 + logger mirror-v3: validar identificação 0x0088, relações, escalas, potência/sinal e status 0x3000, com o baud configurado no relé.
4. URP v4 + loggers mirror-v4-basic/mirror-v4-frequency: validar status 0x0100, frequência, cosphi e CSV. Depois testar plot-mirror com o mesmo mapa.
5. URP v5 + logger snapshot: confirmar ENABLE_RTU=0 e natureza sintética do snapshot. Não usar para controle físico. Conferir offsets, flags e diagnóstico sem modificar o código histórico.
6. URP v6 + web-dashboard Python/HTML e plot-mirror: conferir espelho 0x0100/0x003C/0x02CD, default RTU 115200, idade/sequência, OTA, mDNS, persistência e recuperação. Avaliar loggers v4 apenas como candidatos pelo mapa.
7. station + controle.py: validar comandos com equipamento de bancada; o autor confirmou a leitura do logger conectado ao URP6000 e o envio de comandos ao logger dos inversores. Conferir se a versão do logger URP6000 expõe o snapshot 0x0200/32 esperado por este script; não usar v6 como substituto automático. Validar perda TCP, fallback, caminho normal/cache e CSV.
8. station2 + controle2.py: primeiro PING/GET/READPWR, depois SET em condições supervisionadas. Conferir ACTPWR, unidades, PI 0.25/0.005/0, anti-windup e temporizações preservadas. Conferir o mapa do logger URP6000 como no passo anterior.
9. Edge v1 + seu coletor/HTML: instalar dependência histórica registrada em references.md, conferir contratos da biblioteca, tentativa do ID 1 e máscara 0x07; não assumir integração bem-sucedida do SIW500H.
10. Edge v2 + seu coletor/HTML: manter URP v6 previamente validado, conferir máscara 0x06, IDs 2/3, potência nominal/ativa e escala de limite; liberar controle só depois das leituras.
11. Edge v3 + seu coletor/HTML: repetir v2; avaliar backoff, retirada/reconexão RS485, todos offline, critérios de reinício, retorno de comunicação e continuidade de estatísticas. Fazer ensaio prolongado, registrar falha/reinício sem atribuir causa não comprovada.
12. HTML isolado index.htm: identificar servidor WebSocket original antes de considerar um teste integrado.

Em cada passo registrar versão exata do Core, bibliotecas e Python; parâmetros privados fora do Git; resultados, erros e duração. Interromper controle quando medição ou comunicação não forem confiáveis. Não alterar dependências apenas para obter compilação.

## Resultado estático desta preparação

Resultados efetivamente executados são registrados no relatório de validação ao final da organização. py_compile verifica sintaxe, não importa dependências, não abre conexões e não comprova protocolo. Compilação Arduino só é aceitável se ambiente e versões históricos estiverem disponíveis; não foram instalados ou atualizados nesta preparação.

Resultado: 33 arquivos Python do produto passaram em py_compile com Python 3.13.3 (11 programas, 11 privados, 11 exemplos). Não há comprovação de build Arduino ou teste físico. Detalhes no [relatório de preparação](preparation-report.md).
