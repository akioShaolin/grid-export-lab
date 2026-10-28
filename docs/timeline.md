# Evolução verificada

Segundo lembrança posterior do autor, o logger conectado ao URP6000 usava a última versão e não precisou de novas alterações depois de estabilizar. No acervo organizado, a última versão numerada é a v6. Essa associação é um relato do autor, ainda sem confirmação do binário instalado. Os scripts host preservados esperam snapshot em 0x0200, enquanto o v6 encontrado expõe o espelho descrito em architecture.md; essa diferença permanece documentada para reprodução histórica.

A ordem abaixo é técnica, sustentada pelo conteúdo dos fontes. Datas de cópia do workspace não foram tratadas como datas de desenvolvimento.

| Fase | Evidência | Data e limites |
|---|---|---|
| Transporte RTU/TCP | master_client/slave_server v3, framing A5 5A e keepalives | Sem data interna comprovada |
| Monitoramento URP6000 | v3 espelho 0x3000; v4 espelho 0x0100 e frequência; v5 snapshot experimental com ENABLE_RTU=0; v6 firmware 1.0.1, OTA/EEPROM/mDNS | Numeração não implica melhoria linear |
| Gateway de inversores | station aplica limite; station2 adiciona READPWR | Sem data interna comprovada |
| Controle no PC | controle.py proporcional; controle2.py PI, Kp=0.25, Ki=0.005, Kd=0 | Nomes dos CSVs excluídos indicam registros em março de 2026, não datas de autoria |
| Edge v1 | FW_VERSION 1.0.3, máscara esperada 0x07 | Inclui tentativa com três inversores; ID 1 não integrado com sucesso segundo relato do autor |
| Edge v2 | FW_VERSION 1.0.4, máscara ativa 0x06 | Dois SIW400G, IDs 2 e 3 |
| Edge v3 | FW_VERSION 1.1.0-safe, backoff e critérios de recuperação/reinício | Experimento; o sufixo safe é um rótulo histórico, não certificação |

Os nomes das imagens externas sugerem registros em fevereiro, julho e agosto de 2026; não foram importadas nem usadas para provar o funcionamento de uma versão específica. Os READMEs antigos das versões integradas repetem o título v1: foram preservados como notas históricas e não prevalecem sobre os fontes ou sobre os READMEs novos.

Segundo relato fornecido pelo autor, o controle de dois SIW400G funcionou em períodos de campo e depois houve falha persistente de RTU, recuperada com reinício do ESP. A observação motivou a linha não bloqueante da biblioteca. Não há evidência suficiente para atribuir os períodos de campo a um hash exato deste acervo.
