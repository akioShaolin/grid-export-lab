# Relatório de preparação — 2026-10-27

## Resultado

52 arquivos históricos preservados, com anonimização e configuração externalizada; 11 sketches e 11 programas Python de aplicação. Mapeamento arquivo a arquivo, hashes originais quando registrados e 14 exclusões estão em [source-inventory.md](source-inventory.md). Os hashes originais das duas pontas do túnel não foram registrados antes de remover a pasta antiga; não são apresentados como se fossem hashes dos fontes sanitizados.

Os cinco grupos finais são túnel, monitor URP6000, gateway de inversores, controlador host e controlador edge. Firmware station permanece em 03 e seu controle.py em 04; station2/controle2 seguem a mesma separação, documentada nos READMEs de ambos. Edge v1/v2/v3 mantêm firmware, header, coletor Python, HTML, batch e notas dentro da respectiva versão.

## Configuração local

22 arquivos locais ignorados: 11 headers para os sketches e 11 módulos para os scripts. Para cada um há um arquivo público `.example` correspondente. A lista abaixo mostra o arquivo privado; o exemplo insere `.example` antes da extensão.

- `01-rtu-tcp-tunnel/master_client/site_private.h`
- `01-rtu-tcp-tunnel/slave_server/site_private.h`
- `02-urp6000-monitor/gateway-v3/firmware/urp6000TCP/site_private.h`
- `02-urp6000-monitor/gateway-v4/firmware/urp6000TCP_v4/site_private.h`
- `02-urp6000-monitor/gateway-v5-experimental/firmware/urp6000TCP_v5/site_private.h`
- `02-urp6000-monitor/gateway-v6/firmware/urp6000TCP_v6/site_private.h`
- `02-urp6000-monitor/python-loggers/mirror-v3/site_private.py`
- `02-urp6000-monitor/python-loggers/mirror-v4-basic/site_private.py`
- `02-urp6000-monitor/python-loggers/mirror-v4-frequency/site_private.py`
- `02-urp6000-monitor/python-loggers/plot-mirror/site_private.py`
- `02-urp6000-monitor/python-loggers/snapshot-v5/site_private.py`
- `02-urp6000-monitor/web-dashboard/site_private.py`
- `03-inverter-gateway/v1-command-only/station/site_private.h`
- `03-inverter-gateway/v2-command-feedback/station2/site_private.h`
- `04-host-controller/v1-proportional/software/site_private.py`
- `04-host-controller/v2-pi/software/site_private.py`
- `05-edge-controller/v1-integrated/firmware/grid_controller_v1/site_private.h`
- `05-edge-controller/v1-integrated/software/site_private.py`
- `05-edge-controller/v2-two-siw400g/firmware/grid_controller_v2/site_private.h`
- `05-edge-controller/v2-two-siw400g/software/site_private.py`
- `05-edge-controller/v3-recovery-experiments/firmware/grid_controller_v3/site_private.h`
- `05-edge-controller/v3-recovery-experiments/software/site_private.py`

Credenciais e endereços originais foram mantidos apenas em arquivos locais ignorados quando possível. Identificadores que continham a identidade da instalação foram substituídos inclusive nesses arquivos; por isso as credenciais/defaults relacionados à identidade precisam ser configurados novamente antes de uso. Hosts privados preservados continuam dependendo da rede local. Nenhum programa passou a ler configuração C++ a partir de Python.

## Exclusões e mídia

Excluídos da árvore final: CSVs reais, diretório build com bin/elf/map e checksums dos artefatos antigos. PDFs de fabricantes ficaram fora do repositório. Não foi vendorizada a InverterModbusLib; os dois commits fornecidos pelo autor e suas linhas de desenvolvimento estão em [references.md](references.md).

Na preparação inicial, as 12 imagens ficaram fora para revisão. Posteriormente, após inspeção visual das cópias já tarjadas e autorização do autor, três capturas foram incluídas em `docs/images/dashboards`: gráfico de controle, painel online e painel offline. O mapeamento está no README dessa pasta. As outras nove imagens e os diagramas editáveis/previews de `sk1/` continuam fora. Nenhum arquivo fora de repo foi modificado.

## Validação executada

- Python 3.13.3: py_compile sem erros nos 11 programas, 11 configurações privadas e 11 exemplos (33 arquivos do produto). Helpers temporários foram removidos depois da execução.
- Imports de configuração: todos os nomes importados existem no módulo privado correspondente.
- Includes C++ locais: nenhum header referenciado faltando; todos os 11 nomes de sketch coincidem com sua pasta Arduino.
- Coletores Python: HTML referenciado por with_name encontrado ao lado em cada conjunto.
- git check-ignore: os 22 arquivos privados são ignorados. Exemplos permanecem visíveis ao Git.
- Busca literal dos valores operacionais privados nos textos públicos: zero ocorrências após excluir hostnames/defaults genéricos documentados.
- Busca recursiva sem distinção de caixa em texto e nomes: zero ocorrências do identificador original da instalação. Nenhum valor secreto foi incluído neste relatório.
- Referências de instalação nos textos públicos (redes originais, SSIDs e caminhos pessoais conhecidos): zero ocorrências. Ocorrências de termos password/senha remanescentes são parâmetros, macros, placeholders e documentação genérica.
- 192.168.4.1 foi preservado como AP padrão; 10.255.255.255 foi preservado como destino genérico de sondagem UDP para descobrir interface local, sem tráfego de aplicação/site. Loopback, bind 0.0.0.0, máscaras de rede e endereços RFC5737 também não são dados da instalação.
- Build Arduino não executado: PlatformIO está disponível, mas faltam confirmação das versões exatas e definição reproduzível do ambiente histórico. Nenhuma biblioteca foi instalada, atualizada ou substituída.
- Testes de rede, RTU, hardware e estabilidade prolongada não executados. Não foi provada correção do travamento RTU.

## Pendências antes de publicar

Licença MIT aplicada por instrução do titular: Copyright (c) 2025 Pedro Akio Sakuma. Bibliotecas e documentos externos mantêm suas próprias licenças.

O autor confirmou que os hosts liam o logger conectado ao URP6000 e enviavam comandos ao logger conectado aos inversores. A origem física das medidas está estabelecida. A versão exata do firmware URP6000 usada com os scripts snapshot ainda não está comprovada pelos fontes: o v5 encontrado tem esse formato, mas RTU desabilitado. O dashboard index.htm usa WebSocket e não tem servidor correspondente confirmado. READMEs antigos repetidos como v1 foram preservados em historical-notes para evitar falsificar a história; os novos READMEs descrevem as versões reais. APIs e formatos não foram unificados.

Ordem humana recomendada: túnel → URP v3/logger original → URP v4/loggers e gráfico → v5 sintético → v6/painel → station/controle.py → station2/controle2.py → edge v1 → edge v2 → edge v3. Conferir o mapa do firmware do logger URP6000 antes de reproduzir os conjuntos host. Checklist detalhado em [testing.md](testing.md).

Nenhum commit, push, tag ou release foi realizado. Os fontes estão na working tree e não foram adicionados ao index. Portanto git diff --stat está vazio: arquivos novos ainda são untracked. O status final foi registrado no terminal e abaixo.

```text
$ git diff --stat
(sem saída: nenhum arquivo rastreado)

$ git status --short
?? .gitignore
?? 01-rtu-tcp-tunnel/
?? 02-urp6000-monitor/
?? 03-inverter-gateway/
?? 04-host-controller/
?? 05-edge-controller/
?? LICENSE
?? README.md
?? data/
?? docs/
?? references/
```

Após remover os helpers e caches, a busca final não encontrou referências à instalação original nem arquivos CSV, bin, elf, map ou pyc remanescentes. Os arquivos privados continuam locais e ignorados, conforme solicitado.
