# Validação de bancada — grid_controller_v1

Este firmware controla equipamentos reais. Faça a primeira validação em bancada e com o controle PI desabilitado. Atividades em instalações de média tensão devem seguir os procedimentos da empresa e ser executadas somente por pessoal autorizado, sem usar este documento como procedimento de segurança elétrica.

## 1. Compilação

Confirme que o sketch compila com:

- a versão atual da InverterModbusLib;
- WiFiManager compatível com modo não bloqueante e porta configurável;
- modbus-esp8266;
- ESP8266 Arduino Core.

Confirme também que não há uso de `Serial.print()` no sketch.

## 2. Rede e recuperação

Com o RS485 desconectado:

1. Energize o ESP.
2. Confirme a criação do AP `grid_controller-XXXXXX` quando não houver rede configurada.
3. Configure o Wi-Fi pelo portal em `192.168.4.1:8080`.
4. Confirme `ping grid-controller.local`.
5. Abra `/`, `/config`, `/api/status` e `/update`.
6. Acione o botão de AP temporário e confirme que o painel principal continua respondendo.
7. Reinicie e confirme a persistência das configurações.

Critério: o ESP deve voltar sozinho após queda e retorno do Wi-Fi, sem regravação.

## 3. URP6000 sem controle

Mantenha o PI desabilitado.

Confirme na API:

```text
meter_valid = true
meter_seq avançando
meter_age_s dentro do limite
meter_raw_w coerente
meter_filtered_w coerente
```

Compare o valor com o painel Python já validado. Teste as duas opções de sinal antes de escolher a definitiva.

Critério: nenhuma amostra deve ser aceita quando o `SEQ` muda no meio da leitura, fica parado além do limite ou quando `age` fica antigo.

## 4. Inversores, um por vez

Valide cada endereço separadamente antes de manter os três no mesmo barramento:

```text
ID 1 — SIW500H_ST030_M3
ID 2 — SIW400G_T100_W0
ID 3 — SIW400G_T100_W0
```

Para cada um, confira:

- `getRatedPower()` retorna a potência nominal esperada;
- `getActivePower()` retorna watts e sinal coerentes;
- o equipamento permanece online em várias leituras;
- timeout curto não reinicia o ESP;
- o contador de erros aumenta quando a comunicação é interrompida;
- o objeto tenta se recuperar sem reinicialização manual.

Critério esperado das potências nominais:

```text
ID 1: aproximadamente 30 kW
ID 2: aproximadamente 100 kW
ID 3: aproximadamente 100 kW
```

## 5. Validação de `setPowerLimit()`

Antes do PI, confirme no equipamento e na biblioteca:

- unidade em watts;
- escala do registrador;
- necessidade ou não de habilitar previamente o modo de limite;
- valor mínimo aceito;
- tempo entre comando e resposta física;
- comportamento ao receber o mesmo valor repetidamente.

A versão atual do sketch usa apenas `setPowerLimit()`. Se algum mapa exigir uma habilitação separada, essa regra deve ser resolvida na InverterModbusLib antes de habilitar o PI.

## 6. Distribuição proporcional

Com os três inversores disponíveis, o comando total deve ser dividido aproximadamente na proporção:

```text
30 : 100 : 100
```

Se um inversor perder feedback, ele deve sair da distribuição e os demais devem receber o novo rateio dentro de seus limites.

Critério: a soma dos comandos individuais deve acompanhar o comando total e nunca ultrapassar a capacidade configurada.

## 7. PI

Somente depois das etapas anteriores:

1. Mantenha o limite máximo conservador.
2. Confirme novamente o sinal do erro.
3. Habilite o controle com ganhos baixos e observe sem mudanças bruscas.
4. Verifique a banda morta, rampa, saturação e termo integral.
5. Registre potência do URP, potência dos inversores, erro e comando.

Interrompa o ensaio se o comando caminhar na direção oposta ao necessário, se houver oscilação crescente ou se os equipamentos não confirmarem os limites.

## 8. Failsafe

Com o sistema em condição controlada, simule separadamente:

- perda do URP TCP;
- `age` antigo;
- `SEQ` parado;
- perda do Wi-Fi;
- perda de cada inversor.

Critérios:

- falha do medidor: após o atraso configurado, aplicar o percentual de failsafe;
- falha de um inversor: continuar com os demais;
- retorno do medidor: zerar o integrador antes de retomar;
- retorno de inversor: obter novamente potência nominal e feedback antes de reinseri-lo no PI.

## 9. Estatísticas e Python

Deixe o ESP rodar por mais de uma janela estatística e confirme:

```text
/api/stats?after=0&limit=72
```

Depois execute:

```bat
py grid_controller_web_server.py
```

Confirme:

- criação de `grid_controller_stats.csv`;
- uma linha por janela;
- ausência de duplicatas após reiniciar apenas o Python;
- nova `session_id` após reiniciar o ESP;
- gráfico com média, mínimo, máximo e setpoint;
- recuperação após o computador ficar temporariamente desligado por menos que a retenção do ESP.

## 10. Antes da instalação definitiva

Registre as versões exatas das bibliotecas, exporte o binário validado, guarde uma cópia das configurações e mantenha o PI inicialmente desabilitado após qualquer alteração relevante de firmware ou mapa Modbus.
