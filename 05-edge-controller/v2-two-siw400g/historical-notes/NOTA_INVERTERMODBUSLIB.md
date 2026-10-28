# Pontos a conferir na InverterModbusLib

O projeto depende da versão da biblioteca usada no `FullWebPanel.ino` enviado, na qual `getRatedPower()`, `getActivePower()` e `setPowerLimit()` são chamados pela API pública.

## 1. Assinatura de `getRatedPower()`

O sketch espera:

```cpp
bool getRatedPower(uint32_t& watts);
```

Se a versão instalada ainda não declarar esse método em `Inverter.h`, atualize a biblioteca antes de compilar. Como alternativa temporária, seria possível usar `getDescriptor(model).nominalPowerW`, mas isso deixaria de cumprir a exigência de obter a potência por `getRatedPower()`.

## 2. Condição suspeita em `setPowerLimit()`

Em uma versão analisada de `InverterControl.cpp`, apareceu:

```cpp
if (_map.PowerLimit.writable && _map.PowerLimit.scale == 0.0f) {
    return writeField(_map.PowerLimit, watts / _map.PowerLimit.scale);
}
```

Essa condição permite divisão por zero e provavelmente deveria ser:

```cpp
if (_map.PowerLimit.writable && _map.PowerLimit.scale != 0.0f) {
    return writeField(_map.PowerLimit, watts / _map.PowerLimit.scale);
}
```

O caminho por `PowerLimitPercent` já verifica escala diferente de zero. Confirme qual caminho os mapas `SIW500H` e `SIW400G_T050_T100` utilizam antes de controlar os inversores.

## 3. Habilitação do limite

Na versão analisada, `setPowerLimitEnabled()` aparece como método privado. Por isso o sketch não o chama. Valide se os inversores aceitam `setPowerLimit()` diretamente ou se a própria biblioteca deve habilitar o modo de limite internamente.

## 4. Operações síncronas

Os métodos da biblioteca aguardam a callback do Modbus dentro de um laço com `mb.task()` e timeout. O sketch faz escalonamento cooperativo, mas uma chamada ainda pode bloquear até o timeout. Uma API assíncrona futura eliminaria essa limitação.

## 5. Timeout efetivo

O `#define MODBUSRTU_TIMEOUT 350` reduz o timeout interno da `modbus-esp8266`, mas a versão analisada de `InverterCore.cpp` ainda possui um laço de espera de até `1000 ms` em cada método público. Portanto, o tempo máximo percebido pelo sketch pode continuar próximo de 1 s por operação com falha. Para um RTU realmente não bloqueante e timeout integralmente configurável, a biblioteca precisa expor uma operação assíncrona (`start/poll/callback`) ou usar o timeout de `ModbusConfig` no laço de espera.
