#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
URP6000 Tank RTU Logger (Modbus TCP -> snapshot 0x0200)

- tolerante a jitter
- reconecta sempre
- imprime valores convertidos (Ia/Ib/Ic, Va/Vb/Vc, fL, cos, Pt, RTP/RTC)
- NÃO usa frequência de barramento

Requisitos:
  py -m pip install pymodbus==3.6.9   (recomendado)
  (ou qualquer pymodbus, o wrapper abaixo tenta ser compatível)

Uso:
  py controle2.py --setpoint 179000 --inv_pot 100000 --poll 1.0 --control_period 2.0
pi
"""

from site_private import URP_HOST, INVERTER_MDNS_NAME
import os
import time
import csv
import socket
import argparse
import struct
import sys
import threading
from datetime import datetime
from collections import deque
from typing import Optional
from zeroconf import ServiceBrowser, ServiceListener, Zeroconf

from pymodbus.client import ModbusTcpClient

SERVICE_TYPE = "_inverterctrl._tcp.local."
TCP_PORT = 8899
KEEPALIVE_S = 1.0
DISCOVERY_TIMEOUT_S = 8.0
RECONNECT_DELAY_S = 2.0

REG_SNAP_START = 0x0200
REG_SNAP_LEN   = 32
REG_DIAG_START = 0x0100
REG_DIAG_LEN   = 0x20
                                       
setpoint_default = 180000.0 #Setpoint em kilowatts, pode ser ajustado conforme necessário
inv_timeout_s = 20.0 # Tempo para considerar o inversor offline e parar de enviar comandos (em segundos)
hold_last_ok_s = 30.0 # Tempo para manter o último valor válido do setpoint mesmo se o medidor estiver offline (em segundos)

kp = 0.25 # Ganho do controlador proporcional (ajuste conforme necessário, cuidado para não ser muito alto e causar oscilações)
ki = 0.005 # Ganho do controlador integral (pode ser adicionado depois se necessário, cuidado para não ser muito alto e causar oscilações)
kd = 0.0 # Ganho do controlador derivativo (pode ser adicionado depois se necessário, cuidado para não ser muito alto e causar oscilações)

deadband_w = 1000.0 # 1 kW de banda morta para evitar oscilações

# anti wind-up

integral_min = -100000.0
integral_max = 100000.0

class MdnsListener(ServiceListener):
    def __init__(self, target_name: Optional[str] = None):
        self.ip = None
        self.port = None
        self.name = None
        self.target_name = target_name.lower() if target_name else None

    def update_service(self, zc, type_, name):
        self.add_service(zc, type_, name)

    def remove_service(self, zc, type_, name):
        pass

    def add_service(self, zc, type_, name):
        if self.target_name and self.target_name not in name.lower():
            return
        info = zc.get_service_info(type_, name, timeout=3000)
        if not info or not info.addresses:
            return
        self.ip = socket.inet_ntoa(info.addresses[0])
        self.port = info.port or TCP_PORT
        self.name = name

def discover_ip(target_name: Optional[str] = None, timeout_s: float = DISCOVERY_TIMEOUT_S):
    zc = Zeroconf()
    listener = MdnsListener(target_name=target_name)
    ServiceBrowser(zc, SERVICE_TYPE, listener)

    t0 = time.time()
    try:
        while time.time() - t0 < timeout_s:
            if listener.ip:
                return listener.ip, listener.port, listener.name
            time.sleep(0.1)
    finally:
        zc.close()
    return None, None, None

class InverterClient:
    def __init__(self, target_name: Optional[str] = None, default_value: float = setpoint_default):
        self.target_name = target_name
        self.default_value = default_value
        self.sock = None
        self.ip = None
        self.port = TCP_PORT
        self.lock = threading.Lock()
        self.last_set = None
        self.stop_flag = False
        self.last_set = None
        self.last_set_time = 0.0
        self.last_set_response = ""

    def discover_and_connect(self):
        while not self.stop_flag:
            print("[mDNS] Procurando dispositivo...")
            ip, port, name = discover_ip(self.target_name)
            if not ip:
                print("[mDNS] Nenhum dispositivo encontrado. Nova tentativa em 2 s.")
                time.sleep(RECONNECT_DELAY_S)
                continue

            print(f"[mDNS] Encontrado: {name} -> {ip}:{port}")
            try:
                sock = socket.create_connection((ip, port), timeout=5)
                sock.settimeout(3)
                hello = sock.recv(256).decode(errors="ignore").strip()
                if hello:
                    print(f"[TCP] {hello}")
                self.sock = sock
                self.ip = ip
                self.port = port
                print(f"[TCP] Conectado em {ip}:{port}")
                return
            except OSError as e:
                print(f"[TCP] Falha ao conectar: {e}")
                time.sleep(RECONNECT_DELAY_S)

    def disconnect(self):
        with self.lock:
            if self.sock:
                try:
                    self.sock.close()
                except OSError:
                    pass
            self.sock = None
            self.ip = None

    def send_line(self, line: str) -> str:
        with self.lock:
            if not self.sock:
                raise ConnectionError("socket fechado")
            self.sock.sendall((line.strip() + "\n").encode())
            data = self.sock.recv(256)
            if not data:
                raise ConnectionError("sem resposta")
            return data.decode(errors="ignore").strip()

    def set_power(self, value: int):
        resp = self.send_line(f"SET {value}")
        print(f"[SET] {value} -> {resp}")
        self.last_set_response = resp

        if resp == "OK":
            self.last_set = value
            self.last_set_time = time.time()

        else:
            self.last_set_response = f"erro: {resp}"

    def keepalive_loop(self):
        while not self.stop_flag:
            try:
                if not self.sock:
                    self.discover_and_connect()
                    continue

                if (time.time() - self.last_set_time) > KEEPALIVE_S:
                    resp = self.send_line("PING")
                    # print(f"[PING] {resp}")
                
                time.sleep(KEEPALIVE_S)

            except (OSError, ConnectionError) as e:
                print(f"[TCP] Conexao perdida: {e}")
                self.disconnect()
                time.sleep(RECONNECT_DELAY_S)

    def start(self, initial_value: Optional[int] = None):
        t = threading.Thread(target=self.keepalive_loop, daemon=True)
        t.start()

        while not self.stop_flag and not self.sock:
            time.sleep(0.2)

        if initial_value is not None and self.sock:
            try:
                self.set_power(initial_value)
            except Exception as e:
                print(f"[SET inicial] erro: {e}")

    def read_power(self):
        try:
            resp = self.send_line("READPWR")

            if resp.startswith("ACTPWR"):
                parts = resp.split()
                power = int(parts[1])
                return power * 100.0  # converter de 0.01 kW para W
            
            return None
        except Exception as e:
            print(f"[READPWR] erro: {e}")
            return None

def csv_filename():
    return f"log_export_{datetime.now().strftime('%Y%m%d')}.csv"

def ensure_csv_header(path):
    file_exists = os.path.exists(path)

    if not file_exists or os.path.getsize(path) == 0:
        with open(path, "a", newline="", encoding="utf-8") as f:
            w = csv.writer(f, delimiter=';')
            w.writerow([
                "data_hora",
                "Ia_A",
                "Ib_A",
                "Ic_A",
                "Va_V",
                "Vb_V",
                "Vc_V",
                "fL_Hz",
                "cosA",
                "cosB",
                "cosC",
                "Pt_W",
                "SP_W",
                "Erro_W",
                "pot_inv_atual_W",
                "limite_atual_W",
                "novo_limite_W",
                "cmd_pct",
                "cmd_raw",
                "resposta_set",
            ])

def append_csv_row(path, row):
    with open(path, "a", newline="", encoding="utf-8") as f:
        w = csv.writer(f, delimiter=';')
        w.writerow(row)

def fmt_dt() -> str:
    return datetime.now().strftime("%d/%m/%Y %H:%M:%S")

def cosphi_from_raw(raw: int) -> float:
    # igual ao script anterior
    if raw >= 39936:
        return (raw - 65536) / 256.0
    return raw / 256.0

def i32_from_regs_be(hi: int, lo: int) -> int:
    raw = struct.pack(">HH", hi & 0xFFFF, lo & 0xFFFF)
    return struct.unpack(">i", raw)[0]

def read_holding(client: ModbusTcpClient, address: int, count: int, unit: int = 1):
    """
    Wrapper compatível com várias versões do pymodbus.
    """
    try:
        rr = client.read_holding_registers(address=address, count=count, slave=unit)
    except TypeError:
        try:
            rr = client.read_holding_registers(address=address, count=count, unit=unit)
        except TypeError:
            rr = client.read_holding_registers(address=address, count=count)

    if rr is None:
        raise IOError("Resposta None")
    if hasattr(rr, "isError") and rr.isError():
        raise IOError(str(rr))
    regs = getattr(rr, "registers", None)
    if regs is None or len(regs) != count:
        raise IOError(f"len(registers)={0 if regs is None else len(regs)}")
    return regs

def should_reconnect(msg: str) -> bool:
    m = (msg or "").lower()
    hard = [
        "winerror 10061",
        "refused",
        "reset by peer",
        "broken pipe",
        "not connected",
        "connection aborted",
        "timed out",
        "timeout",
        "0 received",
        "expected at least 8 bytes",
        "invalid message",
    ]
    return any(k in m for k in hard)

def connect(ip: str, port: int, timeout_s: float) -> ModbusTcpClient:
    client = ModbusTcpClient(host=ip, port=port, timeout=timeout_s, retries=0)
    if not client.connect():
        raise OSError("connect() retornou False")

    # keepalive best-effort
    try:
        sock = client.socket
        if sock is not None:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            if hasattr(socket, "SIO_KEEPALIVE_VALS"):
                sock.ioctl(socket.SIO_KEEPALIVE_VALS, (1, 30000, 5000))
    except Exception:
        pass

    return client

def clamp(v, vmin, vmax):
    return max(vmin, min(vmax, v))

def watts_to_pct(power_w: float, inv_pot: float) -> int:
    pct = (power_w / inv_pot) * 100.0
    return int(round(clamp(pct, 0.0, 100.0) * 10.0))

def pct_to_watts(pct: int, inv_max_w: float) -> float:
    return (pct / 1000.0) * inv_max_w

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", default=URP_HOST)
    ap.add_argument("--port", type=int, default=502)
    ap.add_argument("--poll", type=float, default=1.0)
    ap.add_argument("--timeout", type=float, default=10.0)
    ap.add_argument("--diag_every", type=int, default=2)
    ap.add_argument("--setpoint", nargs="?", type=float, required=True, default=setpoint_default, help="valor inicial de 0 a 300000kW")
    ap.add_argument("--inv_pot", type=float, required=True, help="potência máxima do inversor em Watts (ex: 100000 para 100kW)")
    ap.add_argument("--name", default=INVERTER_MDNS_NAME, help="parte do nome mDNS a procurar")
    ap.add_argument("--control_period", type=float, default=10.0, help="período mínimo entre comandos de controle (s)")
    args = ap.parse_args()

    client_inv = InverterClient(target_name=args.name, default_value=args.setpoint)
    client_inv.start()

    client = None
    fail_streak = 0
    last_ok_ts = 0.0
    backoff = 1.0
    lat_hist = deque(maxlen=20)

    last_snap = None
    last_diag = None
    last_data_ts = 0.0
    reading_fault_active = False

    last_control_ts = None
    integral_ws = 0.0

    next_t = time.time() + args.poll
    cycle = 0

    csv_path = csv_filename()
    ensure_csv_header(csv_path)

    
# Loop principal de leitura e controle
    while True:
        now = time.time()
        if now < next_t:
            time.sleep(0.01)
            continue
        next_t = max(next_t + args.poll, now + args.poll)
        cycle += 1

        if last_control_ts is None:
            dt = args.control_period
        else:
            dt = now - last_control_ts

        last_control_ts = now

        if client is None:
            try:
                client = connect(args.ip, args.port, args.timeout)
                print(f"[{fmt_dt()}] Conectado em {args.ip}:{args.port}")
                time.sleep(0.25)
                fail_streak = 0
                backoff = 1.0
            except Exception:
                print(f"[{fmt_dt()}] TCP offline ({args.ip}:{args.port}). Aguardando...")
                time.sleep(backoff)
                backoff = min(backoff * 1.4, 15.0)
                continue

        t0 = time.time()
        try:
            snap = read_holding(client, REG_SNAP_START, REG_SNAP_LEN)
            diag = last_diag
            if (cycle % max(1, args.diag_every)) == 0:
                diag = read_holding(client, REG_DIAG_START, REG_DIAG_LEN)

            lat_ms = int((time.time() - t0) * 1000)
            lat_hist.append(lat_ms)

            fail_streak = 0
            last_ok_ts = time.time()
            last_data_ts = last_ok_ts

            last_snap = snap
            last_diag = diag
            reading_fault_active = False

            # snapshot decode
            flags = snap[0]
            seq = snap[1]
            age_s = snap[2]

            RTC_FN = snap[3]
            RTC_D  = snap[4]
            RTP    = snap[5]

            meas = snap[6:14]  # 8 regs
            Ia_raw, Ib_raw, Ic_raw, ID_raw, IN_raw = meas[0], meas[1], meas[2], meas[3], meas[4]
            Va_raw, Vb_raw, Vc_raw = meas[5], meas[6], meas[7]

            # conversões iguais ao log antigo (se seus valores já estavam bons, mantenho)
            Ia = Ia_raw * RTC_FN / 256.0
            Ib = Ib_raw * RTC_FN / 256.0
            Ic = Ic_raw * RTC_FN / 256.0
            Va = Va_raw * RTP / 128.0
            Vb = Vb_raw * RTP / 128.0
            Vc = Vc_raw * RTP / 128.0

            fL = snap[14] / 256.0
            cosA = cosphi_from_raw(snap[15])
            cosB = cosphi_from_raw(snap[16])
            cosC = cosphi_from_raw(snap[17])

            p_regs = snap[18:24]
            Pa = i32_from_regs_be(p_regs[0], p_regs[1]) * (RTC_FN * RTP) / 256.0
            Pb = i32_from_regs_be(p_regs[2], p_regs[3]) * (RTC_FN * RTP) / 256.0
            Pc = i32_from_regs_be(p_regs[4], p_regs[5]) * (RTC_FN * RTP) / 256.0
            Pt = Pa + Pb + Pc

            pot_inv_atual_w = client_inv.read_power()

            if pot_inv_atual_w is None:
                if time.time() - client_inv.last_set_time < hold_last_ok_s:
                    pot_inv_atual_w = pct_to_watts(client_inv.last_set or 0, args.inv_pot)
                else:
                    print("[CTRL] Sem feedback → congelando controle")
                    continue    
                
            erro_w = Pt - args.setpoint

            if abs(erro_w) < deadband_w:
                erro_ctrl_w = 0.0
            else:
                erro_ctrl_w = erro_w

            integral_ws += erro_ctrl_w * dt
            integral_ws = clamp(integral_ws, integral_min, integral_max) # anti-windup

            u_w = kp * erro_ctrl_w + ki * integral_ws + kd * (erro_ctrl_w / dt if dt > 0 else 0.0)
            
            novo_limite_w = pot_inv_atual_w - u_w
            novo_limite_w = clamp(novo_limite_w, 0.0, args.inv_pot)

            novo_limite_pct = watts_to_pct(novo_limite_w, args.inv_pot)

            tempo_min_entre_comandos_s = 1.0
            pode_enviar = (time.time() - client_inv.last_set_time) >= tempo_min_entre_comandos_s

            dif_relevante_pct_x10 = 5 # 0,5%

            if client_inv.sock and pode_enviar:
                if client_inv.last_set is None or abs(novo_limite_pct - client_inv.last_set) >= dif_relevante_pct_x10:
                    try:
                        client_inv.set_power(novo_limite_pct)
                    except Exception as e:
                        print(f"[SET] erro: {e}\n")

            ID_type = snap[24]
            ID_ver  = snap[25]

            rtu_last_err = snap[26]
            rtu_ok_cnt   = snap[27]
            rtu_err_cnt  = snap[28]

            rssi_dbm = ""
            if diag:
                rssi_dbm = int(diag[2]) - 200

            lat_avg = int(sum(lat_hist)/len(lat_hist)) if lat_hist else lat_ms

            print(
                f"[{fmt_dt()}] | Pt={Pt:.1f}W | "
                f"PotInv={pot_inv_atual_w:.1f}W | "
                f"SP={args.setpoint:.1f}W | "
                f"Erro={erro_w:.1f}W | "
                f"Limite={novo_limite_w:.1f}W | "
                f"Cmd={novo_limite_pct/10.0:.1f}%"
            )

            cmd_pct = novo_limite_pct / 10.0
            cmd_raw = novo_limite_pct

            append_csv_row(csv_path, [
                fmt_dt(),
                round(Ia, 2),
                round(Ib, 2),
                round(Ic, 2),
                round(Va, 0),
                round(Vb, 0),
                round(Vc, 0),
                round(fL, 2),
                round(cosA, 3),
                round(cosB, 3),
                round(cosC, 3),
                round(Pt, 0),
                round(args.setpoint, 0),
                round(erro_w, 0),
                round(pot_inv_atual_w, 0),
                round(novo_limite_w, 0),
                round(cmd_pct, 1),
                cmd_raw,
                client_inv.last_set_response,
            ])

        except KeyboardInterrupt:
            print("\nEncerrado pelo usuário.")
            try:
                client.close()
            except Exception:
                pass
            return

        except Exception as e:
            msg = str(e)
            fail_streak += 1

            now_fail = time.time()
            since_data_ok = (now_fail - last_data_ts) if last_data_ts else 9999.0

            # ainda estamos dentro da janela de retenção
            if last_snap is not None and since_data_ok <= hold_last_ok_s:
                snap = last_snap
                diag = last_diag

                print(
                    f"\n[{fmt_dt()}] Leitura Modbus falhou temporariamente: {msg} "
                    f"(usando ultima leitura valida ha {since_data_ok:.1f}s)"
                )

                # continua o loop normalmente com os ultimos valores
                # sem derrubar cliente e sem marcar falha definitiva
                try:
                    # repetir daqui para frente o mesmo decode que você já faz com snap/diag
                    flags = snap[0]
                    seq = snap[1]
                    age_s = snap[2]

                    RTC_FN = snap[3]
                    RTC_D  = snap[4]
                    RTP    = snap[5]

                    meas = snap[6:14]  # 8 regs
                    Ia_raw, Ib_raw, Ic_raw, ID_raw, IN_raw = meas[0], meas[1], meas[2], meas[3], meas[4]
                    Va_raw, Vb_raw, Vc_raw = meas[5], meas[6], meas[7]

                    Ia = Ia_raw * RTC_FN / 256.0
                    Ib = Ib_raw * RTC_FN / 256.0
                    Ic = Ic_raw * RTC_FN / 256.0
                    Va = Va_raw * RTP / 128.0
                    Vb = Vb_raw * RTP / 128.0
                    Vc = Vc_raw * RTP / 128.0

                    fL = snap[14] / 256.0
                    cosA = cosphi_from_raw(snap[15])
                    cosB = cosphi_from_raw(snap[16])
                    cosC = cosphi_from_raw(snap[17])

                    p_regs = snap[18:24]
                    Pa = i32_from_regs_be(p_regs[0], p_regs[1]) * (RTC_FN * RTP) / 256.0
                    Pb = i32_from_regs_be(p_regs[2], p_regs[3]) * (RTC_FN * RTP) / 256.0
                    Pc = i32_from_regs_be(p_regs[4], p_regs[5]) * (RTC_FN * RTP) / 256.0
                    Pt = Pa + Pb + Pc

                    pot_inv_atual_w = client_inv.read_power()

                    if pot_inv_atual_w is None:
                        if time.time() - client_inv.last_set_time < hold_last_ok_s:
                            pot_inv_atual_w = pct_to_watts(client_inv.last_set or 0, args.inv_pot)
                        else:
                            print("[CTRL] Sem feedback → congelando controle")
                            continue          

                    erro_w = Pt - args.setpoint

                    if abs(erro_w) < deadband_w:
                        erro_ctrl_w = 0.0
                    else:
                        erro_ctrl_w = erro_w

                    integral_ws += erro_ctrl_w * dt
                    integral_ws = clamp(integral_ws, integral_min, integral_max) # anti-windup

                    u_w = kp * erro_ctrl_w + ki * integral_ws + kd * (erro_ctrl_w / dt if dt > 0 else 0.0)
            
                    novo_limite_w = pot_inv_atual_w - u_w
                    novo_limite_w = clamp(novo_limite_w, 0.0, args.inv_pot)

                    novo_limite_pct = watts_to_pct(novo_limite_w, args.inv_pot)

                    tempo_min_entre_comandos_s = 1.0
                    pode_enviar = (time.time() - client_inv.last_set_time) >= tempo_min_entre_comandos_s

                    dif_relevante_pct_x10 = 5 # 0,5%

                    if client_inv.sock and pode_enviar:
                        if client_inv.last_set is None or abs(novo_limite_pct - client_inv.last_set) >= dif_relevante_pct_x10:
                            try:
                                client_inv.set_power(novo_limite_pct)
                            except Exception as e2:
                                print(f"[SET] erro: {e2}")

                    print(
                        f"[{fmt_dt()}] | Pt={Pt:.1f}W | "
                        f"PotInv={pot_inv_atual_w:.1f}W | "
                        f"SP={args.setpoint:.1f}W | "
                        f"Erro={erro_w:.1f}W | "
                        f"Limite={novo_limite_w:.1f}W | "
                        f"Cmd={novo_limite_pct/10:.1f}% | "
                        f"FONTE=cache"
                    )

                    cmd_pct = novo_limite_pct / 10.0
                    cmd_raw = novo_limite_pct

                    append_csv_row(csv_path, [
                        fmt_dt(),
                        round(Ia, 2),
                        round(Ib, 2),
                        round(Ic, 2),
                        round(Va, 0),
                        round(Vb, 0),
                        round(Vc, 0),
                        round(fL, 2),
                        round(cosA, 3),
                        round(cosB, 3),
                        round(cosC, 3),
                        round(Pt, 0),
                        round(args.setpoint, 0),
                        round(erro_w, 0),
                        round(pot_inv_atual_w, 0),
                        round(novo_limite_w, 0),
                        round(cmd_pct, 1),
                        cmd_raw,
                        client_inv.last_set_response,
                    ])

                    continue

                except Exception as e_cache:
                    print(f"\n[{fmt_dt()}] Erro ao usar leitura em cache: {e_cache}")

            # passou do tempo de retenção: falha real
            if not reading_fault_active:
                print(
                    f"\n[{fmt_dt()}] Falha de leitura confirmada: {msg} "
                    f"(sem leitura valida ha {since_data_ok:.1f}s)"
                )
                reading_fault_active = True

            if should_reconnect(msg) or fail_streak >= 5:
                try:
                    client.close()
                except Exception:
                    pass
                client = None
                time.sleep(args.control_period)
            else:
                time.sleep(0.2)

if __name__ == "__main__":
    main()
