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
  py urp6000_tank_rtu_v1.py --port 502 --poll 60.0 --timeout 30.0
"""

from site_private import URP_HOST
import os
import time
import csv
import socket
import argparse
import struct
from datetime import datetime
from collections import deque

from pymodbus.client import ModbusTcpClient

REG_SNAP_START = 0x0200
REG_SNAP_LEN   = 32
REG_DIAG_START = 0x0100
REG_DIAG_LEN   = 0x20

OUT_DIR = "logs"
OUT_FILE = "urp6000_tank_rtu.csv"
CSV_PATH = os.path.join(OUT_DIR, OUT_FILE)

CSV_FIELDS = [
    "DataHora",
    "Ia_A","Ib_A","Ic_A",
    "Va_V","Vb_V","Vc_V",
    "Pt_W",
    "fL_Hz",
    "cosA","cosB","cosC",
    "RTC_FN","RTC_D","RTP",
    "ID_type","ID_ver",
    "SEQ","AGE_S",
    "RTU_last_err","RTU_ok_cnt","RTU_err_cnt",
    "RSSI_dBm","lat_ms"
]

def fmt_dt() -> str:
    return datetime.now().strftime("%d/%m/%Y %H:%M:%S")

def ensure_csv():
    os.makedirs(OUT_DIR, exist_ok=True)
    if not os.path.exists(CSV_PATH):
        with open(CSV_PATH, "w", newline="", encoding="utf-8") as f:
            csv.writer(f, delimiter=";").writerow(CSV_FIELDS)

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

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", default=URP_HOST)
    ap.add_argument("--port", type=int, default=502)
    ap.add_argument("--poll", type=float, default=60.0)
    ap.add_argument("--timeout", type=float, default=30.0)
    ap.add_argument("--diag_every", type=int, default=2)
    args = ap.parse_args()

    ensure_csv()

    client = None
    fail_streak = 0
    last_ok_ts = 0.0
    backoff = 1.0
    lat_hist = deque(maxlen=20)

    next_t = time.time() + args.poll
    cycle = 0

    while True:
        now = time.time()
        if now < next_t:
            time.sleep(0.01)
            continue
        next_t = max(next_t + args.poll, now + args.poll)
        cycle += 1

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
            diag = None
            if (cycle % max(1, args.diag_every)) == 0:
                diag = read_holding(client, REG_DIAG_START, REG_DIAG_LEN)

            lat_ms = int((time.time() - t0) * 1000)
            lat_hist.append(lat_ms)

            fail_streak = 0
            last_ok_ts = time.time()

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
                f"[{fmt_dt()}] Ia={Ia:.2f}A Ib={Ib:.2f}A Ic={Ic:.2f}A | "
                f"Va={Va:.1f}V Vb={Vb:.1f}V Vc={Vc:.1f}V | "
                f"fL={fL:.2f}Hz | cosA={cosA:.3f} cosB={cosB:.3f} cosC={cosC:.3f} | "
                f"Pt={Pt:.1f}W | SEQ={seq} age={age_s}s lat={lat_ms}ms(avg {lat_avg}ms) rssi={rssi_dbm}dBm",
                end="\r"
            )

            with open(CSV_PATH, "a", newline="", encoding="utf-8") as f:
                csv.writer(f, delimiter=";").writerow([
                    fmt_dt(),
                    f"{Ia:.2f}".replace(".", ","), f"{Ib:.2f}".replace(".", ","), f"{Ic:.2f}".replace(".", ","),
                    f"{Va:.1f}".replace(".", ","), f"{Vb:.1f}".replace(".", ","), f"{Vc:.1f}".replace(".", ","),
                    f"{Pt:.1f}".replace(".", ","),
                    f"{fL:.2f}".replace(".", ","),
                    f"{cosA:.3f}".replace(".", ","), f"{cosB:.3f}".replace(".", ","), f"{cosC:.3f}".replace(".", ","),
                    RTC_FN, RTC_D, RTP,
                    ID_type, ID_ver,
                    seq, age_s,
                    rtu_last_err, rtu_ok_cnt, rtu_err_cnt,
                    rssi_dbm, lat_ms
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
            since_ok = (time.time() - last_ok_ts) if last_ok_ts else 9999.0
            print(f"\n[{fmt_dt()}] Falha leitura: {msg} (falhas seguidas={fail_streak}, desde OK={since_ok:.1f}s)")

            if should_reconnect(msg) or fail_streak >= 5:
                try:
                    client.close()
                except Exception:
                    pass
                client = None
                time.sleep(1.0)
            else:
                time.sleep(0.2)

if __name__ == "__main__":
    main()
