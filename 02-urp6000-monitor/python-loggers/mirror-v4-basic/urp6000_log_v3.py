#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
urp6000_log.py - Lê o ESP8266 (Modbus TCP) que espelha registradores do relé URP6000 (Modbus RTU)
e grava um CSV com valores já convertidos.

Requisitos:
  pip install pymodbus

Uso:
  python urp6000_log.py --ip 192.0.2.10 --port 502 --poll 1.0

Saída:
  logs/urp6000_log.csv  (separador ";", decimais com vírgula)
"""

from site_private import URP_HOST
import os
import time
import argparse
import struct
from datetime import datetime
from pymodbus.client import ModbusTcpClient

# --- Endereços (HEX) ---
REG_STATUS = 0x0100
REG_SEQ    = 0x0101
REG_AGE_S  = 0x0102

REG_ID_START   = 0x0088  # 2 regs
REG_RAT_START  = 0x003C  # RTC_FN, RTC_D, RTP (3 regs)
REG_MEAS_START = 0x02BC  # 0700..0707 (8 regs)
REG_P_START    = 0x02CD  # 0717..0722 (6 regs) = 3x int32

# --- CSV ---
OUT_DIR = "logs"
OUT_FILE = "urp6000_log.csv"
CSV_PATH = os.path.join(OUT_DIR, OUT_FILE)

ID_READ_EVERY_N = 10  # a cada N amostras, lê e imprime 0x0088..0x0089

CSV_FIELDS = [
    "DataHora",
    "RTC_FN", "RTC_D", "RTP",
    "Ia_A", "Ib_A", "Ic_A", "ID_A", "IN_A",
    "Va_V", "Vb_V", "Vc_V",
    "Pa_W", "Pb_W", "Pc_W", "Pt_W",
    "SEQ", "AGE_S", "STATUS"
]

def ensure_csv():
    os.makedirs(OUT_DIR, exist_ok=True)
    if not os.path.exists(CSV_PATH):
        with open(CSV_PATH, "a", encoding="utf-8", newline="") as f:
            f.write(";".join(CSV_FIELDS) + "\n")

def fmt_dt():
    return datetime.now().strftime("%d/%m/%Y %H:%M:%S")

def i32_from_regs_be(hi: int, lo: int) -> int:
    """2x uint16 (hi, lo) -> int32 (big-endian por palavra)."""
    raw = struct.pack(">HH", hi & 0xFFFF, lo & 0xFFFF)
    return struct.unpack(">i", raw)[0]

def read_holding(client: ModbusTcpClient, addr: int, count: int, unit: int = 1):
    try:
        rr = client.read_holding_registers(address=addr, count=count, unit=unit)
    except TypeError:
        rr = client.read_holding_registers(address=addr, count=count)

    if rr.isError():
        return None, str(rr)
    return rr.registers, None

def connect_with_retry(ip: str, port: int, timeout_s: float):
    while True:
        c = ModbusTcpClient(ip, port=port, timeout=timeout_s)
        if c.connect():
            return c
        print(f"[{fmt_dt()}] TCP offline ({ip}:{port}). Aguardando...")
        time.sleep(2.0)

def compute_units(rtc_fn: int, rtp: int, in_nominal: int, meas_regs, p_regs):
    """
    Converte grandezas com base nos formatos mostrados nas tabelas do URP.

    Correntes (0700..0704):
      In=5A  -> A = raw * RTC_FN / 256
      In=1A  -> A = raw * RTC_FN / 1280

    Tensões (0705..0707):
      V = raw * RTP / 256

    Potência ativa por fase (0717..0722):
      int32 * RTC_FN * RTP / scale
      In=5A  -> scale = 256
      In=1A  -> scale = 1280
    """
    scale_i = 256.0 if in_nominal == 5 else 1280.0
    scale_p = 256.0 if in_nominal == 5 else 1280.0

    ia_raw, ib_raw, ic_raw, id_raw, in_raw = meas_regs[0:5]
    va_raw, vb_raw, vc_raw = meas_regs[5:8]

    ia = ia_raw * rtc_fn / scale_i
    ib = ib_raw * rtc_fn / scale_i
    ic = ic_raw * rtc_fn / scale_i
    idd = id_raw * rtc_fn / scale_i
    inn = in_raw * rtc_fn / scale_i

    va = va_raw * rtp / 256.0
    vb = vb_raw * rtp / 256.0
    vc = vc_raw * rtp / 256.0

    pa_i32 = i32_from_regs_be(p_regs[0], p_regs[1])
    pb_i32 = i32_from_regs_be(p_regs[2], p_regs[3])
    pc_i32 = i32_from_regs_be(p_regs[4], p_regs[5])

    pa = pa_i32 * rtc_fn * rtp / scale_p
    pb = pb_i32 * rtc_fn * rtp / scale_p
    pc = pc_i32 * rtc_fn * rtp / scale_p
    pt = pa + pb + pc

    return ia, ib, ic, idd, inn, va, vb, vc, pa, pb, pc, pt

def write_csv_row(row):
    def fmt(v):
        if isinstance(v, float):
            s = f"{v:.3f}"
            return s.replace(".", ",")
        return str(v)
    with open(CSV_PATH, "a", encoding="utf-8", newline="") as f:
        f.write(";".join(fmt(v) for v in row) + "\n")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", default=URP_HOST, help="Endereço IP do Modbus TCP (ESP8266)")
    ap.add_argument("--port", type=int, default=502)
    ap.add_argument("--unit", type=int, default=1, help="Unit ID do Modbus TCP (geralmente não importa)")
    ap.add_argument("--poll", type=float, default=1.0, help="Período de polling (s)")
    ap.add_argument("--timeout", type=float, default=2.0, help="Timeout TCP (s)")
    ap.add_argument("--in", dest="in_nominal", type=int, choices=[1,5], default=5,
                    help="Corrente nominal do relé (In=1A ou In=5A), para escolher o fator de escala")
    args = ap.parse_args()

    ensure_csv()
    client = None
    last_seq = None
    sample_count = 0
    next_t = time.time()

    while True:
        if client is None:
            client = connect_with_retry(args.ip, args.port, args.timeout)
            print(f"[{fmt_dt()}] Conectado em {args.ip}:{args.port}")
            next_t = time.time() + args.poll

        now = time.time()
        if now < next_t:
            time.sleep(0.01)
            continue
        if now - next_t > 2 * args.poll:
            next_t = now + args.poll
        else:
            next_t += args.poll

        try:
            seq_regs, err = read_holding(client, REG_SEQ, 1, unit=args.unit)
            if err:
                raise RuntimeError(f"Erro lendo SEQ: {err}")
            seq = seq_regs[0]
            if last_seq is not None and seq == last_seq:
                continue
            last_seq = seq

            sample_count += 1
            if (sample_count % ID_READ_EVERY_N) == 0:
                id_regs, id_err = read_holding(client, REG_ID_START, 2, unit=args.unit)
                if not id_err and id_regs and len(id_regs) == 2:
                    id_type, id_ver = id_regs[0], id_regs[1]
                    print(f"[{fmt_dt()}] ID: tipo={id_type} (0x{id_type:04X}) versao={id_ver} (0x{id_ver:04X})")
                else:
                    print(f"[{fmt_dt()}] ID: falha lendo 0x0088..0x0089: {id_err}")

            status, _ = read_holding(client, REG_STATUS, 1, unit=args.unit)
            age_s,  _ = read_holding(client, REG_AGE_S, 1, unit=args.unit)

            rat, err = read_holding(client, REG_RAT_START, 3, unit=args.unit)
            if err:
                raise RuntimeError(f"Erro lendo relações 0x003C: {err}")
            rtc_fn, rtc_d, rtp = rat[0], rat[1], rat[2]

            meas, err = read_holding(client, REG_MEAS_START, 8, unit=args.unit)
            if err:
                raise RuntimeError(f"Erro lendo medições 0x02BC: {err}")

            p_regs, err = read_holding(client, REG_P_START, 6, unit=args.unit)
            if err:
                raise RuntimeError(f"Erro lendo potências 0x02CD: {err}")

            ia, ib, ic, idd, inn, va, vb, vc, pa, pb, pc, pt = compute_units(
                rtc_fn=rtc_fn, rtp=rtp, in_nominal=args.in_nominal,
                meas_regs=meas, p_regs=p_regs
            )

            row = [
                fmt_dt(),
                rtc_fn, rtc_d, rtp,
                ia, ib, ic, idd, inn,
                va, vb, vc,
                pa, pb, pc, pt,
                seq, age_s[0] if age_s else "", status[0] if status else ""
            ]

            print(f"[{row[0]}] Ia={ia:.2f}A Ib={ib:.2f}A Ic={ic:.2f}A | Va={va:.1f}V Vb={vb:.1f}V Vc={vc:.1f}V | Pt={pt:.1f}W | SEQ={seq}")
            write_csv_row(row)

        except Exception as e:
            print(f"[{fmt_dt()}] Exceção: {e}")
            try:
                client.close()
            except Exception:
                pass
            client = None
            time.sleep(1.0)

if __name__ == "__main__":
    main()
