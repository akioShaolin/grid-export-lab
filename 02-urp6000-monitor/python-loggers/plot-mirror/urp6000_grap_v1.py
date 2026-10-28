#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Gráfico em tempo real da potência ativa total do URP6000 via datalogger ESP8266.

Compatível com o espelho Modbus TCP dos sketches v4 e produção:
  0x0100..0x0102 -> STATUS, SEQ, AGE_S
  0x003C..0x003E -> RTC_FN, RTC_D, RTP
  0x02CD..0x02D2 -> Pa, Pb e Pc (3 valores int32)

Instalação:
  py -m pip install pymodbus matplotlib

Uso:
  py urp6000_power_plot.py
  py urp6000_power_plot.py --host 192.0.2.10
"""

from site_private import URP_HOST
import argparse
import struct
import threading
import time
from collections import deque
from datetime import datetime, timedelta

import matplotlib.dates as mdates
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from pymodbus.client import ModbusTcpClient


REG_STATUS = 0x0100
REG_RATIOS = 0x003C
REG_POWER = 0x02CD

STATUS_LEN = 3
RATIOS_LEN = 3
POWER_LEN = 6

MAX_DATA_AGE_S = 5


def int32_from_registers(high_word, low_word):
    """Converte dois registradores uint16, na ordem high/low, para int32."""
    packed = struct.pack(">HH", high_word & 0xFFFF, low_word & 0xFFFF)
    return struct.unpack(">i", packed)[0]


def read_holding_registers(client, address, count, device_id):
    """
    Compatibilidade com versões diferentes do pymodbus:
    versões atuais usam device_id; versões anteriores usaram slave ou unit.
    """
    last_type_error = None

    for keyword in ("device_id", "slave", "unit", None):
        kwargs = {"address": address, "count": count}
        if keyword is not None:
            kwargs[keyword] = device_id

        try:
            response = client.read_holding_registers(**kwargs)
            if response is None:
                raise RuntimeError("Resposta Modbus vazia")
            if response.isError():
                raise RuntimeError(f"Erro Modbus: {response}")
            if not hasattr(response, "registers") or len(response.registers) < count:
                raise RuntimeError("Resposta Modbus incompleta")
            return response.registers
        except TypeError as exc:
            last_type_error = exc
            continue

    raise RuntimeError(f"API do pymodbus incompatível: {last_type_error}")


class PowerMonitor:
    def __init__(
        self,
        host,
        port,
        device_id,
        poll_s,
        timeout_s,
        current_nominal_a,
        history_hours,
    ):
        self.host = host
        self.port = port
        self.device_id = device_id
        self.poll_s = poll_s
        self.timeout_s = timeout_s
        self.current_nominal_a = current_nominal_a

        max_points = max(2, int(history_hours * 3600.0 / poll_s))
        self.timestamps = deque(maxlen=max_points)
        self.power_kw = deque(maxlen=max_points)

        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread = None
        self.client = None

        self.rtc_fn = None
        self.rtp = None
        self.status_text = "Aguardando conexão..."
        self.last_seq = None

    def set_status(self, text):
        with self.lock:
            self.status_text = text

    def connect(self):
        self.close_client()
        self.set_status(f"Conectando a {self.host}:{self.port}...")

        client = ModbusTcpClient(
            host=self.host,
            port=self.port,
            timeout=self.timeout_s,
        )

        connected = client.connect()
        if connected is False:
            client.close()
            raise ConnectionError("connect() retornou False")

        self.client = client
        self.rtc_fn = None
        self.rtp = None
        self.last_seq = None
        self.set_status(f"Conectado a {self.host}:{self.port}")

    def close_client(self):
        if self.client is not None:
            try:
                self.client.close()
            except Exception:
                pass
            self.client = None

    def read_ratios(self):
        registers = read_holding_registers(
            self.client,
            REG_RATIOS,
            RATIOS_LEN,
            self.device_id,
        )

        rtc_fn = registers[0]
        rtp = registers[2]

        if rtc_fn <= 0 or rtp <= 0:
            raise RuntimeError(
                f"Relações inválidas: RTC_FN={rtc_fn}, RTP={rtp}"
            )

        self.rtc_fn = rtc_fn
        self.rtp = rtp

    def read_total_power_kw(self):
        status_regs = read_holding_registers(
            self.client,
            REG_STATUS,
            STATUS_LEN,
            self.device_id,
        )

        status, seq, age_s = status_regs

        if status != 1:
            raise RuntimeError("ESP acessível, mas o snapshot RTU ainda não é válido")

        if age_s > MAX_DATA_AGE_S:
            raise RuntimeError(f"Dados RTU antigos: {age_s} s")

        if self.rtc_fn is None or self.rtp is None:
            self.read_ratios()

        power_regs = read_holding_registers(
            self.client,
            REG_POWER,
            POWER_LEN,
            self.device_id,
        )

        pa_raw = int32_from_registers(power_regs[0], power_regs[1])
        pb_raw = int32_from_registers(power_regs[2], power_regs[3])
        pc_raw = int32_from_registers(power_regs[4], power_regs[5])

        scale = 256.0 if self.current_nominal_a == 5 else 1280.0
        total_w = (
            (pa_raw + pb_raw + pc_raw)
            * self.rtc_fn
            * self.rtp
            / scale
        )

        self.last_seq = seq
        return total_w / 1000.0, age_s, seq

    def polling_loop(self):
        next_poll = time.monotonic()

        while not self.stop_event.is_set():
            now_monotonic = time.monotonic()
            wait_s = next_poll - now_monotonic
            if wait_s > 0:
                self.stop_event.wait(wait_s)
                continue

            next_poll += self.poll_s
            if now_monotonic - next_poll > 3.0 * self.poll_s:
                next_poll = now_monotonic + self.poll_s

            try:
                if self.client is None:
                    self.connect()

                total_kw, age_s, seq = self.read_total_power_kw()
                timestamp = datetime.now()

                with self.lock:
                    self.timestamps.append(timestamp)
                    self.power_kw.append(total_kw)
                    self.status_text = (
                        f"Online | Pt = {total_kw:.3f} kW | "
                        f"SEQ={seq} | idade={age_s} s"
                    )

            except Exception as exc:
                self.set_status(f"Sem leitura: {exc}")
                self.close_client()
                self.stop_event.wait(1.0)

        self.close_client()

    def start(self):
        self.thread = threading.Thread(
            target=self.polling_loop,
            name="modbus-poller",
            daemon=True,
        )
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=max(2.0, self.timeout_s + 1.0))
        self.close_client()

    def get_plot_data(self):
        with self.lock:
            return (
                list(self.timestamps),
                list(self.power_kw),
                self.status_text,
            )


def main():
    parser = argparse.ArgumentParser(
        description="Gráfico de potência total do URP6000 via Modbus TCP"
    )
    parser.add_argument("--host", default=URP_HOST)
    parser.add_argument("--port", type=int, default=502)
    parser.add_argument("--device-id", type=int, default=1)
    parser.add_argument("--poll", type=float, default=1.0)
    parser.add_argument("--timeout", type=float, default=3.0)
    parser.add_argument(
        "--in-nominal",
        type=int,
        choices=(1, 5),
        default=5,
        help="Entrada nominal de corrente do relé: 1 A ou 5 A",
    )
    parser.add_argument(
        "--hours",
        type=float,
        default=12.0,
        help="Janela máxima do gráfico em horas",
    )
    args = parser.parse_args()

    if args.poll <= 0:
        parser.error("--poll deve ser maior que zero")
    if args.hours <= 0 or args.hours > 12:
        parser.error("--hours deve estar entre 0 e 12")

    monitor = PowerMonitor(
        host=args.host,
        port=args.port,
        device_id=args.device_id,
        poll_s=args.poll,
        timeout_s=args.timeout,
        current_nominal_a=args.in_nominal,
        history_hours=args.hours,
    )
    monitor.start()

    fig, ax = plt.subplots()
    line, = ax.plot([], [], linewidth=1.2)

    ax.set_title("Potência ativa total — URP6000")
    ax.set_xlabel("Horário")
    ax.set_ylabel("Potência total (kW)")
    ax.grid(True, alpha=0.3)

    locator = mdates.AutoDateLocator(minticks=4, maxticks=10)
    ax.xaxis.set_major_locator(locator)
    ax.xaxis.set_major_formatter(mdates.ConciseDateFormatter(locator))

    status_box = ax.text(
        0.01,
        0.99,
        "Aguardando conexão...",
        transform=ax.transAxes,
        va="top",
        ha="left",
        bbox={"boxstyle": "round", "alpha": 0.8},
    )

    history_window = timedelta(hours=args.hours)

    def update_plot(_frame):
        timestamps, power_kw, status_text = monitor.get_plot_data()
        status_box.set_text(status_text)

        if not timestamps:
            return line, status_box

        line.set_data(timestamps, power_kw)

        right = max(datetime.now(), timestamps[-1])
        left_limit = right - history_window
        left = max(timestamps[0], left_limit)

        if right <= left:
            right = left + timedelta(seconds=1)

        ax.set_xlim(left, right)

        visible_values = [
            value
            for timestamp, value in zip(timestamps, power_kw)
            if timestamp >= left
        ]

        if visible_values:
            minimum = min(visible_values)
            maximum = max(visible_values)
            span = maximum - minimum
            margin = max(span * 0.08, abs(maximum) * 0.02, 0.1)
            ax.set_ylim(minimum - margin, maximum + margin)

        return line, status_box

    animation = FuncAnimation(
        fig,
        update_plot,
        interval=1000,
        blit=False,
        cache_frame_data=False,
    )

    def on_close(_event):
        monitor.stop()

    fig.canvas.mpl_connect("close_event", on_close)
    fig.tight_layout()

    try:
        plt.show()
    finally:
        monitor.stop()
        del animation


if __name__ == "__main__":
    main()
