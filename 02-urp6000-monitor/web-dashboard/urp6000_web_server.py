#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Servidor web responsivo para visualizar a potência ativa total do URP6000 no celular.

O navegador não acessa Modbus TCP diretamente. Este programa faz a ponte:
URP6000/ESP (Modbus TCP) -> Python -> HTTP -> celular.

Instalação:
    py -m pip install pymodbus

Execução:
    py urp6000_web_server.py

Depois, no celular conectado à mesma rede, abra:
    http://IP_DO_COMPUTADOR:8000
"""

from site_private import URP_HOST
import argparse
import json
import socket
import struct
import threading
import time
from collections import deque
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

from pymodbus.client import ModbusTcpClient


REG_STATUS = 0x0100
REG_RATIOS = 0x003C
REG_POWER = 0x02CD

STATUS_LEN = 3
RATIOS_LEN = 3
POWER_LEN = 6
MAX_DATA_AGE_S = 5
MAX_HISTORY_HOURS = 12


def int32_from_registers(high_word, low_word):
    packed = struct.pack(">HH", high_word & 0xFFFF, low_word & 0xFFFF)
    return struct.unpack(">i", packed)[0]


def read_holding_registers(client, address, count, device_id):
    """Compatibilidade entre versões do pymodbus."""
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
    ):
        self.host = host
        self.port = port
        self.device_id = device_id
        self.poll_s = poll_s
        self.timeout_s = timeout_s
        self.current_nominal_a = current_nominal_a

        max_points = max(2, int(MAX_HISTORY_HOURS * 3600 / poll_s))
        self.points = deque(maxlen=max_points)

        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread = None
        self.client = None

        self.rtc_fn = None
        self.rtp = None
        self.status = "starting"
        self.message = "Aguardando conexão..."
        self.last_power_kw = None
        self.last_seq = None
        self.last_age_s = None
        self.last_update_ms = None
        self.next_point_id = 1

    def set_state(self, status, message):
        with self.lock:
            self.status = status
            self.message = message

    def connect(self):
        self.close_client()
        self.set_state("connecting", f"Conectando a {self.host}:{self.port}...")

        client = ModbusTcpClient(
            host=self.host,
            port=self.port,
            timeout=self.timeout_s,
        )

        if client.connect() is False:
            client.close()
            raise ConnectionError("Não foi possível conectar ao datalogger")

        self.client = client
        self.rtc_fn = None
        self.rtp = None
        self.set_state("online", f"Conectado a {self.host}:{self.port}")

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
            raise RuntimeError(f"Relações inválidas: RTC_FN={rtc_fn}, RTP={rtp}")

        self.rtc_fn = rtc_fn
        self.rtp = rtp

    def read_total_power_kw(self):
        status_regs = read_holding_registers(
            self.client,
            REG_STATUS,
            STATUS_LEN,
            self.device_id,
        )
        snapshot_status, seq, age_s = status_regs

        if snapshot_status != 1:
            raise RuntimeError("ESP acessível, mas o snapshot RTU não é válido")
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
        return total_w / 1000.0, age_s, seq

    def polling_loop(self):
        next_poll = time.monotonic()

        while not self.stop_event.is_set():
            wait_s = next_poll - time.monotonic()
            if wait_s > 0:
                self.stop_event.wait(wait_s)
                continue

            next_poll += self.poll_s
            if time.monotonic() - next_poll > self.poll_s * 3:
                next_poll = time.monotonic() + self.poll_s

            try:
                if self.client is None:
                    self.connect()

                power_kw, age_s, seq = self.read_total_power_kw()
                now_ms = int(time.time() * 1000)

                with self.lock:
                    point = {
                        "id": self.next_point_id,
                        "t": now_ms,
                        "kw": round(power_kw, 6),
                    }
                    self.next_point_id += 1
                    self.points.append(point)
                    self.status = "online"
                    self.message = "Leitura normal"
                    self.last_power_kw = power_kw
                    self.last_seq = seq
                    self.last_age_s = age_s
                    self.last_update_ms = now_ms

            except Exception as exc:
                self.set_state("offline", str(exc))
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

    def get_state(self):
        with self.lock:
            return {
                "status": self.status,
                "message": self.message,
                "power_kw": (
                    round(self.last_power_kw, 3)
                    if self.last_power_kw is not None
                    else None
                ),
                "seq": self.last_seq,
                "age_s": self.last_age_s,
                "last_update_ms": self.last_update_ms,
                "host": self.host,
                "poll_s": self.poll_s,
            }

    def get_history(self, hours):
        cutoff_ms = int((time.time() - hours * 3600) * 1000)
        with self.lock:
            return [point.copy() for point in self.points if point["t"] >= cutoff_ms]

    def get_points_after(self, after_id):
        with self.lock:
            return [point.copy() for point in self.points if point["id"] > after_id]


class DashboardHandler(BaseHTTPRequestHandler):
    server_version = "URP6000Web/1.0"

    def log_message(self, format_string, *args):
        # Evita poluir o terminal a cada segundo.
        return

    @property
    def monitor(self):
        return self.server.monitor

    @property
    def html_bytes(self):
        return self.server.html_bytes

    def send_bytes(self, content, content_type, status=HTTPStatus.OK):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.end_headers()
        self.wfile.write(content)

    def send_json(self, payload, status=HTTPStatus.OK):
        content = json.dumps(
            payload,
            ensure_ascii=False,
            separators=(",", ":"),
        ).encode("utf-8")
        self.send_bytes(content, "application/json; charset=utf-8", status)

    def do_GET(self):
        parsed = urlparse(self.path)

        if parsed.path in ("/", "/index.html"):
            self.send_bytes(self.html_bytes, "text/html; charset=utf-8")
            return

        if parsed.path == "/api/history":
            query = parse_qs(parsed.query)
            try:
                hours = float(query.get("hours", ["12"])[0])
            except ValueError:
                hours = 12.0

            hours = min(max(hours, 1 / 60), MAX_HISTORY_HOURS)
            points = self.monitor.get_history(hours)
            self.send_json({
                "state": self.monitor.get_state(),
                "points": points,
            })
            return

        if parsed.path == "/api/latest":
            query = parse_qs(parsed.query)
            try:
                after_id = int(query.get("after", ["0"])[0])
            except ValueError:
                after_id = 0

            self.send_json({
                "state": self.monitor.get_state(),
                "points": self.monitor.get_points_after(after_id),
            })
            return

        if parsed.path == "/api/state":
            self.send_json(self.monitor.get_state())
            return

        self.send_json(
            {"error": "Rota não encontrada"},
            status=HTTPStatus.NOT_FOUND,
        )


def find_local_ip():
    """Tenta descobrir o IP da máquina sem depender de acesso à internet."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect(("10.255.255.255", 1))
        return sock.getsockname()[0]
    except OSError:
        try:
            return socket.gethostbyname(socket.gethostname())
        except OSError:
            return "127.0.0.1"
    finally:
        sock.close()


def main():
    parser = argparse.ArgumentParser(
        description="Painel web móvel da potência total do URP6000"
    )
    parser.add_argument("--host", default=URP_HOST)
    parser.add_argument("--port", type=int, default=502)
    parser.add_argument("--device-id", type=int, default=1)
    parser.add_argument("--poll", type=float, default=1.0)
    parser.add_argument("--timeout", type=float, default=3.0)
    parser.add_argument("--in-nominal", type=int, choices=(1, 5), default=5)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--http-port", type=int, default=8000)
    args = parser.parse_args()

    if args.poll <= 0:
        parser.error("--poll deve ser maior que zero")

    html_path = Path(__file__).with_name("urp6000_mobile.html")
    if not html_path.exists():
        raise FileNotFoundError(
            f"Arquivo HTML não encontrado: {html_path}"
        )

    monitor = PowerMonitor(
        host=args.host,
        port=args.port,
        device_id=args.device_id,
        poll_s=args.poll,
        timeout_s=args.timeout,
        current_nominal_a=args.in_nominal,
    )
    monitor.start()

    server = ThreadingHTTPServer((args.bind, args.http_port), DashboardHandler)
    server.monitor = monitor
    server.html_bytes = html_path.read_bytes()

    local_ip = find_local_ip()
    print()
    print("Painel URP6000 iniciado.")
    print(f"Computador: http://127.0.0.1:{args.http_port}")
    print(f"Celular:    http://{local_ip}:{args.http_port}")
    print("Pressione Ctrl+C para encerrar.")
    print()

    try:
        server.serve_forever(poll_interval=0.5)
    except KeyboardInterrupt:
        print("\nEncerrando...")
    finally:
        server.shutdown()
        server.server_close()
        monitor.stop()


if __name__ == "__main__":
    main()
