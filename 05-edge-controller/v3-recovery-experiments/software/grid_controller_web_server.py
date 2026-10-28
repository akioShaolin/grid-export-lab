#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
grid_controller v1 - coletor de estatísticas e painel web.

Fluxo:
    ESP grid-controller.local -> /api/stats (janelas processadas no ESP)
                      -> Python -> CSV persistente -> navegador

O ESP agrega uma janela padrão de 5 minutos. Este programa consulta a fila
periodicamente, evita duplicatas por chip/boot/seq e preserva o histórico em CSV.
Usa somente a biblioteca padrão do Python.

Uso:
    py grid_controller_web_server.py
    py grid_controller_web_server.py --device http://192.0.2.10

No celular/computador:
    http://IP_DESTA_MAQUINA:8000
"""

from __future__ import annotations
from site_private import DEVICE_URL

import argparse
import csv
import json
import socket
import sys
import threading
import time
from collections import deque
from datetime import datetime, timezone
from http import HTTPStatus
from http.client import HTTPException
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.parse import parse_qs, urlparse
from urllib.request import Request, urlopen

CSV_FIELDS = [
    "timestamp_iso",
    "device",
    "chip_id",
    "session_id",
    "boot_count",
    "bucket_seq",
    "end_uptime_s",
    "duration_s",
    "samples",
    "meter_avg_w",
    "meter_min_w",
    "meter_max_w",
    "inv_avg_w",
    "inv_min_w",
    "inv_max_w",
    "setpoint_avg_w",
    "error_avg_w",
    "command_avg_w",
    "online_mask",
    "alarm_flags",
]


def utc_now_ms() -> int:
    return int(time.time() * 1000)


def iso_from_ms(timestamp_ms: int) -> str:
    return datetime.fromtimestamp(timestamp_ms / 1000.0, tz=timezone.utc).isoformat(
        timespec="seconds"
    )


def safe_float(value: Any, default: float = 0.0) -> float:
    try:
        result = float(value)
        if result != result or result in (float("inf"), float("-inf")):
            return default
        return result
    except (TypeError, ValueError):
        return default


def safe_int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def fetch_json(
    url: str,
    timeout_s: float,
    attempts: int = 3,
) -> dict[str, Any]:
    """Obtém um JSON do ESP com novas tentativas em falhas transitórias.

    O ESP8266 pode encerrar a conexão antes de enviar todo o corpo HTTP quando
    há perda momentânea de Wi-Fi, reinicialização ou falta de recursos. Nesses
    casos, urllib levanta HTTPException/IncompleteRead. Como a chamada é GET,
    repetir a requisição é seguro.
    """
    attempts = max(1, attempts)
    last_error: Exception | None = None

    for attempt in range(1, attempts + 1):
        request = Request(
            url,
            headers={
                "Accept": "application/json",
                "Cache-Control": "no-cache",
                "Connection": "close",
                "User-Agent": "grid_controller-v1-logger/1.1",
            },
        )

        try:
            with urlopen(request, timeout=timeout_s) as response:
                if response.status != HTTPStatus.OK:
                    raise RuntimeError(f"HTTP {response.status} em {url}")
                data = response.read()

            payload = json.loads(data.decode("utf-8"))
            if not isinstance(payload, dict):
                raise RuntimeError("Resposta JSON inesperada")
            return payload

        except (
            HTTPException,
            URLError,
            TimeoutError,
            OSError,
            UnicodeDecodeError,
            json.JSONDecodeError,
        ) as exc:
            last_error = exc
            if attempt < attempts:
                time.sleep(0.25 * attempt)

    raise RuntimeError(
        f"Falha ao receber JSON completo de {url} após {attempts} tentativas: "
        f"{last_error}"
    ) from last_error


class grid_controllerCollector:
    def __init__(
        self,
        device_url: str,
        csv_path: Path,
        poll_s: float,
        timeout_s: float,
        memory_days: float,
    ) -> None:
        self.device_url = device_url.rstrip("/")
        self.csv_path = csv_path
        self.poll_s = poll_s
        self.timeout_s = timeout_s
        self.memory_ms = int(max(1.0, memory_days) * 86_400_000)

        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.thread: threading.Thread | None = None

        self.points: deque[dict[str, Any]] = deque()
        self.seen: set[tuple[int, int, int]] = set()
        self.next_web_id = 1

        self.device_state: dict[str, Any] = {
            "status": "starting",
            "message": "Aguardando o ESP...",
            "last_contact_ms": None,
            "device_url": self.device_url,
        }
        self.last_boot_by_chip: dict[int, int] = {}
        # Chave: (chip_id, session_id). session_id muda a cada boot e evita
        # colisão após desligamento total, quando a memória RTC pode zerar.
        self.last_seq_by_identity: dict[tuple[int, int], int] = {}

        self._prepare_csv()
        self._load_csv()

    def _prepare_csv(self) -> None:
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        if not self.csv_path.exists() or self.csv_path.stat().st_size == 0:
            with self.csv_path.open("w", newline="", encoding="utf-8-sig") as handle:
                csv.DictWriter(handle, fieldnames=CSV_FIELDS, delimiter=";").writeheader()
            return

        # Migração simples de versões anteriores do CSV. Preserva os registros e
        # usa boot_count como session_id quando esse campo ainda não existia.
        with self.csv_path.open("r", newline="", encoding="utf-8-sig") as handle:
            reader = csv.DictReader(handle, delimiter=";")
            current_fields = reader.fieldnames or []
            if current_fields == CSV_FIELDS:
                return
            old_rows = list(reader)

        temporary = self.csv_path.with_suffix(self.csv_path.suffix + ".tmp")
        with temporary.open("w", newline="", encoding="utf-8-sig") as handle:
            writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS, delimiter=";")
            writer.writeheader()
            for row in old_rows:
                if not row.get("session_id"):
                    row["session_id"] = row.get("boot_count", "0")
                writer.writerow({field: row.get(field, "") for field in CSV_FIELDS})
        temporary.replace(self.csv_path)

    def _load_csv(self) -> None:
        cutoff_ms = utc_now_ms() - self.memory_ms
        try:
            with self.csv_path.open("r", newline="", encoding="utf-8-sig") as handle:
                for row in csv.DictReader(handle, delimiter=";"):
                    try:
                        timestamp_ms = int(
                            datetime.fromisoformat(row["timestamp_iso"]).timestamp() * 1000
                        )
                    except (KeyError, TypeError, ValueError):
                        continue

                    chip_id = safe_int(row.get("chip_id"))
                    boot_count = safe_int(row.get("boot_count"))
                    session_id = safe_int(row.get("session_id"), boot_count)
                    seq = safe_int(row.get("bucket_seq"))
                    self.seen.add((chip_id, session_id, seq))
                    self.last_boot_by_chip[chip_id] = max(
                        boot_count, self.last_boot_by_chip.get(chip_id, 0)
                    )
                    identity = (chip_id, session_id)
                    self.last_seq_by_identity[identity] = max(
                        seq, self.last_seq_by_identity.get(identity, 0)
                    )

                    if timestamp_ms >= cutoff_ms:
                        self.points.append(self._row_to_point(row, timestamp_ms))
        except OSError as exc:
            raise RuntimeError(f"Não foi possível abrir o CSV: {exc}") from exc

        if self.points:
            self.next_web_id = max(point["id"] for point in self.points) + 1

    def _row_to_point(self, row: dict[str, Any], timestamp_ms: int) -> dict[str, Any]:
        point = {
            "id": self.next_web_id,
            "t": timestamp_ms,
            "device": row.get("device", "grid-controller"),
            "chip_id": safe_int(row.get("chip_id")),
            "session_id": safe_int(row.get("session_id"), safe_int(row.get("boot_count"))),
            "boot_count": safe_int(row.get("boot_count")),
            "seq": safe_int(row.get("bucket_seq")),
            "duration_s": safe_int(row.get("duration_s")),
            "samples": safe_int(row.get("samples")),
            "meter_avg_kw": safe_float(row.get("meter_avg_w")) / 1000.0,
            "meter_min_kw": safe_float(row.get("meter_min_w")) / 1000.0,
            "meter_max_kw": safe_float(row.get("meter_max_w")) / 1000.0,
            "inv_avg_kw": safe_float(row.get("inv_avg_w")) / 1000.0,
            "inv_min_kw": safe_float(row.get("inv_min_w")) / 1000.0,
            "inv_max_kw": safe_float(row.get("inv_max_w")) / 1000.0,
            "setpoint_kw": safe_float(row.get("setpoint_avg_w")) / 1000.0,
            "error_kw": safe_float(row.get("error_avg_w")) / 1000.0,
            "command_kw": safe_float(row.get("command_avg_w")) / 1000.0,
            "online_mask": safe_int(row.get("online_mask")),
            "alarm_flags": safe_int(row.get("alarm_flags")),
        }
        self.next_web_id += 1
        return point

    def _append_csv(self, row: dict[str, Any]) -> None:
        with self.csv_path.open("a", newline="", encoding="utf-8-sig") as handle:
            writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS, delimiter=";")
            writer.writerow({field: row.get(field, "") for field in CSV_FIELDS})
            handle.flush()

    def _prune_memory(self) -> None:
        cutoff = utc_now_ms() - self.memory_ms
        while self.points and self.points[0]["t"] < cutoff:
            self.points.popleft()

    def _status_url(self) -> str:
        return f"{self.device_url}/api/status"

    def _stats_url(self, after: int) -> str:
        return f"{self.device_url}/api/stats?after={after}&limit=72"

    def _get_cursor(self, chip_id: int, session_id: int) -> int:
        return self.last_seq_by_identity.get((chip_id, session_id), 0)

    def collect_once(self) -> int:
        state = fetch_json(self._status_url(), self.timeout_s)
        now_ms = utc_now_ms()
        chip_id = safe_int(state.get("chip_id"), 0)

        # Versões iniciais do firmware podem não expor chip_id no status.
        # O endpoint /api/stats sempre o expõe; usa cursor zero até obtê-lo.
        status_boot = safe_int(state.get("boot_count"), 0)
        status_session = safe_int(state.get("session_id"), status_boot)
        provisional_after = self._get_cursor(chip_id, status_session) if chip_id else 0
        payload = fetch_json(self._stats_url(provisional_after), self.timeout_s)

        chip_id = safe_int(payload.get("chip_id"), chip_id)
        boot_count = safe_int(payload.get("boot_count"), status_boot)
        session_id = safe_int(payload.get("session_id"), status_session or boot_count)
        device = str(payload.get("device") or state.get("hostname") or "grid-controller")
        cursor = self._get_cursor(chip_id, session_id)

        # Se o primeiro pedido usou identidade incompleta, repete com o cursor correto.
        if provisional_after != cursor:
            payload = fetch_json(self._stats_url(cursor), self.timeout_s)

        buckets = payload.get("buckets", [])
        if not isinstance(buckets, list):
            raise RuntimeError("Campo buckets inválido")

        uptime_now_s = safe_int(state.get("uptime_s"), 0)
        latest_end_uptime = max(
            (safe_int(bucket.get("end_uptime_s"), 0) for bucket in buckets if isinstance(bucket, dict)),
            default=uptime_now_s,
        )
        if uptime_now_s <= 0:
            uptime_now_s = latest_end_uptime

        added = 0
        with self.lock:
            for bucket in buckets:
                if not isinstance(bucket, dict):
                    continue
                seq = safe_int(bucket.get("seq"), 0)
                key = (chip_id, session_id, seq)
                if seq <= 0 or key in self.seen:
                    continue

                end_uptime_s = safe_int(bucket.get("end_uptime_s"), uptime_now_s)
                age_s = max(0, uptime_now_s - end_uptime_s)
                timestamp_ms = now_ms - age_s * 1000

                row = {
                    "timestamp_iso": iso_from_ms(timestamp_ms),
                    "device": device,
                    "chip_id": chip_id,
                    "session_id": session_id,
                    "boot_count": boot_count,
                    "bucket_seq": seq,
                    "end_uptime_s": end_uptime_s,
                    "duration_s": safe_int(bucket.get("duration_s")),
                    "samples": safe_int(bucket.get("samples")),
                    "meter_avg_w": safe_float(bucket.get("meter_avg_w")),
                    "meter_min_w": safe_float(bucket.get("meter_min_w")),
                    "meter_max_w": safe_float(bucket.get("meter_max_w")),
                    "inv_avg_w": safe_float(bucket.get("inv_avg_w")),
                    "inv_min_w": safe_float(bucket.get("inv_min_w")),
                    "inv_max_w": safe_float(bucket.get("inv_max_w")),
                    "setpoint_avg_w": safe_float(bucket.get("setpoint_avg_w")),
                    "error_avg_w": safe_float(bucket.get("error_avg_w")),
                    "command_avg_w": safe_float(bucket.get("command_avg_w")),
                    "online_mask": safe_int(bucket.get("online_mask")),
                    "alarm_flags": safe_int(bucket.get("alarm_flags")),
                }
                self._append_csv(row)
                self.points.append(self._row_to_point(row, timestamp_ms))
                self.seen.add(key)
                self.last_seq_by_identity[(chip_id, session_id)] = max(
                    seq, self.last_seq_by_identity.get((chip_id, session_id), 0)
                )
                added += 1

            self.last_boot_by_chip[chip_id] = max(
                boot_count, self.last_boot_by_chip.get(chip_id, 0)
            )
            self._prune_memory()
            self.device_state = {
                **state,
                "status": "online",
                "message": "Leitura normal",
                "last_contact_ms": now_ms,
                "device_url": self.device_url,
                "csv_path": str(self.csv_path),
                "stored_points": len(self.points),
                "last_bucket_seq": self.last_seq_by_identity.get(
                    (chip_id, session_id), 0
                ),
                "chip_id": chip_id,
                "session_id": session_id,
                "boot_count": boot_count,
            }

        return added

    def polling_loop(self) -> None:
        next_run = time.monotonic()
        while not self.stop_event.is_set():
            wait_s = next_run - time.monotonic()
            if wait_s > 0:
                self.stop_event.wait(wait_s)
                continue
            next_run = time.monotonic() + self.poll_s

            try:
                self.collect_once()
            except (HTTPError, URLError, TimeoutError, OSError, ValueError, RuntimeError) as exc:
                with self.lock:
                    previous = dict(self.device_state)
                    previous.update(
                        status="offline",
                        message=str(exc),
                        device_url=self.device_url,
                    )
                    self.device_state = previous

    def start(self) -> None:
        self.thread = threading.Thread(
            target=self.polling_loop, name="grid_controller-collector", daemon=True
        )
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=max(2.0, self.timeout_s + 1.0))

    def get_state(self) -> dict[str, Any]:
        with self.lock:
            return dict(self.device_state)

    def get_history(self, hours: float) -> list[dict[str, Any]]:
        cutoff = utc_now_ms() - int(hours * 3_600_000)
        with self.lock:
            return [dict(point) for point in self.points if point["t"] >= cutoff]

    def get_points_after(self, after_id: int) -> list[dict[str, Any]]:
        with self.lock:
            return [dict(point) for point in self.points if point["id"] > after_id]


class DashboardHandler(BaseHTTPRequestHandler):
    server_version = "grid_controllerWeb/1.1"
    protocol_version = "HTTP/1.0"

    def log_message(self, format_string: str, *args: Any) -> None:
        return

    @property
    def collector(self) -> grid_controllerCollector:
        return self.server.collector  # type: ignore[attr-defined]

    @property
    def html_bytes(self) -> bytes:
        return self.server.html_bytes  # type: ignore[attr-defined]

    def send_bytes(
        self,
        content: bytes,
        content_type: str,
        status: HTTPStatus = HTTPStatus.OK,
    ) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.send_header("Connection", "close")
        self.end_headers()
        self.close_connection = True
        try:
            self.wfile.write(content)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            # O navegador pode cancelar uma requisição ao recarregar a página.
            pass

    def send_json(
        self, payload: Any, status: HTTPStatus = HTTPStatus.OK
    ) -> None:
        content = json.dumps(
            payload, ensure_ascii=False, separators=(",", ":")
        ).encode("utf-8")
        self.send_bytes(content, "application/json; charset=utf-8", status)

    def do_GET(self) -> None:
        parsed = urlparse(self.path)

        if parsed.path in ("/", "/index.html"):
            self.send_bytes(self.html_bytes, "text/html; charset=utf-8")
            return

        if parsed.path == "/api/state":
            self.send_json(self.collector.get_state())
            return

        if parsed.path == "/api/history":
            query = parse_qs(parsed.query)
            try:
                hours = float(query.get("hours", ["24"])[0])
            except ValueError:
                hours = 24.0
            hours = min(max(hours, 1.0), 24.0 * 31.0)
            self.send_json(
                {
                    "state": self.collector.get_state(),
                    "points": self.collector.get_history(hours),
                }
            )
            return

        if parsed.path == "/api/latest":
            query = parse_qs(parsed.query)
            try:
                after_id = int(query.get("after", ["0"])[0])
            except ValueError:
                after_id = 0
            self.send_json(
                {
                    "state": self.collector.get_state(),
                    "points": self.collector.get_points_after(after_id),
                }
            )
            return

        self.send_json({"error": "Rota não encontrada"}, HTTPStatus.NOT_FOUND)



class QuietThreadingHTTPServer(ThreadingHTTPServer):
    """Servidor que ignora desconexões normais do navegador sem traceback."""

    daemon_threads = True
    allow_reuse_address = True

    def handle_error(self, request: Any, client_address: Any) -> None:
        error = sys.exc_info()[1]
        if isinstance(
            error,
            (BrokenPipeError, ConnectionResetError, ConnectionAbortedError),
        ):
            return
        super().handle_error(request, client_address)

def find_local_ip() -> str:
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


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Coletor CSV e painel web do controlador grid_controller_v1"
    )
    parser.add_argument("--device", default=DEVICE_URL)
    parser.add_argument("--poll", type=float, default=30.0)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--csv", default="grid_controller_stats.csv")
    parser.add_argument("--memory-days", type=float, default=31.0)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--http-port", type=int, default=8000)
    args = parser.parse_args()

    if args.poll < 5.0:
        parser.error("--poll deve ser pelo menos 5 segundos")
    if not 1 <= args.http_port <= 65535:
        parser.error("--http-port inválida")

    html_path = Path(__file__).with_name("grid_controller_mobile.html")
    if not html_path.exists():
        raise FileNotFoundError(f"Página não encontrada: {html_path}")

    csv_path = Path(args.csv)
    if not csv_path.is_absolute():
        csv_path = Path(__file__).resolve().parent / csv_path

    collector = grid_controllerCollector(
        device_url=args.device,
        csv_path=csv_path,
        poll_s=args.poll,
        timeout_s=args.timeout,
        memory_days=args.memory_days,
    )
    collector.start()

    server = QuietThreadingHTTPServer((args.bind, args.http_port), DashboardHandler)
    server.collector = collector  # type: ignore[attr-defined]
    server.html_bytes = html_path.read_bytes()  # type: ignore[attr-defined]

    local_ip = find_local_ip()
    print()
    print("grid_controller v1 - coletor iniciado")
    print(f"ESP:        {args.device}")
    print(f"CSV:        {csv_path}")
    print(f"Computador: http://127.0.0.1:{args.http_port}")
    print(f"Rede local: http://{local_ip}:{args.http_port}")
    print("Pressione Ctrl+C para encerrar.")
    print()

    try:
        server.serve_forever(poll_interval=0.5)
    except KeyboardInterrupt:
        print("\nEncerrando...")
    finally:
        server.shutdown()
        server.server_close()
        collector.stop()


if __name__ == "__main__":
    main()
