import argparse
import json
import yaml
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

from .detection import load_config
from .service import ScanService


def make_server(service, host="127.0.0.1", port=8765):
    assets = Path(__file__).parent / "static"

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, format_string, *arguments):
            return

        def respond(self, status, payload, content_type="application/json; charset=utf-8"):
            if isinstance(payload, (dict, list)):
                payload = json.dumps(payload, ensure_ascii=False, allow_nan=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            try:
                self.wfile.write(payload)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_GET(self):
            route = urlsplit(self.path).path
            if route == "/api/state":
                self.respond(200, service.state())
            elif route == "/api/config":
                self.respond(200, service.get_config())
            elif route in ("/", "/app.js", "/roi.js", "/style.css"):
                name, mime = {
                    "/": ("index.html", "text/html"),
                    "/app.js": ("app.js", "text/javascript"),
                    "/roi.js": ("roi.js", "text/javascript"),
                    "/style.css": ("style.css", "text/css"),
                }[route]
                self.respond(200, (assets / name).read_bytes(), mime + "; charset=utf-8")
            else:
                self.respond(404, {"error": "路径不存在"})

        def do_POST(self):
            route = urlsplit(self.path).path
            if route not in ("/api/config", "/api/config/save"):
                self.respond(404, {"error": "路径不存在"})
                return
            origin = self.headers.get("Origin")
            if origin and origin != "http://" + self.headers.get("Host", ""):
                self.respond(403, {"error": "只允许本页面修改参数"})
                return
            if self.headers.get_content_type() != "application/json":
                self.respond(415, {"error": "请使用 application/json"})
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 16384:
                    raise ValueError("请求长度无效")
                changes = json.loads(self.rfile.read(length))
                self.respond(200, service.update_config(changes, persist=route == "/api/config/save"))
            except OSError as error:
                self.respond(500, {"error": f"保存失败: {error}"})
            except (yaml.YAMLError, KeyError) as error:
                self.respond(400, {"error": f"YAML 配置无效: {error}"})
            except (ValueError, TypeError, OverflowError) as error:
                self.respond(400, {"error": str(error)})

    return ThreadingHTTPServer((host, port), Handler)


def main():
    parser = argparse.ArgumentParser(description="MID-360 无线二维扫描 Web 测试")
    parser.add_argument("--ws-url", default="ws://192.168.20.199:6060")
    parser.add_argument("--frame", default="scan_link")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--config", type=Path, default=Path(__file__).resolve().parents[1] / "config" / "scan_web.yaml")
    arguments = parser.parse_args()
    if urlsplit(arguments.ws_url).scheme not in ("ws", "wss") or not urlsplit(arguments.ws_url).hostname:
        parser.error("--ws-url 必须是有效的 ws:// 或 wss:// 地址")
    if not arguments.frame.lstrip("/"):
        parser.error("--frame 不能为空")
    try:
        service = ScanService(load_config(arguments.config), arguments.ws_url, arguments.frame, config_path=arguments.config)
        server = make_server(service, arguments.host, arguments.port)
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.exit(1, f"启动失败: {error}\n")
    service.start()
    print(f"MID-360 二维扫描: http://{arguments.host}:{server.server_port}", flush=True)
    print(f"数据源: {arguments.ws_url} /scan [{arguments.frame}]；Ctrl+C 退出", flush=True)
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        service.close()
