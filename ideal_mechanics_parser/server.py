import sys, os, json, mimetypes, traceback
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from core.engine import Engine
from core.exceptions import TopologyError

FRONTEND_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "frontend")


class Handler(BaseHTTPRequestHandler):
    def _send_json(self, status, data):
        self.send_response(status)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.end_headers()
        self.wfile.write(json.dumps(data, ensure_ascii=False).encode("utf-8"))

    def _send_error(self, status, message):
        self._send_json(status, {"error": message})

    def _parse_body(self):
        length = int(self.headers.get("Content-Length", 0))
        return json.loads(self.rfile.read(length))

    def _serve_static(self, path):
        if path in ("/", ""):
            path = "/index.html"
        fp = os.path.join(FRONTEND_DIR, path.lstrip("/"))
        if not os.path.isfile(fp) or not fp.startswith(FRONTEND_DIR):
            fp = os.path.join(FRONTEND_DIR, "index.html")
        ct, _ = mimetypes.guess_type(fp)
        try:
            with open(fp, "rb") as f:
                data = f.read()
            self.send_response(200)
            self.send_header("Content-Type", ct or "application/octet-stream")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
        except OSError:
            self._send_error(404, "Not found")

    def do_GET(self):
        self._serve_static(urlparse(self.path).path)

    def do_POST(self):
        parsed = urlparse(self.path)

        try:
            topology = self._parse_body()
        except Exception as e:
            self._send_error(400, f"Invalid JSON: {e}")
            return

        if parsed.path == "/solve":
            try:
                from io_handler.parser import _validate
                _validate(topology)
                engine = Engine(topology)
                result = engine.run()
                self._send_json(200, result)
            except TopologyError as e:
                self._send_error(422, str(e))
            except (AssertionError, KeyError, ValueError) as e:
                self._send_error(422, str(e))
            except Exception as e:
                tb = traceback.format_exc()
                sys.stderr.write(tb + "\n")
                self._send_error(500, f"Engine error: {e}")

        elif parsed.path == "/solve/stream":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()

            try:
                from io_handler.parser import _validate
                _validate(topology)
                engine = Engine(topology)
                engine.run_stream(on_chunk=lambda c: self._send_sse(c))
                self._send_sse({"complete": True})
            except TopologyError as e:
                self._send_sse({"error": str(e), "complete": True})
            except (AssertionError, KeyError, ValueError) as e:
                self._send_sse({"error": str(e), "complete": True})
            except Exception as e:
                tb = traceback.format_exc()
                sys.stderr.write(tb + "\n")
                self._send_sse({"error": str(e), "complete": True})

        else:
            self._send_error(404, f"Not found: {parsed.path}")

    def _send_sse(self, chunk):
        msg = f"data: {json.dumps(chunk, ensure_ascii=False)}\n\n"
        self.wfile.write(msg.encode("utf-8"))
        self.wfile.flush()

    def log_message(self, fmt, *args):
        sys.stderr.write(f"[IMP] {args[0]} {args[1]} {args[2]}\n")


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8000
    HTTPServer(("0.0.0.0", port), Handler).serve_forever()


if __name__ == "__main__":
    main()
