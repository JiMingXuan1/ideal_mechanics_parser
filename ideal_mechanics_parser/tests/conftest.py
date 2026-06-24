import sys, os, socket, subprocess, time, threading
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


@pytest.fixture(scope="session")
def server_port():
    """Start server.py on a free port and yield the port number."""
    server_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    server_script = os.path.join(server_dir, "server.py")

    s = socket.socket()
    s.bind(("", 0))
    port = s.getsockname()[1]
    s.close()

    proc = subprocess.Popen(
        [sys.executable, server_script, str(port)],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, cwd=server_dir,
    )
    for _ in range(200):
        try:
            import urllib.request
            urllib.request.urlopen(f"http://127.0.0.1:{port}/", timeout=1)
            break
        except Exception:
            time.sleep(0.1)
    else:
        proc.kill()
        pytest.fail("Server did not start")

    yield port

    proc.kill()
    proc.wait(timeout=5)
