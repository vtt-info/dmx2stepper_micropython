import socket
import time

HOST = "raspberrypi.local"
PORT = 9999

while True:
    try:
        with socket.create_connection((HOST, PORT), timeout=5) as sock:
            print(f"Connected to {HOST}:{PORT}")
            buf = ""
            while True:
                chunk = sock.recv(256).decode()
                if not chunk:
                    raise ConnectionResetError("server closed connection")
                buf += chunk
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.strip()
                    x = None if line == "null" else int(line)
                    print(x)
    except (OSError, ConnectionResetError) as e:
        print(f"Disconnected: {e}, retrying in 2s...")
        time.sleep(2)
