#the below code is for macos for testing the server side code:-

#!/usr/bin/env python3
import ssl
import socket
import sys

SERVER_IP = "0.0.0.0"
SERVER_PORT = 5762            # mTLS listen port
FORWARD_HOST = "127.0.0.1"
FORWARD_PORT = 5760          # Mission Planner TCP endpoint

SERVER_CERT = "certs/server.crt"
SERVER_KEY = "certs/server.key"
CA_CERT = "certs/ca.crt"

def forward(src, dst):
    try:
        while True:
            data = src.recv(4096)
            if not data:
                break
            dst.sendall(data)
    except Exception:
        pass

def main():
    print("Starting mTLS GCS server...")

    # Create SSL context for server-side mTLS
    context = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
    context.verify_mode = ssl.CERT_REQUIRED
    context.load_cert_chain(certfile=SERVER_CERT, keyfile=SERVER_KEY)
    context.load_verify_locations(CA_CERT)

    # plain TCP listener to accept encrypted client
    bindsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    bindsock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    bindsock.bind((SERVER_IP, SERVER_PORT))
    bindsock.listen(5)
    print(f"mTLS server listening on {SERVER_IP}:{SERVER_PORT}")

    while True:
        newsock, addr = bindsock.accept()
        print("Connection from", addr)
        try:
            connstream = context.wrap_socket(newsock, server_side=True)
            print("mTLS handshake ok; peer cert:", connstream.getpeercert())
        except ssl.SSLError as e:
            print("TLS handshake failed:", e)
            newsock.close()
            continue

        # create local forward to Mission Planner: connect to 127.0.0.1:5760
        forward_sock = socket.create_connection((FORWARD_HOST, FORWARD_PORT))
        # bidirectional copy
        import threading
        t1 = threading.Thread(target=forward, args=(connstream, forward_sock), daemon=True)
        t2 = threading.Thread(target=forward, args=(forward_sock, connstream), daemon=True)
        t1.start(); t2.start()
        t1.join(); t2.join()
        connstream.close(); forward_sock.close()
        print("Client disconnected")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Server exiting")
        sys.exit(0)
