import ssl
import socket
import threading

HOST = "0.0.0.0"
PORT = 5762

CA_CERT = "certs/ca.crt"
SERVER_COMBINED = "certs/server-combined.pem"

def handle_client(conn):
    print(" Drone connected over TLS")

    try:
        while True:
            data = conn.recv(4096)
            if not data:
                break
            # Forward data to COM5 (Mission Planner)
            try:
                gcs_socket.sendall(data)
            except:
                pass
    except Exception as e:
        print("Error:", e)
    finally:
        conn.close()
        print("Drone disconnected")

def listen_gcs():
    while True:
        data = gcs_socket.recv(4096)
        if data:
            for c in clients:
                c.sendall(data)

def start_server():
    context = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
    context.load_verify_locations(CA_CERT)
    context.load_cert_chain(SERVER_COMBINED)
    context.verify_mode = ssl.CERT_REQUIRED

    sock = socket.socket()
    sock.bind((HOST, PORT))
    sock.listen(5)

    print(f" GCS mTLS Server Running on port {PORT}")

    while True:
        raw, addr = sock.accept()
        try:
            conn = context.wrap_socket(raw, server_side=True)
            print("Client verified:", addr)
            clients.append(conn)
            threading.Thread(target=handle_client, args=(conn,), daemon=True).start()
        except ssl.SSLError as e:
            print("TLS error:", e)

# Setup COM5 forwarding
import serial
gcs_serial = serial.Serial("COM5", 57600, timeout=0.1)
gcs_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

clients = []

# Relay COM5 → Drone
def serial_to_drone():
    while True:
        data = gcs_serial.read(gcs_serial.in_waiting or 1)
        if data:
            for c in clients:
                try:
                    c.sendall(data)
                except:
                    pass

# Start everything
threading.Thread(target=serial_to_drone, daemon=True).start()
start_server()
