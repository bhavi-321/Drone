#!/usr/bin/env python3
import ssl, socket, serial, threading, os, sys

SERIAL_PORT = '/dev/serial0'
SERIAL_BAUD = 57600

SERVER_HOST = '192.168.137.33'   # <-------Windows laptop IP
SERVER_PORT = 5762

CA_CERT = '/home/pi/certs/ca.crt'
CLIENT_CERT = '/home/pi/certs/client.crt'
CLIENT_KEY = '/home/pi/certs/client.key'

def forward_serial_to_tls(ser, ssock):
    while True:
        data = ser.read(ser.in_waiting or 1)
        if data:
            ssock.sendall(data)

def forward_tls_to_serial(ser, ssock):
    while True:
        data = ssock.recv(4096)
        if not data:
            break
        ser.write(data)

def main():

    # Validate certs
    for f in [CA_CERT, CLIENT_CERT, CLIENT_KEY]:
        if not os.path.exists(f):
            print("❌ Missing:", f)
            sys.exit(1)

    print(" Certificates OK")

    ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.1)
    print(" Pixhawk Serial Opened")

    context = ssl.create_default_context(
        ssl.Purpose.SERVER_AUTH,
        cafile=CA_CERT
    )
    context.load_cert_chain(certfile=CLIENT_CERT, keyfile=CLIENT_KEY)
    context.check_hostname = False
    context.verify_mode = ssl.CERT_REQUIRED

    print(" Connecting to GCS...")
    raw = socket.create_connection((SERVER_HOST, SERVER_PORT))
    ssock = context.wrap_socket(raw, server_hostname=SERVER_HOST)

    print(" TLS Connection Established!")

    t1 = threading.Thread(target=forward_serial_to_tls, args=(ser, ssock), daemon=True)
    t2 = threading.Thread(target=forward_tls_to_serial, args=(ser, ssock), daemon=True)

    t1.start()
    t2.start()

    t1.join()
    t2.join()

if __name__ == "__main__":
    main()
