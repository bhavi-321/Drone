#!/usr/bin/env python3
import ssl, socket, serial, threading, time, sys

# Serial connected to Pixhawk
SERIAL_PORT = '/dev/serial0'
SERIAL_BAUD = 57600

# GCS mTLS server (your MacBook IP)
SERVER_HOST = '192.168.137.33'   # <<-- replace with your Mac IP
SERVER_PORT = 5762

CA_CERT = 'certs/ca.crt'
CLIENT_CERT = 'certs/client.crt'
CLIENT_KEY = 'certs/client.key'

def forward_serial_to_tls(ser, ssock):
    try:
        while True:
            data = ser.read(ser.in_waiting or 1)
            if data:
                ssock.sendall(data)
    except Exception as e:
        print("serial->tls error", e)

def forward_tls_to_serial(ser, ssock):
    try:
        while True:
            data = ssock.recv(4096)
            if not data:
                break
            ser.write(data)
    except Exception as e:
        print("tls->serial error", e)

def main():
    # open serial
    ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.1)
    print("Opened serial", SERIAL_PORT)
    # build SSL context
    context = ssl.create_default_context(ssl.Purpose.SERVER_AUTH, cafile=CA_CERT)
    context.load_cert_chain(certfile=CLIENT_CERT, keyfile=CLIENT_KEY)
    context.check_hostname = False
    # connect
    raw = socket.create_connection((SERVER_HOST, SERVER_PORT), timeout=15)
    ssock = context.wrap_socket(raw, server_hostname=SERVER_HOST)
    print("Connected TLS to", SERVER_HOST)
    # start forwarding
    t1 = threading.Thread(target=forward_serial_to_tls, args=(ser, ssock), daemon=True)
    t2 = threading.Thread(target=forward_tls_to_serial, args=(ser, ssock), daemon=True)
    t1.start(); t2.start(); t1.join(); t2.join()
    ssock.close(); ser.close()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Exiting")
        sys.exit(0)
