import network
import usocket
import _thread
import time
import uerrno

# CONFIG ZONE
AP_ESSID = 'UESTC_RC'
AP_PASSWORD = 'UESTC_RC2018'
AP_HIDDEN = False
AP_AUTHMODE = network.AUTH_WPA2_PSK
SOCKET_PORT = 2018
# CONFIG ZONE

def guard():
    while True:
        conn, addr = s.accept()
        _thread.start_new_thread(handler, (conn, addr))
        time.sleep_ms(1)

def handler(conn, addr):
    while True:
        try:
            conn.send('Hello, World!')
        except OSError as exc:
            if exc.args[0] == uerrno.ECONNRESET:
                print('Connection Reset, Break.')
                break
            else:
                print(exc.args[0])
        time.sleep(1)
    _thread.exit()

ap = network.WLAN(network.AP_IF)
ap.active(True)
ap.config(essid=AP_ESSID, hidden=AP_HIDDEN, authmode=AP_AUTHMODE, password=AP_PASSWORD)

s = usocket.socket(usocket.AF_INET, usocket.SOCK_STREAM)
s.bind(('0.0.0.0', SOCKET_PORT))
s.listen(64)

_thread.start_new_thread(guard, ())
