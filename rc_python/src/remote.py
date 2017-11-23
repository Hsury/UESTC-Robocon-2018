from socket import *
import threading
import json

class Remote():
    '''UESTC 2018 Robocon Team
    Remote Package
    '''
    def __init__(self, dash, feed, port=2018):
        self._dash = dash
        self._feed = feed
        self._s = socket(AF_INET, SOCK_STREAM)
        self._s.bind(('0.0.0.0', port))
        self._s.listen()
        listenThd = threading.Thread(target=self.__listen)
        listenThd.setDaemon(True)
        listenThd.start()
    
    def __listen(self):
        while True:
            conn, addr = self._s.accept()
            conn.setblocking(False)
            handleThd = threading.Thread(target=self.__handle, args=(conn, addr))
            handleThd.setDaemon(True)
            handleThd.start()
    
    def __handle(self, conn, addr):
        from time import sleep
        recvBuffer = ''
        while True:
            try:
                recvBuffer = conn.recv(1024)
                if recvBuffer:
                    recvJson = json.loads(recvBuffer)
                    for cmd in recvJson:
                        eval(cmd)
                    sendBuffer = {'dash_dist': self._dash.dist,
                                  'dash_goal': self._dash.goal,
                                  'dash_locked': self._dash.locked,
                                  'dash_position': self._dash.position,
                                  'dash_speed': self._dash.speed,
                                  'feed_status': self._feed.status}
                    conn.send(json.dumps(sendBuffer).encode())
            except BlockingIOError:
                pass
            except:
                break
            sleep(0.01)
        self._dash.lock()
