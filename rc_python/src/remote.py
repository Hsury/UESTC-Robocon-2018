from socket import *
import threading
import json

class Remote():
    '''UESTC 2018 Robocon Team
    Remote Package
    '''
    def __init__(self, dash, flow, port=2018):
        self._dash = dash
        self._flow = flow
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
                    recvJson = json.loads(recvBuffer.decode())
                    for cmd in recvJson:
                        eval(cmd)
                    sendBuffer = {'dist': self._dash.dist,
                                  'goal': self._dash.goal,
                                  'locked': self._dash.locked,
                                  'position': self._dash.position,
                                  'speed': self._dash.speed,
                                  'flow_busy': self._flow.status['busy'],
                                  'flow_id': self._flow.status['id'],
                                  'flow_task': self._flow.status['task']}
                    conn.send(json.dumps(sendBuffer).encode())
            except BlockingIOError:
                pass
            except:
                break
            sleep(0.01)
        self._dash.lock()
