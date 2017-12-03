import network
sta_if = network.WLAN(network.STA_IF)
ap_if = network.WLAN(network.AP_IF)
sta_if.active(True)
sta_if.connect('Robocon', 'duoguanriben8')

sta_if.isconnected()
sta_if.ifconfig()

import webrepl_setup
