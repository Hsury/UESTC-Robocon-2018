import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib.gridspec import GridSpec

def cubicBezier(p0, p1, p2, p3, t):
    return pow(1 - t, 3) * p0 + \
           3 * pow(1 - t, 2) * t * p1 + \
           3 * (1 - t) * pow(t, 2) * p2 + \
           pow(t, 3) * p3

def cubicBezierDt(p0, p1, p2, p3, t):
    return 3 * pow(1 - t, 2) * (p1 - p0) + \
           6 * (1 - t) * t * (p2 - p1) + \
           3 * pow(t, 2) * (p3 - p2)

def cubicBezierDt2(p0, p1, p2, p3, t):
    return 6 * (1 - t) * (p2 - 2 * p1 + p0) + \
           6 * t * (p3 - 2 * p2 + p1)

print(r'                                                                                                    ')
print(r'  /\\\        /\\\  /\\\\\\\\\\\\\\\     /\\\\\\\\\\\    /\\\\\\\\\\\\\\\        /\\\\\\\\\         ')
print(r'  \/\\\       \/\\\ \/\\\///////////    /\\\/////////\\\ \///////\\\/////      /\\\////////         ')
print(r'   \/\\\       \/\\\ \/\\\              \//\\\      \///        \/\\\         /\\\/                 ')
print(r'    \/\\\       \/\\\ \/\\\\\\\\\\\       \////\\\               \/\\\        /\\\                  ')
print(r'     \/\\\       \/\\\ \/\\\///////           \////\\\            \/\\\       \/\\\                 ')
print(r'      \/\\\       \/\\\ \/\\\                     \////\\\         \/\\\       \//\\\               ')
print(r'       \//\\\      /\\\  \/\\\              /\\\      \//\\\        \/\\\        \///\\\            ')
print(r'         \///\\\\\\\\\/   \/\\\\\\\\\\\\\\\ \///\\\\\\\\\\\/         \/\\\          \////\\\\\\\\\  ')
print(r'            \/////////     \///////////////    \///////////           \///              \/////////  ')
print(r'                                                                                                    ')
print(r'                       ===== Bezier Curve Analyzer For ABU Robocon 2018 =====                       ')
print(r'                               Copyright (c) 2018 HsuRY <i@hsury.com>                               ')
print(r'                                                                                                    ')

print('Cubic Bezier')
p0List = input("P0 Coordinate: ").split(',')
p0x = float(p0List[0])
p0y = float(p0List[1])
p1List = input("P1 Coordinate: ").split(',')
p1x = float(p1List[0])
p1y = float(p1List[1])
p2List = input("P2 Coordinate: ").split(',')
p2x = float(p2List[0])
p2y = float(p2List[1])
p3List = input("P3 Coordinate: ").split(',')
p3x = float(p3List[0])
p3y = float(p3List[1])
duration = int(input("Duration: "))

t = np.arange(0, duration, 0.005)

plt.figure(num='Bezier Curve Analyzer', figsize=(16, 9))
gs = GridSpec(9, 16)
gs.update(hspace=1.5, wspace=1.5)

plt.subplot(gs[0: 9, 0: 9])
plt.title('Game Field')
plt.imshow(mpimg.imread('data/map.png'), extent=[0, 14, 0, 14])
plt.plot(cubicBezier(p0x, p1x, p2x, p3x, t / duration), cubicBezier(p0y, p1y, p2y, p3y, t / duration), 'w--')

plt.subplot(gs[0: 3, 10: 13])
plt.title('Position X')
plt.plot(t, cubicBezier(p0x, p1x, p2x, p3x, t / duration))

plt.subplot(gs[0: 3, 13: 16])
plt.title('Position Y')
plt.plot(t, cubicBezier(p0y, p1y, p2y, p3y, t / duration))

plt.subplot(gs[3: 6, 10: 13])
plt.title('Velocity X')
plt.plot(t, cubicBezierDt(p0x, p1x, p2x, p3x, t / duration))

plt.subplot(gs[3: 6, 13: 16])
plt.title('Velocity Y')
plt.plot(t, cubicBezierDt(p0y, p1y, p2y, p3y, t / duration))

plt.subplot(gs[6: 9, 10: 13])
plt.title('Acceleration X')
plt.plot(t, cubicBezierDt2(p0x, p1x, p2x, p3x, t / duration))

plt.subplot(gs[6: 9, 13: 16])
plt.title('Acceleration Y')
plt.plot(t, cubicBezierDt2(p0y, p1y, p2y, p3y, t / duration))

plt.show()
