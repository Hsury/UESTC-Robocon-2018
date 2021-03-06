import swig_control

class Wrapper():
    '''Cui's Algorithm Wrapper'''
    def __init__(self):
        self.globalParam = swig_control.CGlobalParam()
        self.car = swig_control.CBasicChassis()
        self.RouteControlInit = swig_control.RouteControlInit_t()
    
    def setPosition(self, x, y, z):
        self.globalParam.updateGlobalParam(int(x * 1000), int(y * 1000), z, 0)
    
    def setGoal(self, x, y, z, typeNum=0):
        print('------------------Set Route Target -------------------')
        # self.RouteControlInit.route_type = input("Route Type (0:Line 1:ARC_ACLK -1:ARC_CLK) = ")
        # self.RouteControlInit.end_point.x = input('Target Point X = ')
        # self.RouteControlInit.end_point.y = input('Target Point Y = ')
        # self.RouteControlInit.end_point.ang = input('Target Point ANG = ')
        self.RouteControlInit.route_type = typeNum
        self.RouteControlInit.end_point.x = int(x * 1000)
        self.RouteControlInit.end_point.y = int(y * 1000)
        self.RouteControlInit.end_point.ang = z
        print('------------------Set Route Speed -------------------')
        # self.RouteControlInit.forward_init.unif_v = input("Route Uinf V = ")
        # self.RouteControlInit.forward_init.end_v = input("Route End V = ")
        # self.RouteControlInit.forward_init.aclt = input("Route Aclt = ")
        # self.RouteControlInit.forward_init.decr = input("Route Decr = ")
        self.RouteControlInit.forward_init.unif_v = 2000
        self.RouteControlInit.forward_init.end_v = 0
        self.RouteControlInit.forward_init.aclt = 2000
        self.RouteControlInit.forward_init.decr = 2000
        if self.RouteControlInit.route_type == 0:
            print('Route Type Line')
            # Lock Line Params
            self.RouteControlInit.lock_pid_init.Kp = 4
            self.RouteControlInit.lock_pid_init.Ki = 0
            self.RouteControlInit.lock_pid_init.Kd = 0
            self.RouteControlInit.lock_pid_init.max_out = 5000
            self.RouteControlInit.lock_pid_init.min_out = -5000
            self.RouteControlInit.lock_pid_init.iteg_max = 3000
            self.RouteControlInit.lock_pid_init.dead_zone = 0
            # Forward Params
            self.RouteControlInit.forward_init.kf = 100
            self.RouteControlInit.forward_init.kp_aclt = 6
            self.RouteControlInit.forward_init.kp_decr = 5
            self.RouteControlInit.forward_init.kd_aclt = 0
            self.RouteControlInit.forward_init.kd_decr = 0
            self.RouteControlInit.forward_init.maxout = 3000
            self.RouteControlInit.forward_init.minout = -3000
        else:
            print('------------------Set Arc Center -------------------')
            # Arc Forward Params
            self.RouteControlInit.center.x = input('Arc Center X = ')
            self.RouteControlInit.center.y = input('Arc Center Y = ')
            # Lock Arc Params
            self.RouteControlInit.lock_pid_init.Kp = 3
            self.RouteControlInit.lock_pid_init.Ki = 0
            self.RouteControlInit.lock_pid_init.Kd = 0
            self.RouteControlInit.lock_pid_init.max_out = 2000
            self.RouteControlInit.lock_pid_init.min_out = -2000
            self.RouteControlInit.lock_pid_init.iteg_max = 3000
            self.RouteControlInit.lock_pid_init.dead_zone = 0
            # Forward Params
            self.RouteControlInit.forward_init.kf = 100
            self.RouteControlInit.forward_init.kp_aclt = 4
            self.RouteControlInit.forward_init.kp_decr = 4
            self.RouteControlInit.forward_init.kd_aclt = 0
            self.RouteControlInit.forward_init.kd_decr = 0
            self.RouteControlInit.forward_init.maxout = 3000
            self.RouteControlInit.forward_init.minout = -3000
        # Route Forward Init (Line And Arc)
        # Rotate Forward Params
        self.RouteControlInit.rotate_forward_init.kf = 100
        self.RouteControlInit.rotate_forward_init.kp_aclt = 3
        self.RouteControlInit.rotate_forward_init.kp_decr = 3
        self.RouteControlInit.rotate_forward_init.kd_aclt = 0
        self.RouteControlInit.rotate_forward_init.kd_decr = 0
        self.RouteControlInit.rotate_forward_init.maxout = 3000
        self.RouteControlInit.rotate_forward_init.minout = -3000
        #delete sleep(1)
        self.RouteControlInit.start_point = self.globalParam.py_GetCurPos_copy()
        self.route = swig_control.CRouteControl(self.RouteControlInit, self.car, self.globalParam)

    def resolve(self):
        if self.RouteControlInit.route_type == 0:
            self.route.control_line()
            self.route.lock_line_ctl()
        else:
            self.route.control_arc()
            self.route.lock_arc_ctl()
        self.route.control_rotate()
        self.spd_buffer = self.car.py_getSpeed2Pub()
        self.car.ClearWheelSpeed()
        if swig_control.getCurrentTime() > self.route.py_GetEndTime():
            return [0] * 3
        else:
            return [self.spd_buffer.spd_x, self.spd_buffer.spd_y, self.spd_buffer.spd_ang]
