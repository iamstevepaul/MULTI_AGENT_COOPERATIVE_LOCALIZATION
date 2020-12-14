from EKF import EKF

class Robot():

    def __init__(self,
                 x,
                 y,
                 vx,
                 vy,
                 ultra_sonic,
                 bearing,
                 gps,
                 odometer):
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        self.ultra_sonic = ultra_sonic
        self.bearing = bearing
        self.gps = gps
        self.odometer = odometer

        def update_state(self):
            pass

        def sense_ultra_sonic(self):
            pass

        def sense_bearing(self):
            pass

        def sense_gps(self):
            pass

        def update_odometer(self):
            pass