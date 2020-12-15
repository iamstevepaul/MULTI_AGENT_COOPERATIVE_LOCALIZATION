from EKF import EKF

class Robot():

    def __init__(self,
                 x=0.0,
                 y=0.0,
                 vx=0.0,
                 vy=0.0,
                 ultra_sonic=0.0,
                 bearing=0.0,
                 gps=0.0,
                 odometer=0.0):
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        self.ultra_sonic = ultra_sonic
        self.bearing = bearing
        self.gps = gps
        self.odometer = odometer

    def update_position(self, dt):
        self.x += self.vx * dt
        self.y += self.vy * dt

    def sense_ultra_sonic(self):
        pass

    def sense_bearing(self):
        pass

    def sense_gps(self):
        pass

    def update_odometer(self):
        pass