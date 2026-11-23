from pybricks.parameters import Port, Color, Stop
from pybricks.tools import wait
from umath import pi
from robot import Robot



WHEEL_DIAMETER = 62.4
AXLE_TRACK = 150.2
MY_MISSIONS = [1, 2, 3, 4, 5]

LEFT_DRIVE = Port.F
RIGHT_DRIVE = Port.B
LEFT_ATTACH = Port.E
RIGHT_ATTACH = Port.A
LEFT_LINE = Port.C
RIGHT_LINE = Port.D

class Bioglow(Robot):
    LEFT_HOME = 77
    RIGHT_HOME = 0

    def fast_drive(self, degrees, accel=1500):
        orig = self.drive_base.distance_control.limits()
        self.drive_base.distance_control.limits(acceleration=accel)
        self.drive(self.deg_to_mm(degrees), power=100, accel=accel, gyro=True)
        self.drive_base.distance_control.limits(*orig)

    def arm_safe(self, side, angle, power=75, max_ms=1000):
        motor = self.left_attach if side == "left" else self.right_attach
        self.move_attach(side, angle, power=power, wait_for=False)
        for _ in range(max_ms // 10):
            wait(10)
            if motor.done() or motor.stalled():
                break

    def back_until_white(self, speed=100, threshold=70, max_mm=400, heading=None):
        if heading is None:
            heading = self.hub.imu.heading()
        self.drive_base.stop()
        mm_per_wheel_deg = pi * self.wheel_dia / 360
        wheel_speed = speed / mm_per_wheel_deg
        l0 = self.left_motor.angle()
        r0 = self.right_motor.angle()
        hits = 0
        went = 0
        while went < max_mm:
            err = heading - self.hub.imu.heading()
            corr = 10 * err
            self.left_motor.run(-wheel_speed + corr)
            self.right_motor.run(-wheel_speed - corr)
            l = self.line_l.reflection()
            r = self.line_r.reflection()
            went = abs((self.left_motor.angle() - l0) + (self.right_motor.angle() - r0)) / 2 * mm_per_wheel_deg



            hits = hits + 1 if (l > threshold and r > threshold) else 0
            if hits >= 3:
                break
            wait(5)
        self.left_motor.brake()
        self.right_motor.brake()
        self.drive_base.use_gyro(True)

    def left_to(self, target, power=50, wait_for=True):
        self.move_attach("left", target - self.left_attach.angle(), power=power, wait_for=wait_for)

    def right_to(self, target, power=50, wait_for=True):
        self.move_attach("right", target - self.right_attach.angle(), power=power, wait_for=wait_for)

    def run_1_mission_1(self):
        self.left_attach.run_until_stalled(-400, then=Stop.HOLD, duty_limit=30)
        self.left_attach.reset_angle(0)
        self.right_attach.reset_angle(0)
        self.arm_to("left", 75, power=75, wait_for=False)
        self.drive(self.deg_to_mm(1020), power=90, gyro=True)
        self.gyro_turn_to(-12, speed=45, accel=100)
        self.drive(self.deg_to_mm(160), power=50, gyro=False)
        self.arm_to("left", 275, power=50)

    def run_1_mission_2(self):
        self.drive(self.deg_to_mm(90), power=50, gyro=False)
        self.gyro_turn_to(35, speed=45, accel=100)
        self.drive(self.deg_to_mm(220), power=60, gyro=False)
        self.arm_to("right", -100, power=30)
        self.arm_to("right", 0, power=50, wait_for=False)
        self.arm_to("left", 0, power=50, wait_for=False)

    def run_1_mission_3(self):
        self.drive(self.deg_to_mm(-200), power=80, gyro=True)
        self.gyro_turn_to(-25, speed=45, accel=100)
        self.drive(self.deg_to_mm(-500), power=80, gyro=True)
        self.gyro_turn_to(3, speed=45, accel=100)
        self.drive(self.deg_to_mm(200), power=80, gyro=True)
        self.gyro_turn_to(13, speed=45, accel=100)
        self.arm_to("left", 120, power=30)
        self.drive(self.deg_to_mm(-135), power=80, gyro=False)
        self.gyro_turn_to(-5, speed=45, accel=100)
        self.drive(self.deg_to_mm(-179), power=80, gyro=False)
        self.arm_to("left", 280, power=30, wait_for=False)
        self.gyro_turn_to(0, speed=45, accel=100)
        self.arm_to("right", -150, power=30)
        self.drive(self.deg_to_mm(130), power=50, gyro=False)
        self.arm_to("right", -100, power=20)
        self.gyro_turn_to(30, speed=45, accel=100)
        self.drive(self.deg_to_mm(-250), power=40, gyro=False)
        self.arm_to("right", -255, power=20)
        self.drive(self.deg_to_mm(-120), power=80, gyro=False)
        self.arm_to("right", -75, power=50, wait_for=False)
        self.gyro_turn_to(-15, speed=45, accel=100)
        self.drive(self.deg_to_mm(-538), power=90, gyro=False)

    def run_1(self):
        self.run_1_mission_1()
        self.run_1_mission_2()
        self.run_1_mission_3()

    def run_1_1(self):
        self.move_attach("right", -240, power=75, wait_for=False)
        self.drive(self.deg_to_mm(1360), power=90, gyro=True)
        self.move_attach("right", 160, power=75)
        wait(500)

      

    def run_2_mission_1(self):


        self.left_attach.reset_angle(0)
        self.drive(self.deg_to_mm(-60), power=40, gyro=False, stall=True, timeout=600)
        wait(100)
        self.drive_base.use_gyro(True)
        self.drive_base.reset(0, 0)
        self.drive(self.deg_to_mm(50), power=40, gyro=True)
        self.turn_left(87, speed=50)
        wait(50)
        self.drive(self.deg_to_mm(610), power=50, gyro=True)
        self.drive(self.deg_to_mm(-120), power=30, gyro=True)
        self.left_to(-160, power=30)
        self.drive(self.deg_to_mm(60), power=30, gyro=True)
        self.left_to(-300, power=30)
        self.drive(self.deg_to_mm(-110), power=30, accel=3000, gyro=True)
        self.drive(self.deg_to_mm(80), power=30, gyro=True)
        self.left_to(-100, power=80)




        self.drive(self.deg_to_mm(-100), power=50, gyro=True)
        self.gyro_turn_to(-87, speed=45, accel=100, min_rate=20)

        self.back_until_white(speed=100, threshold=60, heading=-87)
        self.gyro_turn_to(0, speed=100, accel=200)
        self.drive(self.deg_to_mm(-150), power=40, gyro=False, stall=True, timeout=600)
        wait(100)


        self.wall_dev = self.hub.imu.heading()
        if abs(self.wall_dev) <= 3:
            self.wall_dev = 0
        self.drive_base.use_gyro(True)
        self.drive_base.reset(0, 0)
        self.drive(self.deg_to_mm(50), power=40, gyro=True)


    def run_2_mission_2(self):


        self.turn_left(38 + self.wall_dev, speed=100, accel=200, min_rate=20)
        wait(50)
        self.drive_base.reset(self.drive_base.distance(), self.hub.imu.heading())
        self.drive(self.deg_to_mm(647), power=60, gyro=True)
        self.gyro_turn_to(-41.5 - self.wall_dev, speed=45, accel=100, min_rate=20)




        self.move_attach("right", 110, power=30)
        self.drive(self.deg_to_mm(-450), power=60, gyro=True)
        self.gyro_turn_to(-1 - self.wall_dev, speed=100, accel=200, min_rate=20)
        wait(50)
        self.drive(self.deg_to_mm(490), power=50, gyro=True)
        self.left_to(-200, power=30)
        self.drive(self.deg_to_mm(-100), power=60, gyro=True)

    def run_2_mission_3(self):
        self.turn_right(19, speed=100, accel=200)
        self.left_to(0, power=80, wait_for=False)
        self.drive(self.deg_to_mm(620), power=70, gyro=True)
        self.turn_left(90, speed=100, accel=200)
        self.drive(self.deg_to_mm(690), power=70, gyro=True)
        self.turn_left(65, speed=100, accel=200)
        self.drive(self.deg_to_mm(-300), power=30, gyro=True)
        self.drive(self.deg_to_mm(60), power=80, gyro=True)

    def run_2_mission_4(self):
        self.turn_right(47, speed=100, accel=200)
        self.drive(self.deg_to_mm(1000), power=70, gyro=True)
        self.turn_left(15, speed=100, accel=200)
        wait(50)
        self.drive(self.deg_to_mm(510), power=70, gyro=True)
        self.turn_right(35, speed=100, accel=200)
        wait(50)
        self.drive(self.deg_to_mm(560), power=70, gyro=True)
        self.left_to(-280, power=20)
        wait(500)
        self.left_to(0, power=80)

        self.drive(self.deg_to_mm(-100), power=70, gyro=True)
        self.turn_right(80, speed=100, accel=200)
        self.fast_drive(-1400)

    def run_2(self):
        self.run_2_mission_1()
        self.run_2_mission_2()
        self.run_2_mission_3()
        self.run_2_mission_4()
    
    def run_3_mission_1(self):
        self.drive(self.deg_to_mm(760), power=80, gyro=True)
        self.turn_to(0)
        self.drive(self.deg_to_mm(-583), power=80, accel=1200, gyro=True)

    def run_3_mission_2(self):
        self.gyro_turn(90, speed=100, accel=250)
        self.move_attach("left", 140, power=75, wait_for=False)
        self.drive(self.deg_to_mm(300), power=60, gyro=True)
        self.gyro_turn(-46, speed=80)
        self.drive(self.deg_to_mm(790), power=60, gyro=True)
        self.move_attach("right", -100, power=75)
        self.fast_drive(-400)
        self.gyro_turn(15, speed=100, accel=250)
        self.fast_drive(-700)

    def run_3_mission_3(self):
        self.gyro_turn(30, speed=100, accel=250)
        self.drive(self.deg_to_mm(1680), power=80, accel=800, gyro=True)
        self.drive(self.deg_to_mm(-160), power=90, gyro=True)
        self.arm_safe("left", -200, power=75)
        self.move_attach("left", 40, power=75)
        self.arm_safe("left", -80, power=75)
        self.move_attach("left", 120, power=100)
        self.fast_drive(-1300)

    def run_3(self):
        self.run_3_mission_1()
        self.run_3_mission_2()
        self.run_3_mission_3()       
 
    def run_4_mission_1(self):


        self.drive_base.use_gyro(True)
        self.drive_base.reset(0, 0)
        self.right_attach.reset_angle(0)
        self.right_to(-240, power=75, wait_for=False)
        self.move_attach("left", 140, power=50, wait_for=False)
        self.drive(self.deg_to_mm(1310), power=90, gyro=True)
        self.right_to(10, power=75)

    def run_4_mission_2(self):
        self.gyro_turn(-35, speed=30)
        self.drive(self.deg_to_mm(-72), power=50, gyro=True)
        self.move_attach("left", -140, power=30)
        self.drive(self.deg_to_mm(179), power=20, gyro=True)
        self.move_attach("left", 180, power=40)
        self.drive(self.deg_to_mm(-251), power=80, gyro=False)

    def run_4_mission_3(self):
        self.gyro_turn_to(94, speed=80, accel=150)
        self.drive(self.deg_to_mm(521), power=80, gyro=True)
        self.right_to(-190, power=75, wait_for=False)
        self.gyro_turn_to(-4, speed=80, accel=250)
        self.drive(self.deg_to_mm(220), power=40, gyro=True)
        self.right_to(-90, power=75)
        self.drive(self.deg_to_mm(-180), power=50, gyro=True)
        self.right_to(-190, power=75)
        self.push(self.deg_to_mm(1100), power=100, timeout=5000)

        for _ in range(2):
            self.drive(self.deg_to_mm(-50), power=60, gyro=True)
            self.push(self.deg_to_mm(70), power=100, timeout=800)
        self.drive(self.deg_to_mm(-269), power=50, gyro=True)
        self.gyro_turn_to(-140, speed=150, accel=400)
        self.right_to(10, power=75, wait_for=False)
        self.fast_drive(1256)

    def run_4(self):
        self.run_4_mission_1()
        self.run_4_mission_2()
        self.run_4_mission_3()
        
    def run_5_mission_1(self):
        self.fast_drive(1040)
        self.gyro_turn(88, speed=100, accel=250)
        self.drive(self.deg_to_mm(808), power=80, gyro=True)
        wait(100)
        self.drive(self.deg_to_mm(269), power=30, gyro=True)
        self.move_attach("right", -100, power=90)
        wait(100)
        self.move_attach("right", 100, power=30)
        self.move_attach("right", -100, power=30)
        self.move_attach("right", 100, power=30)
        self.drive(self.deg_to_mm(-100), power=40, gyro=True)

    def run_5(self):
        self.run_5_mission_1()

    def run_mission(self, mission_id):
        self.reset(0) 
        

        if mission_id == 1:
            self.run_1()
        elif mission_id == 2: 
            self.run_2()
        elif mission_id == 3:
            self.run_3()
        elif mission_id == 4:
            self.run_4()
        elif mission_id == 5:
            self.run_5()


        self.drive_base.stop()
        self.hub.light.on(Color.GREEN)
        wait(100)

bot = Bioglow(LEFT_DRIVE, RIGHT_DRIVE, LEFT_ATTACH, RIGHT_ATTACH, LEFT_LINE, RIGHT_LINE, WHEEL_DIAMETER, AXLE_TRACK)


if bot.line_l: bot.line_l.lights.off()
if bot.line_r: bot.line_r.lights.off()

bot.main_menu(MY_MISSIONS)