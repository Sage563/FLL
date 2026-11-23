from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Direction, Stop, Color, Button
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from umath import pi, sqrt

class Robot:
    def __init__(self, left_m_port, right_m_port, 
                 attach_l_port=None, attach_r_port=None, 
                 line_l_port=None, line_r_port=None,
                 wheel_dia=56, axle_track=114, 
                 base_speed=300, base_accel=800):
        

        self.hub = PrimeHub()
        self.wheel_dia = wheel_dia
        self.axle_track = axle_track

        voltage = self.hub.battery.voltage()
        if voltage < 7400:
            self.hub.light.on(Color.RED)
        else:
            self.hub.light.on(Color.GREEN)

        def safe_init(device_class, port, *args, **kwargs):
            if port is None: return None
            try:
                return device_class(port, *args, **kwargs)
            except OSError:
                return None

        self.left_motor = safe_init(Motor, left_m_port, Direction.COUNTERCLOCKWISE)
        self.right_motor = safe_init(Motor, right_m_port)


        if self.left_motor and self.right_motor:
            self.drive_base = DriveBase(self.left_motor, self.right_motor, wheel_dia, axle_track)
            self.drive_base.use_gyro(True)
            self.drive_base.settings(straight_speed=base_speed, 
                                     straight_acceleration=base_accel,
                                     turn_rate=150, 
                                     turn_acceleration=300)
        else:
            self.drive_base = None

        
        self.left_attach = safe_init(Motor, attach_l_port)
        self.right_attach = safe_init(Motor, attach_r_port)
        self.line_l = safe_init(ColorSensor, line_l_port)
        self.line_r = safe_init(ColorSensor, line_r_port)

    def deg_to_mm(self, degrees):
        return (degrees / 360) * (self.wheel_dia * pi)

    def reset(self, angle=0):
        if not self.drive_base: return
        

        self.drive_base.stop()
        

        wait(200) 
        
        try:
            self.drive_base.reset(0, angle)
        except Exception:
            self.drive_base.reset(0, angle)
        self.planned_heading = angle
            
        self.hub.light.on(Color.CYAN)

    def reset_gyro(self, angle=0):
        if not self.drive_base: return        
        self.drive_base.use_gyro(True)

        while not self.hub.imu.stationary():
            wait(50)
        self.drive_base.reset(0, angle)
        self.hub.light.on(Color.CYAN)

    def wait_for_gyro(self, timeout=1000):
        timer = StopWatch()
        while not self.hub.imu.stationary():
            if timer.time() > timeout:
                break
            wait(50)
        self.hub.light.on(Color.GREEN)

    def run_mission(self, mission_id):
        raise NotImplementedError("You must define run_mission(self, mission_id) in main.py")

    def main_menu(self, mission_list):
        last_run = mission_list[0]
        
        while True:
            while any(self.hub.buttons.pressed()):
                wait(10)

            self.hub.system.set_stop_button(None)
            self.hub.display.char(str(last_run))
            self.hub.light.on(Color.CYAN)


            pressed = []
            while not any(pressed):
                pressed = self.hub.buttons.pressed()
                wait(10)
            

            while any(self.hub.buttons.pressed()):
                wait(10)


            if Button.BLUETOOTH in pressed:
                return
            elif Button.LEFT in pressed:
                idx = (mission_list.index(last_run) - 1) % len(mission_list)
                last_run = mission_list[idx]
            elif Button.RIGHT in pressed:
                idx = (mission_list.index(last_run) + 1) % len(mission_list)
                last_run = mission_list[idx]
            elif Button.CENTER in pressed:

                self.hub.system.set_stop_button(Button.CENTER)

                while any(self.hub.buttons.pressed()):
                    wait(10)
                self.hub.display.off()

                try:
                    self.run_mission(last_run)
                
                except SystemExit:
                    if self.drive_base: self.drive_base.stop()
                    self.hub.light.on(Color.RED)
                    wait(100)
                except Exception:
                    self.hub.light.on(Color.ORANGE)
                    wait(100)
                else:


                    idx = (mission_list.index(last_run) + 1) % len(mission_list)
                    last_run = mission_list[idx]




    def drive(self, distance, speed=None, power=None, accel=None, gyro=True, stall=False, torque_limit=40, wait_for=True, timeout=1500, then=Stop.BRAKE):
        if not self.drive_base: return
        

        orig_settings = self.drive_base.settings()

        max_limits = self.drive_base.distance_control.limits()
        max_phys_speed = max_limits[0] 
        max_phys_accel = max_limits[1]

        safe_max_speed = max_phys_speed * 0.9 

        max_speed = 1000 
        
        if power is not None:

            move_speed = (power / 100) * safe_max_speed 
            move_accel = accel if accel is not None else min(2000, max_phys_accel)
        else:
            move_speed = speed if speed else orig_settings[0]
            move_accel = accel if accel else orig_settings[1]



        self.drive_base.settings(
            straight_speed=min(move_speed, safe_max_speed),
            straight_acceleration=min(move_accel, max_phys_accel),
            turn_rate=orig_settings[2],
            turn_acceleration=orig_settings[3]
        )


        orig_left_limit = self.left_motor.control.limits()
        orig_right_limit = self.right_motor.control.limits()

        if stall:


            self.left_motor.control.limits(torque=torque_limit)
            self.right_motor.control.limits(torque=torque_limit)

        self.drive_base.use_gyro(gyro)
        if not stall and not wait_for:
            self.drive_base.straight(distance, wait=False, then=then)
        elif not stall:
            self.drive_base.straight(distance, wait=False, then=then)
            expected_ms = abs(distance) / min(move_speed, safe_max_speed) * 1000
            timer = StopWatch()
            while not self.drive_base.done():
                if timer.time() > expected_ms * 2 + 2000:
                    self.drive_base.brake()
                    break
                wait(10)
        else:
            self.drive_base.straight(distance, wait=False)
            timer = StopWatch()
            while not self.drive_base.done():
                if self.drive_base.stalled() or timer.time() > timeout: break 
                wait(10)

            self.drive_base.stop()

        self.drive_base.settings(*orig_settings)

        self.left_motor.control.limits(torque=orig_left_limit[2])
        self.right_motor.control.limits(torque=orig_right_limit[2])

    def arc(self, radius, distance=None, angle=None, speed=None, accel=None, power=50, wait_for=True):
        if not self.drive_base: return
        self.drive_base.use_gyro(False)
        
        orig_settings = self.drive_base.settings()

        max_limits = self.drive_base.distance_control.limits()
        max_phys_speed = max_limits[0] 
        max_phys_accel = max_limits[1]

        if power is not None:

            move_speed = (power / 100) * max_phys_speed 
            move_accel = accel if accel is not None else min(2000, max_phys_accel)
        else:
            move_speed = speed if speed else orig_settings[0]
            move_accel = accel if accel else orig_settings[1]



        self.drive_base.settings(
            straight_speed=min(move_speed, max_phys_speed),
            straight_acceleration=min(move_accel, max_phys_accel),
            turn_rate=orig_settings[2],
            turn_acceleration=orig_settings[3]
        )
        

        self.drive_base.arc(radius, distance=distance, angle=angle, wait=wait_for)

        self.drive_base.settings(*orig_settings)
        self.drive_base.use_gyro(True)        

    def turn_to(self, target_angle, speed=150, gyro=True, accel=None):
        self.drive_base.use_gyro(gyro)

        orig_settings = self.drive_base.settings()
        if accel:
            self.drive_base.settings(
                straight_speed=orig_settings[0],
                straight_acceleration=orig_settings[1],
                turn_rate=orig_settings[2],
                turn_acceleration=accel
            )



        timer = StopWatch()
        while timer.time() < 3000:
            error = target_angle - self.drive_base.angle()
            if abs(error) < 0.5: break

            rate = max(40, min(speed, abs(error) * 4))
            self.drive_base.drive(0, rate if error > 0 else -rate)
            wait(10)

        self.drive_base.brake()
        self.planned_heading = target_angle


        self.drive_base.settings(*orig_settings)

    def gyro_turn(self, degrees, speed=100, accel=None, min_rate=40):
        if not self.drive_base: return



        self.drive_base.stop()


        wheel_per_robot_deg = self.axle_track / self.wheel_dia



        if not accel: accel = 400

        start = self.hub.imu.heading()
        target = start + degrees
        direction = 1 if degrees > 0 else -1
        timer = StopWatch()
        while timer.time() < 3000:


            remaining = (target - self.hub.imu.heading()) * direction
            if remaining < 0.5: break
            ramp_up = min_rate + accel * timer.time() / 1000
            ramp_down = sqrt(2 * accel * remaining)

            rate = max(min_rate, min(speed, ramp_up, ramp_down))
            wheel = rate * wheel_per_robot_deg * direction
            self.left_motor.run(wheel)
            self.right_motor.run(-wheel)
            wait(10)
        self.left_motor.brake()
        self.right_motor.brake()




        self.drive_base.use_gyro(True)

        self.planned_heading = target

    def gyro_turn_to(self, heading, speed=100, accel=None, min_rate=40):
        self.gyro_turn(heading - self.hub.imu.heading(), speed=speed, accel=accel, min_rate=min_rate)

    def turn_by(self, degrees, speed=150, accel=None, gyro=True):
        if not self.drive_base: return
        self.drive_base.use_gyro(gyro)

        orig_settings = self.drive_base.settings()
        self.drive_base.settings(
            straight_speed=orig_settings[0],
            straight_acceleration=orig_settings[1],
            turn_rate=speed,
            turn_acceleration=accel if accel else orig_settings[3]
        )

        self.drive_base.turn(degrees)
        if getattr(self, "planned_heading", None) is not None:
            self.planned_heading += degrees


        self.drive_base.settings(*orig_settings)

    def turn_right(self, degrees, speed=100, accel=None, min_rate=40):
        self._turn_from_plan(abs(degrees), speed, accel, min_rate)

    def turn_left(self, degrees, speed=100, accel=None, min_rate=40):
        self._turn_from_plan(-abs(degrees), speed, accel, min_rate)

    def _turn_from_plan(self, degrees, speed, accel, min_rate=40):
        plan = getattr(self, "planned_heading", None)
        if plan is None: plan = self.hub.imu.heading()
        self.gyro_turn_to(plan + degrees, speed=speed, accel=accel, min_rate=min_rate)





    def turn_to_tank(self, target_angle, left_speed, right_speed):
        if not self.drive_base: return
        self.drive_base.use_gyro(True)
        
        current_angle = self.drive_base.angle()
        

        self.left_motor.run(left_speed)
        self.right_motor.run(right_speed)
        

        if target_angle > current_angle:
            while self.drive_base.angle() < target_angle:
                wait(10)
        else:
            while self.drive_base.angle() > target_angle:
                wait(10)
                
        self.drive_base.stop()

    def move_tank(self, left_speed, right_speed, time_ms):
        self.left_motor.run(left_speed)
        self.right_motor.run(right_speed)
        wait(time_ms)
        self.drive_base.stop()

    def drive_curve(self, distance, turn_rate, speed=None, accel=None, power=None, wait_for=True):
        self.drive_base.use_gyro(False)
        self.drive_base.reset()
        orig_settings = self.drive_base.settings()

        max_limits = self.drive_base.distance_control.limits()
        max_phys_speed = max_limits[0] 
        max_phys_accel = max_limits[1]

        if power is not None:

            move_speed = (power / 100) * max_phys_speed 
            move_accel = accel if accel is not None else min(2000, max_phys_accel)
        else:
            move_speed = speed if speed else orig_settings[0]
            move_accel = accel if accel else orig_settings[1]



        self.drive_base.settings(
            straight_speed=min(move_speed, max_phys_speed),
            straight_acceleration=min(move_accel, max_phys_accel),
            turn_rate=orig_settings[2],
            turn_acceleration=orig_settings[3]
        )


        actual_speed = move_speed if distance >= 0 else -move_speed
        

        self.drive_base.drive(actual_speed, turn_rate)

        self.drive_base.drive(actual_speed, turn_rate)
        
        if wait_for:
            while abs(self.drive_base.distance()) < abs(distance):
                wait(10)
            self.drive_base.stop()
            self.drive_base.use_gyro(True)

        self.drive_base.settings(*orig_settings)

    def push(self, distance, power=40, timeout=4000):
        if not self.drive_base: return
        self.drive_base.stop()


        target = abs(distance) / (self.wheel_dia * pi) * 360
        direction = 1 if distance > 0 else -1
        start_left = self.left_motor.angle()
        start_right = self.right_motor.angle()
        start_heading = self.hub.imu.heading()

        timer = StopWatch()
        while timer.time() < timeout:
            travelled = (abs(self.left_motor.angle() - start_left) +
                         abs(self.right_motor.angle() - start_right)) / 2
            if travelled >= target: break

            steer = (self.hub.imu.heading() - start_heading) * 3
            self.left_motor.dc(max(-100, min(100, direction * power - steer)))
            self.right_motor.dc(max(-100, min(100, direction * power + steer)))
            wait(10)

        self.left_motor.brake()
        self.right_motor.brake()




    def square_on_line(self, speed):
        if not (self.line_l and self.line_r): return
        self.left_motor.run(speed)
        self.right_motor.run(speed)
        l_done = r_done = False
        while not (l_done and r_done):
            if self.line_l.reflection() < 20: 
                self.left_motor.stop(); l_done = True
            if self.line_r.reflection() < 20: 
                self.right_motor.stop(); r_done = True
            wait(5)
        self.drive_base.reset(0, 0)

    def square_on_wall(self, speed, time_ms):

        self.drive_base.use_gyro(False)
    


        self.drive_base.drive(speed, 0)
    

        from pybricks.tools import wait
        wait(time_ms)
    

        self.drive_base.stop()
        self.reset_gyro()

    def square_until_stalled(self, speed, torque_limit=30):



        self.drive_base.use_gyro(False)
        self.drive_base.straight(-200, wait=False) 


        from pybricks.tools import wait
        while not self.drive_base.stalled():
            wait(10)
        


        self.drive_base.stop()
        self.left_motor.run_until_stalled(-speed, duty_limit=torque_limit)
        self.right_motor.run_until_stalled(-speed, duty_limit=torque_limit)

        self.drive_base.stop()
        self.reset_gyro()

    def square_until_stalled_raw(self, speed=150, power=25, timeout=1500):
        from pybricks.tools import wait, StopWatch
        timer = StopWatch()


        self.drive_base.stop()
        self.drive_base.use_gyro(False)



        self.left_motor.dc(-power)
        self.right_motor.dc(-power)



        while timer.time() < timeout:

            if abs(self.left_motor.speed()) < 10 and abs(self.right_motor.speed()) < 10:
                break
            wait(10)



        self.left_motor.run_until_stalled(-speed, duty_limit=power)
        self.right_motor.run_until_stalled(-speed, duty_limit=power)


        self.drive_base.stop() 
        self.reset_gyro()

    def square_back_raw_power(self, power=25, time_ms=1500):

        self.drive_base.stop()
        self.drive_base.use_gyro(False)

        self.left_motor.dc(-power)
        self.right_motor.dc(-power)


        from pybricks.tools import wait
        wait(time_ms)

        self.left_motor.stop()
        self.right_motor.stop()
        

        self.reset_gyro()

    def move_attach(self, side, angle, speed=None, accel=None,power=50, wait_for=True):
        orig_settings = self.drive_base.settings()

        max_limits = self.drive_base.distance_control.limits()
        max_phys_speed = max_limits[0] 
        max_phys_accel = max_limits[1]

        if power is not None:

            move_speed = (power / 100) * max_phys_speed 
            move_accel = accel if accel is not None else min(2000, max_phys_accel)
        else:
            move_speed = speed if speed else orig_settings[0]
            move_accel = accel if accel else orig_settings[1]



        self.drive_base.settings(
            straight_speed=min(move_speed, max_phys_speed),
            straight_acceleration=min(move_accel, max_phys_accel),
            turn_rate=orig_settings[2],
            turn_acceleration=orig_settings[3]
        )

        motor = self.left_attach if side == "left" else self.right_attach
        if motor:
            motor.run_angle(move_speed, angle, wait=wait_for, then=Stop.HOLD)

        self.drive_base.settings(*orig_settings)
        self.drive_base.use_gyro(True)


    LEFT_HOME = 0
    RIGHT_HOME = 0

    def arm_to(self, side, target, power=50, wait_for=True):
        motor = self.left_attach if side == "left" else self.right_attach
        if not motor: return
        target = target + (self.LEFT_HOME if side == "left" else self.RIGHT_HOME)
        speed = power / 100 * self.drive_base.distance_control.limits()[0]
        motor.run_target(speed, target, then=Stop.HOLD, wait=wait_for)

    def rotate_attach(self, side, rotations, speed=None, accel=None, power=50, wait_for=True):

        angle_in_degrees = rotations * 360
        

        self.move_attach(
            side=side, 
            angle=angle_in_degrees, 
            speed=speed, 
            accel=accel, 
            power=power, 
            wait_for=wait_for
        )