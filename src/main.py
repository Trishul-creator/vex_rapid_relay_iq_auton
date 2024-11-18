#region VEXcode Generated Configuration
from vex import *

#Brain should be defined by default
brain = Brain()

#Robot configuration code
brain_inertial = Inertial()
left_motor = Motor(Ports.PORT11, 1.0, False)
right_motor = Motor(Ports.PORT5, 1.0, True)

drivetrain = SmartDrive(left_motor, right_motor, brain_inertial, 200)
motor_group_1_motor_a = Motor(Ports.PORT1, False)
motor_group_1_motor_b = Motor(Ports.PORT7, True)
motor_group_1 = MotorGroup(motor_group_1_motor_a, motor_group_1_motor_b)
motor_8 = Motor(Ports.PORT8, False)

vexcode_initial_drivetrain_calibration_completed = False
def calibrate_drivetrain():
    #Calibrate the Drivetrain Inertial
    global vexcode_initial_drivetrain_calibration_completed
    sleep(200, MSEC)
    brain.screen.print("Calibrating")
    brain.screen.next_row()
    brain.screen.print("Inertial")
    brain_inertial.calibrate()
    while brain_inertial.is_calibrating():
        sleep(10, MSEC)
    vexcode_initial_drivetrain_calibration_completed = True
    brain.screen.clear_screen()
    brain.screen.set_cursor(1, 1)

#Calibrate the Drivetrain
calibrate_drivetrain()

#endregion VEXcode Generated Robot Configuration

# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       gurumurthyvenkataraman                                       #
# 	Created:      11/16/2024, 6:36:09 PM                                       #
# 	Description:  IQ2 project                                                  #
#                                                                              #
# ---------------------------------------------------------------------------- #

from vex import *

#Constants for Robot
ENCODER_COUNTS_PER_REVOLUTION = 360
WHEEL_CIRCUMFERENCE_MM = 200
WHEEL_DIAMETER_MM = WHEEL_CIRCUMFERENCE_MM / math.pi



class PIDController:
    def __init__(self):
        self.Kp = 0.5
        self.Ki = 0
        self.Kd = 0
        self.prev_error = 0
        self.integral = 0
        self.setpoint = 0
    def set_setpoint(self, setpoint):
        """Set the target distance in mm."""
        self.setpoint = setpoint
    def update(self, current_position):
        """Calculate the PID output based on the current position."""
        # Calculate the error
        error = self.setpoint - current_position
        
        #Proportional term
        P_term = self.Kp * error
        
        #Integral term
        self.integral += error
        I_term = self.Ki * self.integral
        
        #Derivitave term
        D_term = self.Kd * (error - self.prev_error)
        
        #Calculate total output
        output = P_term + I_term + D_term
        
        self.prev_error = error
        
        return output
    
class AutonomousDriving:
    def __init__(self):
        self.points = 0
        self.goal_number = 0
        self.one_switch_point = 1
        self.one_switch_increment = 4
        
    # Function to convert encoder counts to distance in mm
    def encoder_to_mm(self, encoder_counts):
        #Converts the number of encoder ticks to distance (in mm)
        return (encoder_counts / ENCODER_COUNTS_PER_REVOLUTION) * WHEEL_CIRCUMFERENCE_MM
    
    #Drive function with PID
    def drive_pid(self, direction, target_distance_mm):
        """
        Function to drive the robot a specific distance using PID control.
        
        Parameters:
            - target_distance_mm: Target distance in millimeters to move
            - direction: Specify which direction the motors should roll
        """
        # Initialize the PID Controller
        pid = PIDController()
        pid.set_setpoint(target_distance_mm) # Set the target distance
        
        #Reset motor encoders to start from zero position
        left_motor.set_position(0, DEGREES)
        right_motor.set_position(0, DEGREES)
        
        #Main control loop
        while True:
            #Get the current position from both motors (in degrees)
            left_position = left_motor.position(DEGREES)
            right_position = right_motor.position(DEGREES)
            
            left_position_mm = self.encoder_to_mm(left_position)
            right_position_mm = self.encoder_to_mm(right_position)
            #Convert the encoder positions to distance in mm
            current_distance = (left_position_mm + right_position_mm)/2
            
            #Get the PID output
            pid_output = pid.update(current_distance)
            
            #Limit the PID output to the motor's valid range (-100 to 100 percent speed)
            motor_speed = max(min(pid_output, 100), -100)
            
            #Set the motor speeds to move towards the target position
            left_motor.set_velocity(motor_speed, PERCENT)
            right_motor.set_velocity(motor_speed, PERCENT)
            
            #Spin both motors forward
            left_motor.spin(direction)
            right_motor.spin(direction)
            
            # Stop when target position is reached (with a tolerance of 10 mm)
            if abs(current_distance - pid.setpoint) < 10:
                left_motor.stop()
                right_motor.stop()
                print("Target reached")
                break
            
            #Wait for a short time before the next iteration
            time.sleep(0.02)
                
    # Set Parameters for autonomous drive
    def set_params(self):
        #set velocity and turn parameters
        drivetrain.set_drive_velocity(100, PERCENT)
        print("Drivetrain velocity set to 100%")
        drivetrain.set_turn_velocity(95, PERCENT)
        print("Drivetrain turn velocity set to 95%")
        motor_group_1.set_velocity(100, PERCENT)
        print("Collecting Motor velocity set to 100%")
        motor_8.set_velocity(100, PERCENT)
        print("Shooting Motor velocity set to 100%")
    
    #Collect ball initially
    def initial_collect(self):
        #Drive forward and collect the ball
        self.drive_pid(FORWARD, 370)
        motor_group_1.spin_for(FORWARD, 2300, DEGREES)
        self.drive_pid(FORWARD, 100)
        
        #Play sound for collected ball
        brain.play_sound(SoundType.ALARM2)
        
        #Print collection of ball
        brain.screen.clear_screen()
        brain.screen.print("Ball collected")
        print("Initial ball collection has been completed")
    
    #Score both balls initially there
    def initial_shoot(self):
        #Drive and score first ball
        drivetrain.turn_for(LEFT, 86.5, DEGREES)
        self.drive_pid(REVERSE, 800)
        motor_8.spin_for(REVERSE, 1800, DEGREES)
        
        #Play sound for scoring
        brain.play_sound(SoundType.TADA)
        
        #Add score for clearing one_switch and scoring one ball
        self.points += 1 # - for pushing switch
        self.points += self.one_switch_increment
        self.goal_number += 1
        
        #Print scores
        brain.screen.clear_screen()
        print("1 switch cleared 1")
        print("{0}st Goal Shot !!".format(self.goal_number))
        print("The score is {0}".format(self.points))
        brain.screen.next_row()
        
        # Shoot second ball
        motor_group_1.spin_for(FORWARD, 900, DEGREES, wait=False)
        motor_8.spin_for(REVERSE, 2100, DEGREES)
        
         #Play sound for scoring
        brain.play_sound(SoundType.TADA)
        
        #Add points for second ball
        self.points += self.one_switch_increment
        
        #Print score
        brain.screen.clear_screen()
        brain.screen.print("2nd Goal Shot !!")
        print("2nd Goal Shot !!, It added {0} points !".format(self.one_switch_increment))
        print("Now the score is {0}".format(self.points))
        
    #Collect and score balls loop
    def shoot(self, times):
        for i in range(times): 
            #Collect ball
            brain_inertial.calibrate()
            self.drive_pid(FORWARD, 900)
            motor_group_1.spin_for(FORWARD, 8.7, TURNS, wait=False)
            wait(1.5, SECONDS)
            
            #Play sound for collecting ball
            brain.play_sound(SoundType.ALARM2)
            
            #Score ball
            self.drive_pid(REVERSE , 950)
            motor_8.spin_for(REVERSE, 5.2, TURNS)
            
            #Play sound for scoring
            brain.play_sound(SoundType.TADA)
            
            #Calculate score 
            self.points += self.one_switch_increment
            
            #Print score
            brain.screen.clear_screen()
            brain.screen.print("{0} Gaol Shot !!!".format(i+3))
            print("{0} Goal Shot !!! It added {1} points !".format(i+3, self.one_switch_increment))
            print("Now the score is {0}".format(self.points))
            
            #Don't hog CPU
            wait(0.1, SECONDS)
            
        brain.screen.print("The total number of pionts in this auton game is {0}".format(self.points))
        print("The total number of pionts in this auton game is {0}".format(self.points))
            
class TimerClass:
    def timerfunc(self, seconds):
        x = seconds
        while(True): # When timer goes off
            if(x==0):
                print("0")
                brain.play_sound(SoundType.POWER_DOWN) # Play power down sound
                brain.program_stop() # type: ignore
            elif(x ==60 or x==50 or x==40 or x==30 or x==20 or x<=10):
                print(x)
            x -=1
            wait(1, SECONDS)
            
def run_auton():
    auton_drive = AutonomousDriving()
    brain_inertial.calibrate()
    auton_drive.set_params()
    auton_drive.initial_collect()
    auton_drive.initial_shoot()
    auton_drive.shoot(6)
    
countdown = TimerClass()
timer_thread = Thread(countdown.timerfunc, (60,))
auton_thread = Thread(run_auton)
            


