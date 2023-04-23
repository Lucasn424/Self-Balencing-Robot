import pyb
from pyb import LED, ADC, Pin, Timer	# Use various class libraries in pyb
import time
from oled_938 import OLED_938
from mpu6050 import MPU6050


pot = ADC(Pin('X11'))		# 5k ohm potentiometer to ADC input on pin X11

i2c = pyb.I2C(2, pyb.I2C.MASTER)
devid = i2c.scan()				# find the I2C device number
oled = OLED_938(
    pinout={"sda": "Y10", "scl": "Y9", "res": "Y8"},
    height=64,
    external_vcc=False,
    i2c_devid=i2c.scan()[0],
)

oled.poweron()
oled.init_display()

# Define pins to control motor
A1 = Pin('X3', Pin.OUT_PP)		# Control direction of motor A
A2 = Pin('X4', Pin.OUT_PP)
PWMA = Pin('X1')				# Control speed of motor A
B1 = Pin('X7', Pin.OUT_PP)		# Control direction of motor B
B2 = Pin('X8', Pin.OUT_PP)
PWMB = Pin('X2')				# Control speed of motor B

#Code below defines the motor outputs which allows the PID output to deterimine speed and direciton of motors
def A_forward(value):
	A1.low()
	A2.high()
	motorA.pulse_width_percent(value)
def A_back(value):
	A2.low()
	A1.high()
	motorA.pulse_width_percent(value)
        
def A_stop():
	A1.high()
	A2.high()
        
def B_forward(value):
	B2.low()
	B1.high()
	motorB.pulse_width_percent(value)

def B_back(value):
	B1.low()
	B2.high()
	motorB.pulse_width_percent(value)
	
def B_stop():
	B1.high()
	B2.high()


# Configure timer 2 to produce 1KHz clock for PWM control
tim = Timer(2, freq = 1000)
motorA = tim.channel (1, Timer.PWM, pin = PWMA)
motorB = tim.channel (2, Timer.PWM, pin = PWMB)

imu = MPU6050(1, False) # Use I2C port 1 on Pyboard


# Pitch angle calculation using complementary filter
def pitch_estimate(pitch, dt, alpha):
    theta = imu.pitch ()
    pitch_dot = imu.get_gy() 
    pitch = alpha*(pitch + pitch_dot*dt) + (1-alpha)*theta
    return (pitch, pitch_dot)

class PIDC:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki
        self.error_last = 0       # Global variables to remember various states of controller
        self.tic = pyb.millis()
        self.error_sum = 0

    def getPWM(self, target, pitch, pitch_dot):

        # error input
        error = target - pitch          # e[n]
    
        # derivative input
        derivative = -pitch_dot         # negative feedback


        toc = pyb.millis()
        dt = (toc-self.tic)*0.001       # find dt as close to when used as possible
        # Integration input 
        self.error_sum += error*dt              
 
        #   Output 
        PID_output = (self.Kp * error) + (self.Ki * self.error_sum) + (self.Kd * derivative)
 
        # Store previous values 
        self.error_last = error
        self.tic = toc

        pwm_out = min(abs(PID_output), 100)             # Make sure pwm is less than 100 
 
        if PID_output > 0:                              # Output direction (need to check)
            direction = 'forward'
        elif PID_output < 0:
            direction = 'back'
        else: 
            direction = 'stop'

        return pwm_out, direction
    

# The code below uses the potentionemeter to set the P value, the I value, and the D value
# Since the vlaues for P,I, and D only really work in a certain range, a scale factor has been applied to the potentiementer
# for each value so that it is easier to tune
trigger = pyb. Switch ()
scale = 26
scale2 = 120
scale3 = 1.2
while not trigger(): # wait to tune K
    time.sleep(0.001)
    K_p = pot.read() * scale / 4095 # use pot to set Kp
    oled.draw_text(0, 20, 'Kp = {:5.2f}'.format (K_p)) # display live value on oled
    oled.display ()
while trigger(): pass # wait for button release 
while not trigger(): # wait to tune Ki
    time.sleep (0.001)
    K_i = pot.read () * scale2 / 4095 # use pot to set Ki
    oled.draw_text(0, 30, 'Ki = {:5.2f}'.format (K_i)) # display live value on oled
    oled.display()
while trigger(): pass # wait for button release 
while not trigger (): # wait to tune Kd
    time.sleep (0.001)
    K_d = pot.read() * scale3 / 4095 # use pot to set Kd
    oled.draw_text(0, 40, 'Kd = {:5.2f}'.format (K_d)) # display live value on oled
    oled.display ()
while trigger (): pass # wait for button release

print( 'Button pressed. Running script.')
oled.draw_text(0, 20, "Button pressed. Running")
oled.display ()


Balance = PIDC(K_p, K_i, K_d) #Calling the PIDC class and inputing the PID vlaues set from potentiometer

pitch=float(0.0)
try:
# Try to handle exception
    tic1 = pyb.micros ()
    
    while True:
# infinite loop
        dt = pyb.micros () - tic1
        if (dt > 5000):
            alpha = 0.99 #Alpha value for the complementary filter, uses much more of the gyroscope
            theta = imu.pitch ()
            pitch, pitch_dot =  pitch_estimate(pitch, dt/1000000, alpha)  #calling the pitch estimte fucntion
            #to get the current pitch, and the curreunt pitch rate of change 
            tic1 = pyb.micros ()
            MotorOutput = Balance.getPWM(1.5,pitch,pitch_dot) #IMU was not level so had to set a 1.5º target angle
            direc = MotorOutput[1] # the the direction, it is either 'forward' or 'back'
            sped = abs(MotorOutput[0]) 
            if sped>0:
                if direc == 'forward':
                    A_forward(sped)
                    B_forward(sped)
                if direc == 'back':
                    A_back(sped)
                    B_back(sped)


finally:
    print('here3') #used to debug and make sure it could make it through all the code
    A_stop()
    B_stop()




