# # Source: Electrocredible.com, Language: MicroPython

from machine import Pin,I2C,PWM
from bmp280 import *
from pres import *
import utime

class Submarine:
    def __init__(self, Ts):
        self.refreshPeriodms = Ts

        self.water_pres = Pressure(21, 20)

        self.bus = I2C(0,scl=Pin(1),sda=Pin(0),freq=200000)
        self.bmp = BMP280(self.bus, addr=0x76)
        self.bmp.use_case(BMP280_CASE_WEATHER)
        
        self._bltInLED = Pin(25, Pin.OUT)
        
        self.encoder = RotaryEncoder(5, 4, pulsesPerTurn=11*20.4)
        self.motor = Motor(6, 7, 8)
        self.depthController = Control(10, 20, 0, Ts=self.refreshPeriodms)

        # self.depthActuator TODO: implement
    def start(self):
        self._timer = machine.Timer(-1)
        self._timer.init(period=self.refreshPeriodms, mode=machine.Timer.PERIODIC, callback=self.update)

    def status(self, state):
        self._bltInLED.value(state)
        
    def readAllSensors(self):
        return [self.bmp.temperature, self.bmp.pressure, self.water_pres.pressure]
    
    def home(self, speed=50):
        self.diagnosis()
        print("Home... ", end="")
        self.motor.speed = speed
        
        # Move until stops
        self.motor.move(True)
        prevPos = -9999
        while not self.encoder.position == prevPos:
            prevPos = self.encoder.position
            utime.sleep_ms(400)
            
        # Move back slightly
        self.motor.move(False)
        utime.sleep_ms(100)
        self.motor.stop(True)
        utime.sleep_ms(100)
        
        self.encoder.setHome()
        
        utime.sleep_ms(1000)
        print("done")
    
    def conductTest(self):
        utimeSampleInterval = 200000
        desired_speeds = (88, 101)
        
        fname = "testRes.txt"
        f = open(fname, mode='w')
        f.close()
        
        print(f"Started test for speeds in range {desired_speeds}")
#         self.home()
        for i in range(*desired_speeds):
            self.motor.speed = i
            dataRecorded = []
            
            lastPos = self.encoder.position
            
            for direction in [False, True]:
                utimeStart = utime.ticks_us()
                lastSampleutime = utimeStart - utimeSampleInterval/2
                
                self.motor.move(direction)
                
                # Record data every T 
                while True:
                    if utime.ticks_us() - lastSampleutime >= utimeSampleInterval:
                        frame = [self.encoder.position, \
                                 utime.ticks_diff(utime.ticks_us(), utimeStart)/1000, \
                                 self.motor.speed, \
                                 direction]
                        dataRecorded.append(frame)
                        print(frame)
                        
                        lastSampleutime = utime.ticks_us()
                        
                        if abs(self.encoder.position - lastPos) < 0.001:
                            break
                        lastPos = self.encoder.position
                        
                self.motor.stop(True)
                utime.sleep_ms(1400)
            
            with open(fname, mode='a') as f:
                f.write(f"{dataRecorded}")
            print(f"	Run at speed {self.motor.speed} completed")
            
        print("Test was finished successfully")
            
    def diagnosis(self):
        # TODO: implement
        # wigle and decide if stuck
        # find out if let loose
        print("Diagnosis start:")
        utime.sleep_ms(1000)
        print("	check if stuck... ", end="")
        
        print("pass")
        print("	check if loose... ", end="")
        
        print("pass")

    def update(self, timer=None):
        reading = self.encoder.position
        command = self.depthController.update(reading)

        direction = False
        if command < 0:
            direction = True
        
        self.motor.speed = abs(command)
        self.motor.move(direction)

    def __del__(self):
        self.encoder.__del__()
        self.motor.stop()
        self.motor.__del__()
        self._timer.deinit()
        
class Control:
    def __init__(self, kp, ki, kd, *, Ts=200, maxIntegral=2):
        self._setpoint = 0
        
        self._lastError = 0
        self._integral = 0
        self._derivative = 0
        self._proportional = 0
        self._command = 0

        self._kp = kp
        self._ki = ki
        self._kd = kd

        self._Ts = Ts
        self.maxIntegral = maxIntegral

    def update(self, processValue):
        error = self.setpoint - processValue

        # Calculate errors
        Ts = self._Ts/1000
        self._proportional = error*self._kp
        self._integral += error*Ts
        self._derivative = (error - self._lastError)/Ts

        # Saturate integral
        if self._integral > self.maxIntegral:
            self._integral = self.maxIntegral
        elif self._integral < -self.maxIntegral:
            self._integral = -self.maxIntegral

        self._lastError = error

        self._command = self._proportional*self._kp + self._integral*self._ki + self._derivative*self._kd
        return self._command

    def set_setpoint(self, val):
        # self._integral = 0
        self._setpoint = val

    def get_setpoint(self):
        return self._setpoint

    setpoint = property(get_setpoint, set_setpoint)


class Motor:
    def __init__(self, pinSpeed, pinLeft, pinRight, speed=50):
        self._pinSpeed = PWM(Pin(pinSpeed, Pin.OUT))
        self._pinSpeed.freq(100)
        self._pinLeft = Pin(pinLeft, Pin.OUT)
        self._pinRight = Pin(pinRight, Pin.OUT)
        
        self._speed = speed
        
    def move(self, direction):
        if direction:
            self._pinRight.value(True)
            self._pinLeft.value(False)
        else:
            self._pinRight.value(False)
            self._pinLeft.value(True)
            
    def stop(self, useFastStop=True):
        if useFastStop:
            self._pinRight.value(True)
            self._pinLeft.value(True)
        else:
            self._pinRight.value(False)
            self._pinLeft.value(False)
        
    def set_speed(self, val):
        # In %
        if val > 100:
            val = 100
        if val < 0:
            val = 0
        self._speed = val
        self._pinSpeed.duty_u16(int(65535*val/100))
        
    def get_speed(self):
        return self._speed
    
    def __del__(self):
        self._pinSpeed.deinit()
    
    speed = property(get_speed, set_speed)
        

class RotaryEncoder:
    def __init__(self, pinA, pinB, *, pulsesPerTurn = 11):
        self._phaseA = Pin(pinA, Pin.IN, Pin.PULL_UP)
        self._phaseB = Pin(pinB, Pin.IN, Pin.PULL_UP)
        self._counter = 0
        self._pulsesPerTurn = pulsesPerTurn
        
        self._aLastState = self._phaseA.value()
        self._phaseA.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self.onEncoderTurned)
        
    def onEncoderTurned(self, _):
        aState = self._phaseA.value()
        if aState != self._aLastState:
            if self._phaseB.value() != aState:
                self._counter -= 1
            else:
                self._counter += 1
            self._aLastState = aState
            
    def setHome(self):
        self._counter = 0
        
    @property
    def position(self):
        return self._counter/self._pulsesPerTurn
    
    def __del__(self):
        self._phaseA.irq(handler=None)


sub = Submarine(20)
sub.status(True)
# print(f"Temperature | Air Pressure | Water Pressure")
# 0 - 25 revolutions
try:
    sub.home()
    sub.depthController.setpoint = 10
    sub.start()
#     sub.conductTest()
    while True:
        print(f"pos: {sub.encoder.position} | sp: {sub.depthController.setpoint} |" +
            f"lastE: {sub.depthController._lastError} | i: {sub.depthController._integral} | " +
            f"cmnd: {sub.depthController._command}")
#         sub.motor.move(True)
#         sub.conductTest()
#         print("{:^11} | {:^12} | {:^14} | {:^10}".format(*sub.readAllSensors(), sub.encoder.position))
#         print(f"{str(bmp.temperature) + ' C' : <11}", end='')
#         print(f" | {str(bmp.pressure/100) + ' Pa' : <12}", end='')
#         print(f" | {water_pres.pressure/100}", end='\n')
        utime.sleep_ms(1000)
        
except KeyboardInterrupt as e:
    print(f"File {__name__} was interrupted by {e}") # __file__ not defined

finally:
    sub.status(False)
    sub.__del__()

