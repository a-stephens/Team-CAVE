from gpiozero import OutputDevice
from gpiozero import PWMOutputDevice
from gpiozero import DigitalInputDevice
from time import sleep


class FIT_Motor:
    def __init__(self, in1, in2, in3):
        self.direction = OutputDevice(in1, initial_value=True)  # direction Pin
        self.speed = PWMOutputDevice(in2, initial_value=1, frequency=980)      # speed Pin
        self.encoder = DigitalInputDevice(in3)
        self.total_time = 1.0/(self.speed.frequency/2.0)

    def changeDirection(self):
        self.direction.toggle()

    def setStop(self):
        self.speed.on()

    def setSpeed(self, speed):
        speed_val = abs(speed) / 20.8
        if speed_val < 0.01:
            print("Speed exceeds maximum limit")
            self.setStop()
        else:
            self.speed.blink(on_time=self.total_time-speed_val*self.total_time, off_time=self.total_time*speed_val)

    def readEncoder(self, val):
        rpm = 0
        for i in range(8):
            if val == 1:
                meas_time = self.encoder.active_time
            if val == 0:
                meas_time = self.encoder.inactive_time
            if meas_time == None:
                meas_time = 0
            if meas_time > 0:
                rpm += 60.0 / (45.0*6.0*2.0*meas_time) # some math to get r/min
            else:
                rpm += 0
            sleep(0.010)
        return rpm / 8.0
        

def main():
    # basic control test
    DC_Motor = FIT_Motor(16,12,10)     # change pins accordingly, use GPIO #

    raw_input("Press any key to begin")
    DC_Motor.setSpeed(20.8)
    #DC_Motor.update()
    print("full speed")
    for i in range(20):
        sleep(0.020)
        print(DC_Motor.readEncoder(1) * 7.85 / 60.0)
        

    DC_Motor.setStop()
    #DC_Motor.update()
    print("stahp")
    sleep(2)

    DC_Motor.changeDirection()
    DC_Motor.setSpeed(2)
    #DC_Motor.update()
    print("change dir")
    for i in range(20):
        sleep(0.020)
        print(DC_Motor.readEncoder(1) * 7.85 / 60.0)

    DC_Motor.setStop()
    print("stop")
    raw_input("Press any key to exit")

if __name__ == "__main__":
    main()