from gpiozero import OutputDevice
from gpiozero import PWMOutputDevice
import time as sleep


class FIT_Motor:
    def __init__(self, in1, in2):
        self.speed = OutputDevice(in1)          # Speed Pin
        self.direction = PWMOutputDevice(in2)   # Direction Pin
        self.pins = [self._in1, self._in2]
        self.direction = 1
        self.speed.value(255)

    def changeDirection(self):
        self.direction = 1 - self.direction

    def setStop(self):
        self.speed.value(255)

    def setSpeed(self, speed):
        speed_val = 255 - int(abs(speed) * 12.26)
        if speed_val < 0:
            print("Speed exceeds maximum limit")
            self.setStop()
        else:
            self.speed.value(speed_val)

    def update(self):
        for i_pin, pin in enumerate(self.pins):
            if self.sequence[self.currentStep][i_pin] == 1:
                pin.on()
            else:
                pin.off()


def main():
    # basic control test
    DC_Motor = FIT_Motor(12,16)     # change pins accordingly, use GPIO #
    DC_Motor.setSpeed(20.8)
    DC_Motor.update()
    sleep(.5)

    DC_Motor.setStop()
    DC_Motor.update()
    sleep(.5)

    DC_Motor.changeDirection()
    DC_Motor.setSpeed(10.4)
    DC_Motor.update()
    sleep(.5)


if __name__ == "__main__":
    main()