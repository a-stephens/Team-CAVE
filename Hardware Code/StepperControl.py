from gpiozero import OutputDevice
from time import sleep
import numpy as np

class Stepper_28BYJ_48:
    def __init__(self, in1, in2, in3, in4):
        self._in1 = OutputDevice(in1) # blue
        self._in2 = OutputDevice(in2) # pink
        self._in3 = OutputDevice(in3) # yellow
        self._in4 = OutputDevice(in4) # orange
        self._pins = [self._in1, self._in2, self._in3, self._in4]
        self.steps = 0

        self._direction = -1

        self._currentStep = 0

        self._sequence = [
            # from datasheet, in sequence as is: CW
            # reverse sequence will just go CCW
            [0,0,0,1],
            [0,0,1,1],
            [0,0,1,0],
            [0,1,1,0],
            [0,1,0,0],
            [1,1,0,0],
            [1,0,0,0],
            [1,0,0,1]
        ]

        self._maxStep = len(self._sequence)

    def changeDirection(self):
        self._direction *= -1

    def step(self):
        self._currentStep += self._direction
        self.steps += self._direction
        if self._currentStep >= self._maxStep:
            self._currentStep = 0
        if self._currentStep < 0:
            self._currentStep = self._maxStep - 1
        self._update()

    def _update(self):
        for i_pin, pin in enumerate(self._pins):
            if self._sequence[self._currentStep][i_pin] == 1:
                pin.on()
            else:
                pin.off()

def main():
    STEPS_TO_DEG = 0.0202

    # basic control test
    stepper = Stepper_28BYJ_48(4,17,27,22) # change pins accordingly, use GPIO #
    stepper._update()

    in_deg = input("Input the desired steering angle in degrees")
    des_steer = in_deg*np.pi/180.0
    starting_steer = 0

    step_err = des_steer - starting_steer
    while np.abs(step_err) > 0.1:
        stepper_angle = stepper._direction * stepper.steps * STEPS_TO_DEG
        step_err = (des_steer * 180.0 / np.pi) - stepper_angle

        if step_err < 0:
            stepper.changeDirection()
            stepper.step()
            stepper.changeDirection()
        else:
            stepper.step()
        
        print("Angle: {}deg Goal: {}deg".format(stepper_angle, des_steer*180.0/np.pi))
        print(stepper.steps)

        if np.abs(stepper.steps) >= 1500:
            break

        sleep(0.001)

    raw_input("Press any key to continue")

if __name__ == "__main__":
    main()