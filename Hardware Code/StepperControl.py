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
    # basic control test
    stepper = Stepper_28BYJ_48(4,17,27,22) # change pins accordingly, use GPIO #
    stepper._update()
    input_val = input("Please input a number of steps to turn: ")

    for i in range(0,input_val):
        stepper.step()
        #print(stepper._currentStep)
        sleep(0.001)

    inner_wheel = input("Please input the inner wheel steering angle in degrees: ")
    outer_wheel = input("Please input the outer wheel steering angle in degrees: ")

    steering_angle_1 = np.arctan(1.0 / ((1.0/np.tan(np.pi*inner_wheel/180.0)) + (7.25/(2*9.75))))
    steering_angle_2 = np.arctan(1.0 / ((1.0/np.tan(np.pi*outer_wheel/180.0)) - (7.25/(2*9.75))))
    steering_angle = (steering_angle_1 + steering_angle_2) / 2.0
    steering_angle = steering_angle * 180.0 / np.pi

    print("Moving back to initial position...")
    sleep(2)
    stepper.changeDirection()

    for i in range(0,input_val):
        stepper.step()
        #print(stepper._currentStep)
        sleep(0.001)

    print("Individual Steering Angles: {} {} Steering angle: {}".format(steering_angle_1 * 180 / np.pi, steering_angle_2 * 180 / np.pi, steering_angle))

    raw_input("Press any key to continue")

if __name__ == "__main__":
    main()