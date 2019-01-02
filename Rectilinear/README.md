# Rectilinear

## RectilinearMotion

### Setup

1. Set `NUM_SERVOS` to the number of servos in your robot.

2. Set the values of `forwardAngles` to the angles at which each servo (from head to tail) faces straight forward.

3. Modify the body of the `getServoPortNumber` function to reflect the scheme used for connecting servos on your robot. (servo numbers are assumed to begin from `0` at the head of the snake and increase down the snake body).

4. Review other variables at the beginning of the sketch (the function of each is documented in the sketch).

5. The sketch should be ready to compile and deploy to the Arduino.

### Robot Control

The sketch utilizes the four-button keyfob with the following layout:

**A:** Forward

**B:** Reverse

**C:** Left turn

**D:** Right turn

**Note:** Forward/reverse movement and turns cannot currently be performed at the same time.
