# Serpentine

## Serpentine12New

### Setup

1. The provided sketch uses the following control format:

   **A:** Forward

   **B:** Reverse

   **C:** Left turn

   **D:** Right turn

   If the `TWO_BUTTON_REVERSE_TURN_LAYOUT` flag is set, holding both **B** and **C** or **D** will result in reverse turns; otherwise, the forward/reverse state will be preserved when **C** or **D** is pressed.

2. Set `NUM_SERVOS` to the number of servos in your robot.

3. Set the values of `forwardAngles` to the angles at which each servo (from head to tail) faces straight forward.

4. Modify the body of the `getServoPortNumber` function to reflect the scheme used for connecting servos on your robot (servo numbers are assumed to begin from `0` at the head of the snake and increase down the snake body).

5. Review other variables at the beginning of the sketch (the function of each is documented in the sketch).

6. The sketch should be ready to compile and deploy to the Arduino.

### Robot Control

The sketch uses the four-button keyfob with the control layout specified in Step 1 of "Setup".

## SerpentineRev3

Older serpentine-gait sketch as revised by [jCecchetti](https://github.com/jCecchetti).

### Robot Control

The sketch uses the four-button keyfob with a similar control layout to the two-button-reverse-turn layout specified in Step 1 of "Setup" for "Serpentine12New"; however, two buttons must also be pressed for forward turns.