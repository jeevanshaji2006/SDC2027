# sprint_sim: drive-train simulation for the Sprint robot

Python model of one drive channel, built to design and tune the wheel-speed controller before the hardware exists.

```
battery -> PWM timer -> driver -> motor -> wheel
                                      |
                       encoder -> speed estimate -> controller -> (back to PWM timer)
```

## Setup
```
pip install -r requirements.txt
```
Keep all files in this one folder: they import each other by name (`import params as P`).

## Files
| File | What it does |
|---|---|
| `params.py` | Every number in one place. `(ASSUMED)` marks a guess to replace with a measurement. Contains `BUS_OPTION` (1 = 4S + 12 V motors, 2 = 6S + 24 V motors). |
| `battery.py` | Pack as a voltage source behind a resistance; bus voltage sags with total current. |
| `pwm.py` | STM32 timer: duty cap, enable gate, rounding to a whole compare value, DIR pin only changes at zero duty. |
| `driver.py` | BTS7960 as an averaging switch: motor sees direction x duty x bus voltage. |
| `motor.py` | Brushed DC motor + 19.2:1 gearbox: back-EMF, current, torque, friction, inertia. |
| `encoder.py` | Quadrature encoder with a wrapping 16-bit counter, and a speed estimator that averages over N ms. |
| `controller.py` | Wheel speed loop: acceleration limit -> feedforward -> PI -> duty, with anti-windup. |
| `run_open_loop.py` | Duty step into 1 and 4 motors, no controller. Plots speed, bus voltage, currents. |
| `run_encoder_test.py` | How accurate the encoder speed estimate is at cruise speed (1, 5, 10 ms windows) and a counter-wrap test. |
| `test_reversal.py` | Checks DIR never flips while PWM is on, and measures the hard-reversal current. |
| `run_speed_loop.py` | Closed-loop speed control on 4 wheels sharing one battery; compares open vs closed loop on a weak battery. |

## Run
```
python run_open_loop.py
python run_encoder_test.py
python test_reversal.py
python run_speed_loop.py
```
Plots are saved as PNG files in the same folder (not committed).

## Notes
- Encoder numbers (11 PPR, motor shaft) are ASSUMED: the drive BOM lists no encoder yet.
- `BUS_OPTION = 2` uses an ASSUMED 24 V motor (half the current, 4x the resistance of the 12 V one). The drive BOM motor is 12 V on 4S, so use `BUS_OPTION = 1` for that. The stall-current check in `run_speed_loop.py` only means something with option 1.
