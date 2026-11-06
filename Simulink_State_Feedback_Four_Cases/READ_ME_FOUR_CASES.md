# Four requested cases

Set MATLAB's Current Folder to **Simulink_State_Feedback_Four_Cases** and run:

```matlab
RUN_ALL
```

This runs exactly the four cases and opens the full-range and zoomed response plots. PNG and editable MATLAB FIG copies are saved in `figures` with names beginning `four_cases_`.

It also opens motor-torque plots with all eight motors (left/right hip roll,
hip pitch, knee and ankle). These include full-range and controlled-response
zoom views, saved as `motor_torques_*`. Torques are the actual physical plant
states, including standing holding torque, not observer estimates.

To plot torques from results already in your workspace without rerunning:

```matlab
plot_motor_torques(linearResults,'linear')
% Or: plot_motor_torques(nonlinearResults,'nonlinear')
```

The nonlinear limits are +/-8 degrees per joint correction, followed by
absolute joint-command limits, and +/-0.60 N m per servo torque target.
The torque target is clipped before the 0.03 s lag, so delivered torque
approaches the cap rather than jumping to it. Each new result contains a
`clipping` field with the first time each limiter alters its requested signal;
NaN means that clipping was not detected in that run. These are simulation
sample-based checks, not a guarantee about unrecorded instants.
The original linear model bypasses these limiters: its dashed torque-cap
lines are references, not enforced saturation.

1. Original forward push; no outer controller, inner servos hold standing targets.
2. Original forward push; original state feedback and observer.
3. Same as case 2, plus a small constant IMU/encoder measurement bias.
4. Same as case 3, plus a small sinusoidal IMU/encoder measurement error.

The plant, gains, observer, parameters and SLX files are copied unchanged. Only temporary in-memory models are modified by the new script. Sensor disturbances do not change the actual external push or the internal servo dynamics. Edit their amplitudes at the top of `run_four_cases.m`.

The original push is retained exactly: a **1 N forward Gaussian force**, centred at 0.2 s with width 0.02 s. N m specifies torque rather than that original force. The original observer initial condition is also retained.

For the copied linear model instead, run `results=run_four_cases('linear');`.

To display how the existing controller moves the unstable plant poles into
stable locations, run **`show_pole_movement`**. This opens a pole-movement
figure, shows the actual digital poles against the unit circle, and prints
the controllability checks. It does not change or retune the controller.
The continuous-plane closed-loop poles are equivalent values `log(z)/Ts`,
because this controller is implemented digitally. Stabilising the plant is
different from making it controllable: the balance plant is already controllable.

If the same original model name is open from another folder, save and close that model first. The new script performs no controller-performance analysis. The copied original README and scripts remain available as references; **RUN_ALL** is the entry point for the new four cases.
