# MicroDuck: lecture-based state feedback and observer

**6 November parameter update:** `sf_parameters.m` and both saved plants now
use source-verified total mass **0.7443611 kg** and joint damping **0.053 Nm s/rad**.
The historical result numbers and figures below were produced before this change.
Use **Report_Updated_Plant/START_HERE_REPORT.m** for the new constant/mixed-load
controller, newly rerun data, figures and Overleaf package. That report variant
adds a constant-pitch-moment observer state and equilibrium feedforward.

This folder implements pole-placement state feedback and a Luenberger observer,
using Lectures 9, 10 and 11. The previous LQR/covariance-based gain design has
been replaced. The plant equations, physical settings and three-case experiment
architecture are retained.

Run START_HERE.m, or change MATLAB's Current Folder to this folder and run:

```matlab
results=run_simulations;
```

## Connection to the course

Page numbers below count PDF pages, including the title page.

| Course material | Implementation |
|---|---|
| Lecture 9, PDF pages 6-7 and 19-25: state feedback, pole placement and coordinate transformations | Choose desired poles and calculate K using `place`; apply `du=-K*z` |
| Lecture 10, PDF pages 18-22 and 28-30: Luenberger observer and duality | Estimate the states from sensor innovation; calculate L using pole placement on the dual system |
| Lecture 10, PDF pages 40-41: separation principle and observer speed | Design K and L separately and verify both sets of poles |
| Lecture 11, PDF pages 43-45: actuator saturation and observer input | Send the same limited joint target to the plant and observer predictor |
| Lecture 11, PDF pages 46-49: computer implementation | Sample measurements, update the observer state, and hold commands at Ts=0.02 s |

The source PDFs are in the ELEC3304/Lectures folder:
`ELEC3304___Lecture_9-1139ab64-4920-4740-8195-7160b10c0222 (1).pdf`,
`ELEC3304___Lecture_10-4 (1).pdf`, and `ELEC_3304___Lecture_11-7.pdf`.

The slides mainly introduce continuous-time equations. This implementation uses
their sampled-time counterpart, with exact zero-order-hold plant discretization.
A real Schur coordinate transformation is a numerical way to separate slow
modes from already-fast stable modes; it does not remove any physical states.
This numerical step and the selected pole values are project design choices,
not additional requirements stated in the lectures.

Integral action, disturbance-state augmentation and internal-model extensions
also appear in the course. They are not added to this standing push-rejection
experiment. The current target is zero deviation from standing; persistent
loads or a reference-tracking experiment would require further design.

## The controller

The plant retains the same 36 states: 14 coordinates, 14 coordinate rates and
8 servo torques. The controller and observer use 33 retained balance states,
omitting absolute world X, Y and yaw. Both true and estimated retained states
are absolute physical quantities. The native controller blocks calculate:

```
z = S^(-1) (x_retained - xeq_retained)
du = -K*z
u = ueq + du
```

With the observer, substitute `xhat_retained` for `x_retained`. K is an 8-by-33
matrix and u contains eight desired joint angles. The nonlinear controller
limits joint offsets and absolute commands; the linear baseline bypasses these
limits, as in the earlier plant comparison. The inner servo dynamics and torque
limits remain part of the nonlinear plant.

### How K is chosen

The three slowest/unstable modes are selected using a 2.5/s decay-rate cutoff.
A coordinate transformation exposes this slow subsystem. `place` assigns its
poles, then transforms the feedback gain back to the original scaled states.
The other 30, already-stable modes retain their natural poles. Keeping those
fast contact/servo dynamics avoids forcing numerically tiny sampled poles to
new arbitrary positions.

Default assigned continuous-equivalent decay rates are 6, 1 and 1.2 /s, giving
these discrete poles at Ts=0.02 s:

```
exp(-6*Ts)   = 0.88692044
exp(-1*Ts)   = 0.98019867
exp(-1.2*Ts) = 0.97628571
```

The 6/s assignment stabilizes the unstable mode. The two weaker slow modes get
more conservative assignments to avoid excessive commands. These are starting
design values; they do not guarantee any particular pitch/roll recovery time.
In particular, the slowest internal pole need not dominate the plotted angles.
The maximum closed-loop pole magnitude is about 0.980199. The code verifies the
achieved poles against the requested set and checks stability. All 33 poles
remain part of the resulting model.

```matlab
addpath helpers
r=md_sf_prepare(sf_parameters);
r.feedback.K
r.feedback.assignedSlowPoles
r.feedback.requestedPoles
r.feedback.poles
r.feedback.poleMatchError
```

## The observer

The new native-block observer is the sampled Luenberger predictor:

```
yn[k] = diag(1./yScale) (y[k]-yeq)
innovation[k] = yn[k] - Cn*zhat[k]
zhat[k+1] = Ad*zhat[k] + Bd*(u[k]-ueq) + L*innovation[k]
xhat_retained[k] = xeq_retained + S*zhat[k]
```

A Unit Delay stores zhat[k]. The current controller uses that stored estimate;
current measurements and the applied input produce the next estimate. This
causal implementation has no instantaneous measurement-correction stage or
Kalman covariance tuning. Its error poles are `eig(Ad-L*Cn)`.

Nine slow observer poles are assigned continuous-equivalent rates 20 through
28 /s, or `exp(-(20:28)*Ts)` in the z-plane. The other 24 fast stable poles are
retained. The largest observer-pole magnitude is about 0.670320, faster than
the controller's slow poles, as discussed in Lecture 10. `place` is applied to
the transposed observer subsystem to obtain L by duality.

The observer sees IMU/encoder measurements and the actual limited joint-angle
command. It does not see the external push or actual states. The ideal linear
combined state/error system has the controller and observer pole sets together
(separation principle). This local linear result does not guarantee nonlinear
recovery under large pushes, changing contact or clipping. No measurement noise
is injected in the simulations.

## Three cases on each plant

1. Fixed standing targets: no outer balance controller; inner servos hold ueq.
2. State feedback with true states: the same K receives exact simulated balance
   states sampled at 50 Hz. This is an ideal baseline.
3. State feedback with observer: the same K receives estimated balance states.

All curves show actual physical roll/pitch, including the observer-driven curve.
The observer starts with the same deliberate 0.5-degree roll / 1-degree pitch
estimation error as the original experiment. This can cause motion before the
push. For a push-only comparison:

```matlab
results=run_simulations(struct('zeroObserverInitialError',true));
```

Both plants start from the same true standing equilibrium and receive the same
1 N forward Gaussian push, centred at 0.2 s with width 0.02 s. A forward push
mainly excites pitch; set `cfg.test.pushForce=[0;1;0]` for a lateral roll test.

## Settings, outputs and limitations

Edit sf_parameters.m and save. Both models regenerate trim, linear matrices,
K and L from that one configuration on load and before Run. The runner checks
identical design data and all six recorded push histories. Independent edits to
bound workspace aliases are overwritten. Changes to physical parameters can
change which modes lie below the selected cutoff; if so, the code asks you to
revise the pole list rather than silently assigning a different design.

The continuous plant equations match the PID folder. The linear model uses
constant local matrices; the nonlinear model retains changing geometry,
contact and actuator limits. Source diagrams can be edited, but the comparison
runner uses temporary copies and leaves the saved SLX files unchanged. Each
comparison stops at the same tilt/height validity limits. A truncated run is
labelled and does not count as recovery. Pressing Run directly in a source SLX
does not add the runner's validity guard.

```matlab
results.linear.metrics
results.nonlinear.metrics
results.nonlinear.cases.openLoop
results.nonlinear.cases.stateTrue
results.nonlinear.cases.stateObserver
results.sharedConfiguration
results.comparisonCheck
```

The `.state`, `.estimate` and `.output` fields at plant level refer to the
observer-driven case. Estimated states are 33 absolute retained states.
`output.balanceCorrection` contains eight joint offsets. Figures include three
actual responses plus target, force/torque panels, angle-detail views and a
linear-versus-nonlinear comparison. `figures/performance_metrics.csv` contains
all six cases. A false linear clipping flag means limits are bypassed; it does
not prove the requested actuator effort is physically attainable.

Recovery uses a +/-0.1-degree band after the practical pulse end
`centre+3*width` (0.26 s), with at least 0.5 s confirmation dwell. Options follow
the earlier runner:

```matlab
results=run_simulations(struct('angleBandDeg',0.1, ...
    'recoveryGoalSec',1,'minDwellSec',0.5));
```

## Files

The checked default nonlinear experiment completed all controlled runs without
offset, absolute-command or target-torque clipping. Pitch recovery after 0.26 s
was approximately 0.658 s with true-state feedback and 0.770 s with the observer.
Peak pitch deviations were 0.921 and 1.313 degrees respectively. Both fixed-target
baselines reached the validity boundary around 1.4 s. These observations apply
to the default forward pulse and estimator initial error only. The feedback
equation and observer recurrence were also checked against the recorded signals
to numerical precision, and plant A/B matrices and trim match the PID model.

| File | Purpose |
|---|---|
| START_HERE.m | Set paths and open the annotated nonlinear model |
| sf_parameters.m | Shared physical/test settings and requested pole choices |
| microduck_linear_state_feedback.slx | Linear plant, pole-placement controller and Luenberger observer |
| microduck_nonlinear_state_feedback.slx | Same controller/observer on the nonlinear plant |
| run_simulations.m | Six matched cases, metrics, validity stops and plots |
| helpers/md_sf_prepare.m | Trim, linearize, discretize and calculate K/L by pole placement |
| helpers/md_sf_apply.m | Refresh workspace data and block bindings |
| helpers/md_sf_plot_comparison.m | Plot each plant's three actual responses |
| helpers/md_sf_plot_cross.m | Direct linear/nonlinear comparison |
| helpers/md_calculation_kernel.m | Stateless nonlinear geometry operations |
| helpers/md_rigid_body_math.m | Same rigid-body equations as the PID plant |
| helpers/md_expanded_cache.m | Short temporary build paths for Windows |
| figures/ | Current diagrams, results and metric CSV |

Requires MATLAB, Simulink, Control System Toolbox and MATLAB Function block
support. Tested with R2026b. Keep helpers beside the models and change MATLAB's
Current Folder here before calling run_simulations; the PID folder has a
different function of that name. Nonlinear contact simulations can take several
minutes. Earlier LQR results/pole lists do not describe this revised controller.

## Batch push figures
Run `test_files` from this folder to simulate forward and sideways pushes of
1, 5 and 10 N on both the linear and nonlinear plants. Each run includes all
four controller/sensor cases. Force uses N; applied moments use N m.

PNG and editable FIG plots are saved under `figures_new/forward_1N`,
`forward_5N`, `forward_10N`, `sideways_1N`, `sideways_5N`, and `sideways_10N`.
Each contains `linear` and `nonlinear` subfolders with response and motor-torque
plots, including zoom views. The common pole plot is saved once in
`figures_new/pole_placement`. Each push folder also saves its test configuration.

The batch uses in-memory push overrides and does not edit sf_parameters.m.
Figures close after saving to avoid opening dozens of windows. Set
`keepFiguresOpen=true` in test_files.m to keep them open instead.

### Controller performance CSVs
`test_files` writes `controller_performance.csv` into each plant/test folder,
plus `figures_new/controller_performance_all.csv` covering all runs. The
workspace table is `allPerformance`. Both roll and pitch are recorded for
all four controller/sensor cases.

These are pulse-disturbance recovery tests, not reference-step tests.
`ExcursionRiseTime_s` measures 10%-90% of the largest signed tilt excursion.
`PeakTime_s` measures time to the largest absolute tilt. `SettlingTime_s`
uses +/-2% of that peak about upright, after the peak, with at least 0.5 s
of remaining in-band observations and a completed run. Peak and settling
times start at max(0, pushCentre - 3*pushWidth), the Gaussian push-window
start. `ReverseOvershoot_pct` measures the subsequent opposite-side peak
relative to the largest excursion, not standard step overshoot.

Standard `StepRiseTime_s` and `StepOvershoot_pct` are NaN because there is
no nonzero reference step. Missing/unobserved settling is also NaN. Check
`RunCompleted`, `SettledObserved`, `SimulationEnd_s`, and `FinalTilt_deg`;
a finite observation window does not prove asymptotic settling, especially
with sinusoidal sensor errors. Metrics use actual plant tilt, not noisy readings.
