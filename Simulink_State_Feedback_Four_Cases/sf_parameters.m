function cfg=sf_parameters
% SINGLE SOURCE OF TRUTH for both Simulink plants and comparison cases.
% Edit here, save, then run run_simulations or press Run in either model.
% Physical changes regenerate trim, linear matrices and observer gains.
% Pole-placement gains K and L are designed once and used on both plants.

% Experiment: force in N; moment in Nm; angles in degrees where named Deg.
cfg.test.duration=3;
cfg.test.pushForce=[0;10;0];
cfg.test.pushMoment=[0;0;0];
cfg.test.pushCentre=0.2;
cfg.test.pushWidth=0.02;
cfg.sampleTime=0.02;
% Standing regulation: target is zero deviation in roll and pitch.
% Both true plants start at the same recomputed standing equilibrium.
% Optional startup estimator error, [roll; pitch] degrees. Set [0;0] for push-only tests.
cfg.observer.initialAngleErrorDeg=[0.5;1];

% Robot geometry, masses and gravity. All lengths in m, masses in kg.
cfg.robot.gravity=[0;0;-9.81];
cfg.robot.hipHalfSpacing=0.04;
cfg.robot.hipPitchDrop=0.01;
cfg.robot.thighLength=0.05;
cfg.robot.shinLength=0.05;
cfg.robot.thighWidth=0.025;
cfg.robot.shinWidth=0.022;
cfg.robot.footLength=0.054;
cfg.robot.footWidth=0.041;
cfg.robot.footThickness=0.012;
cfg.robot.soleDepth=0.018;
cfg.robot.torsoDimensions=[0.09;0.1;0.14];
cfg.robot.imuOffset=[0;0;0.05];
cfg.robot.torsoCOM=[0;0;0.055];
cfg.robot.carrierDimensions=[0.025;0.025;0.02];
% Configuration 1 masses supplied in the mass-comparison image (kg).
% One torso and two of each leg component give a total of 0.737243 kg.
% Geometry and box inertias remain simplified; inertias are recomputed.
cfg.robot.massTorso=0.479151;
cfg.robot.massCarrier=0.029230;
cfg.robot.massThigh=0.048207;
cfg.robot.massShin=0.021584;
cfg.robot.massFoot=0.030025;
cfg.robot.standingJointDeg=[0;-20;40;-20;0;-20;40;-20];

% Internal position servo model: SI gains, lag, damping and command limits.
cfg.servo.kp=[0.55;0.55;0.55;0.55;0.55;0.55;0.55;0.55];
cfg.servo.kd=[0;0;0;0;0;0;0;0];
cfg.servo.timeConstant=[0.03;0.03;0.03;0.03;0.03;0.03;0.03;0.03];
cfg.servo.maxTorque=[0.6;0.6;0.6;0.6;0.6;0.6;0.6;0.6];
% Earlier draft nominal equivalent joint damping (not a measured hardware fit).
cfg.servo.jointDamping=0.053*ones(8,1);
cfg.servo.minCommandDeg=[-20.05352282957881;-57.295779513082323;2.8647889756541161;-45.836623610465857;-20.05352282957881;-57.295779513082323;2.8647889756541161;-45.836623610465857];
cfg.servo.maxCommandDeg=[20.05352282957881;34.377467707849391;85.943669269623484;45.836623610465857;20.05352282957881;34.377467707849391;85.943669269623484;45.836623610465857];

% Sole-point contact model.
cfg.contact.stiffness=4000;
cfg.contact.damping=12;
cfg.contact.friction=0.6;
cfg.contact.frictionVelocity=0.005;

% Lecture 9: pole placement. Move the three slowest real open-loop modes.
% The unstable mode gets decay rate 6/s; weak slow modes get 1 and 1.2/s.
% Other stable plant modes are retained. These are editable design choices.
cfg.feedback.assignedDecayRates=[6;1;1.2];
cfg.feedback.slowModeCutoffRate=2.5;
cfg.feedback.poleMatchTolerance=1e-6; % absolute error in discrete z-plane
cfg.feedback.maxCorrectionDeg=8; % each joint offset, nonlinear only

% Lecture 10: Luenberger observer pole placement, no noise covariance tuning.
% Place slow observer modes faster; retain the already-fast stable modes.
cfg.observer.slowModeCutoffRate=15;
cfg.observer.firstDecayRate=20;
cfg.observer.decayRateSpacing=1;

% Numerical solver and common physical validity bounds.
cfg.solver.maxStep=0.002;
cfg.solver.relTol=1e-6;
cfg.solver.absTol=1e-08;
cfg.validity.maxTiltDeg=20;
cfg.validity.minHipHeight=0.06;
end


