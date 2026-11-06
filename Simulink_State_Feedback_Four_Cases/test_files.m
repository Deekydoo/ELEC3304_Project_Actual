%% Overall Roll and Pitch

linearResults = run_four_cases('nonlinear');

linearResults = run_four_cases('linear');

%% Plotting Motor Torques


plot_motor_torques(linearResults,'linear')

plot_motor_torques(linearResults,'nonlinear')

%% Pole Placement

show_pole_movement.m