%% Observer-based state feedback: MicroDuck
% Lecture 9-11 pole placement: state feedback + Luenberger observer. Edit sf_parameters.m.
root=fileparts(mfilename('fullpath'));
cd(root); addpath(root,fullfile(root,'helpers')); md_expanded_cache;
open_system(fullfile(root,'microduck_nonlinear_state_feedback.slx'));
% Linear model:
% open_system(fullfile(root,'microduck_linear_state_feedback.slx'));
% Six matched experiments and plots:
% results=run_simulations;
% Push-only test without initial observer error:
% results=run_simulations(struct('zeroObserverInitialError',true));
