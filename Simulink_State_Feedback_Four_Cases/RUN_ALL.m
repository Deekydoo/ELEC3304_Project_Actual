% Run only the four requested cases and open the response plots.
root=fileparts(mfilename('fullpath'));
cd(root); addpath(root,fullfile(root,'helpers'));
results=run_four_cases;
