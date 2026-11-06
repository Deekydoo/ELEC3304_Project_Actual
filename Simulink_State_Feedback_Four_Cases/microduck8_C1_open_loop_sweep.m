%% MicroDuck: current-setup 81-point OPEN-LOOP eigenvalue sweep
% Run: microduck8_C1_open_loop_sweep
% Uses sf_parameters.m and helpers/md_sf_prepare.m, not microduck8_plant.m.
% Preserves the uploaded parameter grid and separate Kd sensitivity check.
% Historical results are removed: all values are calculated from this plant.
% This does NOT establish C1 closed-loop robustness. There are 81 grid points,
% not 81 geometric corners. No models, parameters or gains are changed.
scriptFolder=fileparts(mfilename('fullpath'));
cd(scriptFolder); addpath(scriptFolder,fullfile(scriptFolder,'helpers'));
nominalConfiguration=sf_parameters;
massScale=[0.95 1.00 1.05];
comValues=nominalConfiguration.robot.torsoCOM(3)+[-0.008 0 0.008];
kpValues=[0.386 0.52 0.55];          % N m/rad
bjValues=[0.041 0.048 0.053];        % N m s/rad, PAIRED with kpValues
TaValues=[0.02 0.03 0.08];           % s
kdValues=[0 0.0265 0.053];           % separate nominal sensitivity test
assert(numel(kpValues)==numel(bjValues),'Kp and bj must be paired.');
tol=1e-6;                           % positive real-part threshold [1/s]
Tsel=setdiff(1:36,[1 2 4],'stable'); % remove world X, Y and yaw
massFields={'massTorso','massCarrier','massThigh','massShin','massFoot'};
N=numel(massScale)*numel(comValues)*numel(kpValues)*numel(TaValues);
results=nan(N,9);                   % fixed: nine values are written per row
failed=false(N,1); failureMessage=strings(N,1); allPoles=cell(N,1);
nominalPoles=current_balance_poles(nominalConfiguration,Tsel);
[nominalCount,nominalMaxReal,~]=summarise_poles(nominalPoles,tol);
nominalMass=nominalConfiguration.robot.massTorso+2*(nominalConfiguration.robot.massCarrier+ ...
    nominalConfiguration.robot.massThigh+nominalConfiguration.robot.massShin+nominalConfiguration.robot.massFoot);
fprintf('Current nominal mass %.9g kg; %d unstable poles; max real part %+.7g /s\n',nominalMass,nominalCount,nominalMaxReal);
fprintf('Running %d open-loop grid points...\n',N);
row=0;
for im=1:numel(massScale)
 for ic=1:numel(comValues)
  for ik=1:numel(kpValues)
   for it=1:numel(TaValues)
    row=row+1; cfg=nominalConfiguration;
    for f=1:numel(massFields)
     cfg.robot.(massFields{f})=nominalConfiguration.robot.(massFields{f})*massScale(im);
    end
    cfg.robot.torsoCOM(3)=comValues(ic);
    cfg.servo.kp=kpValues(ik)*ones(8,1);
    cfg.servo.jointDamping=bjValues(ik)*ones(8,1);
    cfg.servo.timeConstant=TaValues(it)*ones(8,1);
    results(row,1:5)=[massScale(im),comValues(ic),kpValues(ik),bjValues(ik),TaValues(it)];
    try
     poles=current_balance_poles(cfg,Tsel); allPoles{row}=poles;
     [count,maxReal,dominant]=summarise_poles(poles,tol);
     results(row,6:9)=[count,maxReal,real(dominant),imag(dominant)];
    catch ME
     failed(row)=true; failureMessage(row)=string(ME.message);
     fprintf('Grid point %d FAILED: %s\n',row,ME.message);
    end
   end
  end
 end
end
T=array2table(results,'VariableNames',{'MassScale','COMHeight_m','Kp','bj','Ta_s', ...
 'UnstablePoleCount','MaxRealPole','DominantPoleReal','DominantPoleImag'});
T.TotalMass_kg=T.MassScale*nominalMass; T.Failed=failed; T.FailureMessage=failureMessage;
disp(T);
valid=~failed;
fprintf('\n========== OPEN-LOOP SWEEP: 33 BALANCE STATES ==========\n');
fprintf('Grid points %d; successful %d; failed %d\n',N,sum(valid),sum(failed));
if any(valid)
 counts=T.UnstablePoleCount(valid); maxReal=T.MaxRealPole(valid);
 fprintf('Unstable pole count range: %d to %d\n',min(counts),max(counts));
 fprintf('Maximum-real-part range: %+.7g to %+.7g /s\n',min(maxReal),max(maxReal));
 fprintf('All successful points have exactly one unstable pole: %s\n',string(all(counts==1)));
 % Check ALL nominal servo entries, not just their first element.
 nominalRows=valid & abs(T.MassScale-1)<1e-10 & ...
  abs(T.COMHeight_m-nominalConfiguration.robot.torsoCOM(3))<1e-10;
 for i=1:height(T)
  nominalRows(i)=nominalRows(i) && all(abs(nominalConfiguration.servo.kp(:)-T.Kp(i))<1e-10) && ...
   all(abs(nominalConfiguration.servo.jointDamping(:)-T.bj(i))<1e-10) && ...
   all(abs(nominalConfiguration.servo.timeConstant(:)-T.Ta_s(i))<1e-10);
 end
 if any(nominalRows), fprintf('\nNominal grid point:\n'); disp(T(nominalRows,:));
 else, fprintf('Current nominal is not an exact grid point; it was calculated separately above.\n'); end
 validRows=find(valid); [~,i]=max(maxReal);
 fprintf('Grid point with largest pole real part:\n'); disp(T(validRows(i),:));
end

%% One-at-a-time Kd check, with all other CURRENT nominal settings retained
kdData=nan(numel(kdValues),3); kdFailed=false(numel(kdValues),1);
kdFailureMessage=strings(numel(kdValues),1); kdPoles=cell(numel(kdValues),1);
fprintf('\nKd sensitivity:\n');
for i=1:numel(kdValues)
 cfg=nominalConfiguration; cfg.servo.kd=kdValues(i)*ones(8,1); kdData(i,1)=kdValues(i);
 try
  pk=current_balance_poles(cfg,Tsel); kdPoles{i}=pk;
  [count,maxReal,~]=summarise_poles(pk,tol); kdData(i,2:3)=[count,maxReal];
  fprintf('Kd=%.4f: %d unstable; max real part %+.7g /s\n',kdValues(i),count,maxReal);
 catch ME
  kdFailed(i)=true; kdFailureMessage(i)=string(ME.message);
  fprintf('Kd=%.4f FAILED: %s\n',kdValues(i),ME.message);
 end
end
KdSensitivity=array2table(kdData,'VariableNames',{'Kd','UnstablePoleCount','MaxRealPole'});
KdSensitivity.Failed=kdFailed; KdSensitivity.FailureMessage=kdFailureMessage;
outputFolder=fullfile(scriptFolder,'C1_sweep_results');
if ~exist(outputFolder,'dir'), mkdir(outputFolder); end
writetable(T,fullfile(outputFolder,'microduck8_C1_sweep_results.csv'));
writetable(KdSensitivity,fullfile(outputFolder,'microduck8_C1_kd_sensitivity.csv'));
save(fullfile(outputFolder,'microduck8_C1_sweep_results.mat'), ...
 'T','KdSensitivity','nominalConfiguration','nominalPoles','allPoles','kdPoles', ...
 'massScale','comValues','kpValues','bjValues','TaValues','kdValues','tol','Tsel');
if any(failed), fprintf('\nFailed grid configurations:\n'); disp(T(failed,:)); end
fprintf('Outputs saved in: %s\n',outputFolder);
fprintf('This is an open-loop sweep, not a closed-loop C1 compliance test.\n');

function poles=current_balance_poles(cfg,keep)
% Physical matrices only: do not redesign gains at every open-loop grid point.
cfg.reportPlantOnly=true;
plant=md_sf_prepare(cfg);
assert(isequal(size(plant.lin.A),[36 36]),'Expected the current 36-state plant.');
poles=eig(plant.lin.A(keep,keep));
assert(numel(poles)==33 && all(isfinite(poles)),'Invalid balance-plant poles.');
end
function [count,maxReal,dominant]=summarise_poles(poles,tolerance)
unstable=real(poles)>tolerance; count=sum(unstable); maxReal=max(real(poles));
dominant=complex(NaN,NaN);
if count>0, candidates=poles(unstable); [~,i]=max(real(candidates)); dominant=candidates(i); end
end
