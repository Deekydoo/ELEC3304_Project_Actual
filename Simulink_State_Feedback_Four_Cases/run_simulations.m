function results=run_simulations(options)
% Three actual responses + target: fixed targets, true-state feedback, observer state feedback.
% results=run_simulations;
% results=run_simulations(struct('zeroObserverInitialError',true));
% Both plants are refreshed from sf_parameters.m before any case is copied.
% Bound settings override local aliases; saved SLX files are not changed.
if nargin<1, options=struct; end
defaults=struct('angleBandDeg',0.1,'recoveryGoalSec',1,'minDwellSec',0.5, ...
    'zeroObserverInitialError',false,'exportFigures',true);
for f=fieldnames(options)'
    assert(isfield(defaults,f{1}),['Unknown option: ' f{1}]); defaults.(f{1})=options.(f{1});
end
options=defaults;
validateattributes(options.angleBandDeg,{'numeric'},{'scalar','positive','finite'});
validateattributes(options.minDwellSec,{'numeric'},{'scalar','positive','finite'});
root=fileparts(mfilename('fullpath')); addpath(root,fullfile(root,'helpers')); md_expanded_cache;
shared=sf_parameters;
for modes={'linear','nonlinear'}
    source=['microduck_' modes{1} '_state_feedback'];
    load_system(fullfile(root,[source '.slx'])); md_sf_apply(source,shared);
end
left=getVariable(get_param('microduck_linear_state_feedback','ModelWorkspace'),'r');
right=getVariable(get_param('microduck_nonlinear_state_feedback','ModelWorkspace'),'r');
assert(isequaln(rmfield(left,'mode'),rmfield(right,'mode')), ...
    'The two experiments do not share identical physical/design settings.');
fprintf('Matched experiment: peak force [%g %g %g] N, centre %.4g s, width %.4g s, sample period %.4g s.\n', ...
    shared.test.pushForce,shared.test.pushCentre,shared.test.pushWidth,shared.sampleTime);
results=struct; keys={'openLoop','stateTrue','stateObserver'};
labels={'Fixed standing targets (no outer balance controller)','State feedback with true states','State feedback with observer'};
for modes={'linear','nonlinear'}
    mode=modes{1}; source=['microduck_' mode '_state_feedback'];
    load_system(fullfile(root,[source '.slx'])); dirty=get_param(source,'Dirty'); cases=struct;
    for j=1:3
        clone=copy_case(source); cleanup=onCleanup(@()close_case(clone));
        workspace=get_param(clone,'ModelWorkspace'); r=getVariable(workspace,'r');
        if options.zeroObserverInitialError
            r.observer.settings.initialEstimateDelta=zeros(36,1);
        end
        if j==1
            add_block('simulink/Sources/Constant',[clone '/Fixed standing targets'],'Value','r.p.ueq','Position',[80 170 220 205]);
            for dest={'Robot plant/1','State observer 50 Hz/2','Send jointCommand/1'}
                replace_input(clone,dest{1},'Fixed standing targets/1');
            end
            add_block('simulink/Sources/Constant',[clone '/No balance correction'],'Value','zeros(8,1)','Position',[80 220 220 255]);
            replace_input(clone,'Send balanceCorrection/1','No balance correction/1');
        elseif j==2
            H=zeros(33,36); H(:,r.observer.keep)=eye(33); assignin(workspace,'Htrue',H);
            add_block('simulink/Math Operations/Gain',[clone '/Retain true balance states'],'Gain','Htrue','Multiplication','Matrix(K*u)','Position',[40 120 180 160]);
            add_block('simulink/Discrete/Zero-Order Hold',[clone '/True feedback sample'],'SampleTime','r.observer.sampleTime','Position',[300 130 430 175]);
            add_line(clone,'Robot plant/1','Retain true balance states/1','autorouting','on');
            add_line(clone,'Retain true balance states/1','True feedback sample/1');
            replace_input(clone,'State feedback controller/1','True feedback sample/1');
        end
        assignin(workspace,'r',r); add_validity_stop(clone);
        if strcmp(mode,'nonlinear'), log_limit_pairs(clone); end
        output=sim(clone); s=collect(output,r,labels{j},j,mode,options);
        cases.(keys{j})=s;
        fprintf('%s / %s: %.4g s, %s\n',mode,keys{j},s.time(end),s.status); clear cleanup;
    end
    assert(strcmp(get_param(source,'Dirty'),dirty),'Source model was modified.');
    bundle=cases.stateObserver; bundle.cases=cases; bundle.criteria=options;
    bundle.metrics=metrics_table(cases,keys); results.(mode)=bundle;
    md_sf_plot_comparison(cases,mode,root,options);
    disp(bundle.metrics(:,{'Case','CompletedValid','PeakPitch_deg','RecoveryPitch_s','PeakTorque_Nm','CorrectionClipped','TargetTorqueClipped'}));
    if ~cases.stateObserver.complete
        warning('MicroDuck:Validity','%s observer-controlled case reached a validity bound.',mode);
    end
end
results.sharedConfiguration=shared;
clock=(0:shared.sampleTime:min(results.linear.time(end),results.nonlinear.time(end)))';
yl=results.linear.output.worldPush; yn=results.nonlinear.output.worldPush;
mismatch=max(abs(interp1(yl.Time,rows(yl),clock)-interp1(yn.Time,rows(yn),clock)),[],'all');
assert(mismatch<1e-7,'The recorded linear/nonlinear disturbances do not match.');
% Verify all six logged pushes, including early-stopped fixed-target cases.
reference=results.linear.cases.stateObserver.output.worldPush;
for modes={'linear','nonlinear'}
    for k=1:numel(keys)
        measured=results.(modes{1}).cases.(keys{k}).output.worldPush;
        checkTime=(0:shared.sampleTime:min(reference.Time(end),measured.Time(end)))';
        difference=max(abs(interp1(reference.Time,rows(reference),checkTime)- ...
            interp1(measured.Time,rows(measured),checkTime)),[],'all');
        mismatch=max(mismatch,difference);
    end
end
assert(mismatch<1e-7,'One of the six experiments received a different disturbance.');
results.comparisonCheck=struct('sharedParameters',true,'maxWrenchDifference',mismatch);
md_sf_plot_cross(results,root,options);
if options.exportFigures
    linear=results.linear.metrics; nonlinear=results.nonlinear.metrics;
    linear.Plant=repmat({'linear'},height(linear),1);
    nonlinear.Plant=repmat({'nonlinear'},height(nonlinear),1);
    writetable([linear;nonlinear],fullfile(root,'figures','performance_metrics.csv'));
end
end

function clone=copy_case(source)
% Copy through an intermediate subsystem; do not rename or save the source.
[~,token]=fileparts(tempname); token=token(max(1,end-7):end);
container=['md8_box_' token]; clone=['md8_case_' token];
new_system(container); cleanup=onCleanup(@()close_case(container));
add_block('built-in/Subsystem',[container '/Contents']);
Simulink.BlockDiagram.copyContentsToSubSystem(source,[container '/Contents']); new_system(clone);
try
    Simulink.SubSystem.copyContentsToBlockDiagram([container '/Contents'],clone);
    original=get_param(source,'ModelWorkspace'); copied=get_param(clone,'ModelWorkspace'); variables=whos(original);
    for k=1:numel(variables), assignin(copied,variables(k).name,getVariable(original,variables(k).name)); end
    config=copy(getActiveConfigSet(source)); config.Name='Comparison configuration';
    attachConfigSet(clone,config); setActiveConfigSet(clone,config.Name);
    set_param(clone,'PreLoadFcn','','PostLoadFcn','','InitFcn','','StartFcn','','StopFcn','', ...
        'ReturnWorkspaceOutputs','on','SimulationMode','normal');
catch err
    close_case(clone); rethrow(err);
end
clear cleanup;
end
function close_case(model)
if bdIsLoaded(model), close_system(model,0); end
end
function replace_input(model,destination,newSource)
parts=strsplit(destination,'/'); port=str2double(parts{end});
ph=get_param([model '/' strjoin(parts(1:end-1),'/')],'PortHandles'); line=get_param(ph.Inport(port),'Line');
if line>=0
    src=get_param(line,'SrcPortHandle');
    while src<0, line=get_param(line,'LineParent'); src=get_param(line,'SrcPortHandle'); end
    delete_line(model,src,ph.Inport(port));
end
add_line(model,newSource,destination,'autorouting','on');
end
function add_validity_stop(model)
% Terminate unsupported fall trajectories, including the uncontrolled baseline.
H=zeros(3,36); H(1,5)=1; H(2,6)=1; H(3,3)=1;
assignin(get_param(model,'ModelWorkspace'),'Hvalid',H);
add_block('simulink/Math Operations/Gain',[model '/Validity coordinates'],'Gain','Hvalid','Multiplication','Matrix(K*u)','Position',[40 780 200 820]);
add_block('simulink/Signal Routing/Demux',[model '/Pitch roll height'],'Outputs','3','Position',[240 760 245 880]);
add_line(model,'Robot plant/1','Validity coordinates/1','autorouting','on'); add_line(model,'Validity coordinates/1','Pitch roll height/1');
add_block('simulink/Logic and Bit Operations/Logical Operator',[model '/Any invalid coordinate'],'Operator','OR','Inputs','3','Position',[750 780 800 870]);
for j=1:3
    cmp=['Bound ' num2str(j)]; cst=['Limit ' num2str(j)];
    if j<3
        ab=['Absolute tilt ' num2str(j)];
        add_block('simulink/Math Operations/Abs',[model '/' ab],'Position',[300 700+70*j 420 735+70*j]);
        add_line(model,['Pitch roll height/' num2str(j)],[ab '/1']); input=[ab '/1']; op='>='; bound='r.p.cfg.stop.maxTiltDeg*pi/180';
    else
        input='Pitch roll height/3'; op='<='; bound='r.p.cfg.stop.minHipHeight';
    end
    add_block('simulink/Logic and Bit Operations/Relational Operator',[model '/' cmp],'Operator',op,'Position',[570 700+70*j 620 745+70*j]);
    add_block('simulink/Sources/Constant',[model '/' cst],'Value',bound,'Position',[300 1000+70*j 460 1035+70*j]);
    add_line(model,input,[cmp '/1']); add_line(model,[cst '/1'],[cmp '/2']); add_line(model,[cmp '/1'],['Any invalid coordinate/' num2str(j)]);
end
add_block('simulink/Sinks/Stop Simulation',[model '/Stop at validity bound'],'Position',[860 800 1020 840]);
add_line(model,'Any invalid coordinate/1','Stop at validity bound/1');
end
function log_limit_pairs(model)
all=find_system(model,'LookUnderMasks','all','BlockType','Saturate');
names={'Limit state feedback corrections','Joint command limits','Motor target torque limits'}; vars={'balanceLimit','commandLimit','torqueLimit'};
for j=1:3
    chosen='';
    for k=1:numel(all)
        if strcmp(strrep(get_param(all{k},'Name'),newline,' '),names{j}), chosen=all{k}; break; end
    end
    assert(~isempty(chosen),['Missing limit block: ' names{j}]);
    parent=get_param(chosen,'Parent'); ph=get_param(chosen,'PortHandles'); line=get_param(ph.Inport,'Line'); src=get_param(line,'SrcPortHandle');
    while src<0, line=get_param(line,'LineParent'); src=get_param(line,'SrcPortHandle'); end
    mux=add_block('simulink/Signal Routing/Mux',[parent '/Comparison pair ' vars{j}],'Inputs','2','Position',[1700 850 1705 930]); mp=get_param(mux,'PortHandles');
    log=add_block('simulink/Sinks/To Workspace',[parent '/Comparison record ' vars{j}],'VariableName',vars{j},'SaveFormat','Timeseries','Position',[1780 860 1950 900]); lp=get_param(log,'PortHandles');
    add_line(parent,src,mp.Inport(1),'autorouting','on'); add_line(parent,ph.Outport,mp.Inport(2),'autorouting','on'); add_line(parent,mp.Outport,lp.Inport);
end
end

function s=collect(output,r,label,index,mode,options)
t=output.trueState.Time; x=rows(output.trueState); th=output.estimatedState.Time; xhat=rows(output.estimatedState);
angles=(x(:,[6 5])-r.p.xeq([6 5])')*180/pi; target=r.feedback.reference'*180/pi; error=angles-target;
complete=abs(t(end)-r.test.duration)<1e-8;
valid=all(isfinite(x(:))) && all(abs(x(:,[5 6]))<r.p.cfg.stop.maxTiltDeg*pi/180+1e-7,'all') && all(x(:,3)>r.p.cfg.stop.minHipHeight-1e-7);
if complete && valid, status='Completed within validity bounds'; else, status='Stopped at validity bound'; end
pushEnd=r.test.pushCentre+3*r.test.pushWidth; recovery=nan(1,2); rebound=nan(1,2);
if complete && valid
    for j=1:2
        recovery(j)=recovery_time(t,error(:,j),pushEnd,options);
        select=find(t>=r.test.pushCentre); [~,peak]=max(abs(error(select,j))); peak=select(peak); signPeak=sign(error(peak,j));
        if signPeak==0, rebound(j)=0; else, rebound(j)=max([0;-signPeak*error(peak:end,j)]); end
    end
end
span=max(t(end)-t(1),eps); rms=sqrt(trapz(t,error.^2)/span); sat=[false false false];
if strcmp(mode,'nonlinear')
    vars={'balanceLimit','commandLimit','torqueLimit'};
    for j=1:3
        values=rows(output.get(vars{j})); n=size(values,2)/2; sat(j)=any(abs(values(:,1:n)-values(:,n+1:end))>1e-8,'all');
    end
    if index==1, sat(1:2)=false; end % unused outer-controller signals do not count
    assert(all(abs(x(:,29:36))<=r.p.tauMax'+1e-5,'all'),'Plant torque invariant failed.');
end
s=struct('output',output,'parameters',r,'time',t,'state',x,'estimateTime',th,'estimate',xhat, ...
    'anglesDeg',angles,'targetDeg',target,'status',status,'complete',complete,'valid',valid,'label',label,'pushEnd',pushEnd, ...
    'peakErrorDeg',max(abs(error),[],1),'rmsErrorDeg',rms,'recoverySec',recovery,'oppositeExcursionDeg',rebound, ...
    'peakTorqueNm',max(abs(x(:,29:36)),[],'all'),'limitsEnabled',strcmp(mode,'nonlinear'),'saturation',sat, ...
    'recoveryGoalMet',complete && valid && all(isfinite(recovery)) && all(recovery<=options.recoveryGoalSec), ...
    'oppositeGoalMet',complete && valid && all(isfinite(rebound)) && all(rebound<=options.angleBandDeg));
end
function seconds=recovery_time(t,error,start,options)
seconds=NaN; last=find(t>=start & abs(error)>options.angleBandDeg,1,'last');
if isempty(last), first=find(t>=start,1); else, first=last+1; end
if ~isempty(first) && first<=numel(t) && t(end)-t(first)>=options.minDwellSec, seconds=max(0,t(first)-start); end
end
function tableOut=metrics_table(cases,keys)
names=cell(3,1); completed=false(3,1); peak=zeros(3,2); rms=peak; rec=peak; opp=peak; torque=zeros(3,1); sat=false(3,3); active=false(3,1); recoveryOK=active; oppositeOK=active;
for j=1:3
    s=cases.(keys{j}); names{j}=s.label; completed(j)=s.complete && s.valid; peak(j,:)=s.peakErrorDeg;
    rms(j,:)=s.rmsErrorDeg; rec(j,:)=s.recoverySec; opp(j,:)=s.oppositeExcursionDeg;
    torque(j)=s.peakTorqueNm; sat(j,:)=s.saturation; active(j)=s.limitsEnabled;
    recoveryOK(j)=s.recoveryGoalMet; oppositeOK(j)=s.oppositeGoalMet;
end
tableOut=table(names,completed,peak(:,1),peak(:,2),rms(:,1),rms(:,2),rec(:,1),rec(:,2),opp(:,1),opp(:,2),torque,active,sat(:,1),sat(:,2),sat(:,3),recoveryOK,oppositeOK, ...
    'VariableNames',{'Case','CompletedValid','PeakRoll_deg','PeakPitch_deg','RMSRoll_deg','RMSPitch_deg','RecoveryRoll_s','RecoveryPitch_s', ...
    'OppositeRoll_deg','OppositePitch_deg','PeakTorque_Nm','LimitsEnabled','CorrectionClipped','CommandClipped','TargetTorqueClipped','RecoveryGoalMet','OppositeGoalMet'});
end
function values=rows(ts)
if ts.IsTimeFirst, values=reshape(ts.Data,numel(ts.Time),[]); else, values=reshape(ts.Data,[],numel(ts.Time))'; end
end


