function results=run_four_cases(plant,stopTime,showFigures,configuration,outputFolder)
% Four cases using the COPIED original plant, controller and observer.
% results=run_four_cases; or results=run_four_cases('linear').
% Added disturbances affect sensor readouts ONLY.
if nargin<1, plant='nonlinear'; end
plant=validatestring(plant,{'linear','nonlinear'});
root=fileparts(mfilename('fullpath')); cd(root);
addpath(root,fullfile(root,'helpers')); md_expanded_cache;
cfg=sf_parameters;
% Optional in-memory settings/output folder for batch tests. Saved defaults
% and source SLX files are not overwritten.
if nargin>=4 && ~isempty(configuration), cfg=configuration; end
if nargin<5 || isempty(outputFolder), outputFolder=fullfile(root,'figures'); end
if nargin<2 || isempty(stopTime), stopTime=cfg.test.duration; end
if nargin<3, showFigures=true; end

% Small sensor disturbances: gyro xyz, accelerometer xyz, eight encoders.
% Gyro/encoder settings below are converted from degrees into SI units.
sensorBias=[0.05*pi/180*ones(3,1);0.01*ones(3,1);0.10*pi/180*ones(8,1)];
sensorSineAmplitude=[0.05*pi/180*ones(3,1);0.01*ones(3,1);0.05*pi/180*ones(8,1)];
sensorOmega=10; % rad/s, sensor interference rather than an external moment

source=['microduck_' plant '_state_feedback']; sourceFile=fullfile(root,[source '.slx']);
if bdIsLoaded(source) && ~strcmpi(get_param(source,'FileName'),sourceFile)
    error('The model is open from another folder. Save/close that model before running this copy.');
end
load_system(sourceFile); md_sf_apply(source,cfg);
labels={'1. No outer balance controller','2. State feedback + observer', ...
    '3. Controller + constant sensor bias','4. Controller + constant and sinusoidal sensor errors'};
keys={'noController','controller','constantSensorError','constantAndSineSensorError'};
results=struct;
for j=1:4
    clone=copy_case(source); cleanup=onCleanup(@()close_case(clone));
    ws=get_param(clone,'ModelWorkspace'); r=getVariable(ws,'r');
    if j==1
        add_block('simulink/Sources/Constant',[clone '/Fixed standing targets'], ...
            'Value','r.p.ueq','Position',[60 170 240 205]);
        for destination={'Robot plant/1','State observer 50 Hz/2','Send jointCommand/1'}
            replace_input(clone,destination{1},'Fixed standing targets/1');
        end
    elseif j>=3
        assignin(ws,'measurementBias',sensorBias);
        if j==4, amplitude=sensorSineAmplitude; else, amplitude=zeros(14,1); end
        assignin(ws,'measurementSineAmplitude',amplitude); assignin(ws,'sensorOmega',sensorOmega);
        add_sensor_errors(clone);
    end
    add_validity_stop(clone); % Same stop as the original comparison script.
    fprintf('Running %s\n',labels{j}); out=sim(clone,'StopTime',num2str(stopTime));
    x=rows(out.trueState);
    results.(keys{j})=struct('label',labels{j},'time',out.trueState.Time, ...
        'state',x,'anglesDeg',(x(:,[6 5])-r.p.xeq([6 5])')*180/pi, ...
        'pushTime',out.worldPush.Time,'push',rows(out.worldPush), ...
        'plant',plant,'torqueLimits',r.p.tauMax,'pushForce',r.test.pushForce, ...
        'jointCommandTime',out.jointCommand.Time,'jointCommand',rows(out.jointCommand));
    results.(keys{j}).clipping=clipping_times(out,r,plant,j==1);
    clear cleanup;
end
if showFigures
    pushLabel=sprintf('Push: forward %.3g N, sideways %.3g N, vertical %.3g N',cfg.test.pushForce);
    plot_responses(results,keys,labels,plant,outputFolder,pushLabel);
    plot_motor_torques(results,plant,outputFolder);
end
end

function clipping=clipping_times(out,r,plant,noController)
% Check requested signals, not whether the lagged torque happens to equal its cap.
clipping=struct('limitsEnabled',strcmp(plant,'nonlinear'), ...
    'firstCorrectionClip_s',NaN,'firstJointCommandClip_s',NaN,'firstTorqueTargetClip_s',NaN);
if ~clipping.limitsEnabled, return; end
te=out.estimatedState.Time; estimate=rows(out.estimatedState);
du=-(r.feedback.K*(r.observer.S\(estimate-r.p.xeq(r.observer.keep)')'))';
if noController, du=zeros(size(du)); end
hit=any(abs(du)>r.feedback.maxCorrectionRad+1e-8,2);
if any(hit), clipping.firstCorrectionClip_s=te(find(hit,1)); end
limited=r.p.ueq'+min(max(du,-r.feedback.maxCorrectionRad),r.feedback.maxCorrectionRad);
hit=any(limited<r.p.jointMin'-1e-8 | limited>r.p.jointMax'+1e-8,2);
if any(hit), clipping.firstJointCommandClip_s=te(find(hit,1)); end
t=out.trueState.Time; x=rows(out.trueState);
ut=out.jointCommand.Time; u=rows(out.jointCommand);
[ut,idx]=unique(ut,'last'); u=u(idx,:);
if numel(ut)==1
    u=repmat(u,numel(t),1);
else
u=interp1(ut,u,t,'previous','extrap');
end
u=min(max(u,r.p.jointMin'),r.p.jointMax');
target=r.p.kp'.*(u-x(:,7:14))-r.p.kd'.*x(:,21:28);
hit=any(abs(target)>r.p.tauMax'+1e-8,2);
if any(hit), clipping.firstTorqueTargetClip_s=t(find(hit,1)); end
end

function add_sensor_errors(model)
ports=get_param([model '/State observer 50 Hz'],'PortHandles');
ln=get_param(ports.Inport(1),'Line'); src=get_param(ln,'SrcPortHandle');
while src<0, ln=get_param(ln,'LineParent'); src=get_param(ln,'SrcPortHandle'); end
delete_line(model,src,ports.Inport(1));
add_block('simulink/Sources/Constant',[model '/Constant sensor bias'], ...
    'Value','measurementBias','Position',[40 550 230 585]);
add_block('simulink/Sources/Sine Wave',[model '/Sinusoidal sensor error'], ...
    'Amplitude','measurementSineAmplitude','Frequency','sensorOmega','Phase','0', ...
    'Bias','0','SampleTime','r.observer.sampleTime','Position',[40 620 230 655]);
add_block('simulink/Math Operations/Sum',[model '/Disturbed measurements'], ...
    'Inputs','+++','Position',[300 540 340 635]);
p=get_param([model '/Disturbed measurements'],'PortHandles');
add_line(model,src,p.Inport(1),'autorouting','on');
add_line(model,'Constant sensor bias/1','Disturbed measurements/2');
add_line(model,'Sinusoidal sensor error/1','Disturbed measurements/3');
add_line(model,'Disturbed measurements/1','State observer 50 Hz/1','autorouting','on');
end

function plot_responses(results,keys,labels,plant,folder,pushLabel)
colors=lines(4); styles={'--','-','-.',':'};
if ~exist(folder,'dir'), mkdir(folder); end
for zoom=false:true
    if zoom, suffix='_zoom'; nrows=2; else, suffix=''; nrows=4; end
    f=figure('Color','w','Name',['Four cases: ' plant suffix], ...
        'NumberTitle','off','Position',[80 60 1100 800]);
    tiledlayout(nrows,1,'TileSpacing','compact','Padding','compact');
    for axisIndex=1:2
        nexttile; hold on;
        for j=1:4
            a=results.(keys{j}); plot(a.time,a.anglesDeg(:,axisIndex), ...
                'Color',colors(j,:),'LineStyle',styles{j},'LineWidth',1.4,'DisplayName',labels{j});
        end
        if axisIndex==1, ylabel('Roll from standing (deg)'); else, ylabel('Pitch from standing (deg)'); end
        xlabel('Time (s)'); grid on; yline(0,'k:','HandleVisibility','off');
        if zoom
            extent=.02;
            for j=2:4, extent=max(extent,max(abs(results.(keys{j}).anglesDeg(:,axisIndex)))); end
            ylim(1.2*[-extent extent]);
        end
        if axisIndex==1, legend('show','Location','southoutside','NumColumns',2,'FontSize',9); end
    end
    if ~zoom
        nexttile; a=results.controller; plot(a.pushTime,a.push(:,1),'k','LineWidth',1.4);
        xlabel('Time (s)'); ylabel('Forward force (N)'); title('Same original physical push in all four cases'); grid on;
        nexttile; plot(a.pushTime,a.push(:,2),'k','LineWidth',1.4);
        xlabel('Time (s)'); ylabel('Sideways force (N)'); title('Sideways component of the same physical push'); grid on;
    end
    if zoom, sgtitle({['Four cases - ' plant ' - zoom (uncontrolled curve may leave view)'],pushLabel});
    else, sgtitle({['Four cases - ' plant ' - actual plant response'],pushLabel}); end
    exportgraphics(f,fullfile(folder,['four_cases_' plant suffix '.png']),'Resolution',160);
    savefig(f,fullfile(folder,['four_cases_' plant suffix '.fig']));
end
end
function v=rows(ts)
if ts.IsTimeFirst, v=reshape(ts.Data,numel(ts.Time),[]); else, v=reshape(ts.Data,[],numel(ts.Time))'; end
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
