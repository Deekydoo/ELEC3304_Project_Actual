function figures=plot_motor_torques(results,plant,outputFolder)
% Plot all eight ACTUAL delivered torques (physical states 29:36).
% Run after a simulation, without rerunning it:
%   plot_motor_torques(linearResults,'linear')
%   plot_motor_torques(results,'nonlinear')
% The four-case runner calls this automatically when figures are enabled.
root=fileparts(mfilename('fullpath'));
if nargin<2 || isempty(plant)
    if isfield(results.controller,'plant'), plant=results.controller.plant;
    else, plant='unspecified'; end
end
if nargin<3 || isempty(outputFolder), outputFolder=fullfile(root,'figures'); end
if ~exist(outputFolder,'dir'), mkdir(outputFolder); end
keys={'noController','controller','constantSensorError','constantAndSineSensorError'};
colors=lines(4); styles={'--','-','-.',':'};
if isfield(results.controller,'torqueLimits')
    caps=results.controller.torqueLimits(:);
else
    cfg=sf_parameters; caps=cfg.servo.maxTorque(:);
end
if isscalar(caps), caps=repmat(caps,8,1); end
names={'Hip roll','Hip pitch','Knee','Ankle'}; sides={'Left','Right'};
figures=gobjects(2,1);
for zoom=0:1
    if zoom, suffix='_zoom'; else, suffix=''; end
    f=figure('Color','w','Name',['Eight motor torques - ' plant suffix], ...
        'NumberTitle','off','Position',[65 45 1300 900]); figures(zoom+1)=f;
    layout=tiledlayout(4,2,'TileSpacing','compact','Padding','compact'); axesHandles=gobjects(8,1);
    for row=1:4
        for side=1:2
            joint=row+4*(side-1); ax=nexttile; axesHandles(joint)=ax; hold(ax,'on');
            for j=1:4
                a=results.(keys{j});
                plot(ax,a.time,a.state(:,28+joint),'Color',colors(j,:), ...
                    'LineStyle',styles{j},'LineWidth',1.2,'DisplayName',a.label);
            end
            if strcmp(plant,'linear'), limitName='reference'; else, limitName='limit'; end
            yline(ax,caps(joint),'k:',sprintf('+%.2f %s',caps(joint),limitName),'HandleVisibility','off');
            yline(ax,-caps(joint),'k:',sprintf('-%.2f %s',caps(joint),limitName),'HandleVisibility','off');
            title(ax,[sides{side} ' ' lower(names{row})]); ylabel(ax,'Torque (N m)'); grid(ax,'on');
            if row==4, xlabel(ax,'Time (s)'); end
            if zoom
                low=Inf; high=-Inf;
                for j=2:4
                    tau=results.(keys{j}).state(:,28+joint);
                    low=min(low,min(tau)); high=max(high,max(tau));
                end
                margin=max(5e-5,.12*(high-low)); ylim(ax,[low-margin high+margin]);
                text(ax,.98,.96,sprintf('Caps: +/-%.2f N m',caps(joint)), ...
                    'Units','normalized','HorizontalAlignment','right','VerticalAlignment','top','FontSize',8);
            else
                extent=caps(joint);
                for j=1:4, extent=max(extent,max(abs(results.(keys{j}).state(:,28+joint)))); end
                ylim(ax,1.12*[-extent extent]);
            end
        end
    end
    lgd=legend(axesHandles(1),'show','NumColumns',2,'FontSize',9); lgd.Layout.Tile='south';
    if strcmp(plant,'linear')
        note='Linear model: dashed caps are reference values; saturation is bypassed.';
    elseif strcmp(plant,'nonlinear')
        note='Nonlinear model: torque targets are limited before the actuator lag.';
    else, note='Dashed lines show the configured torque caps.'; end
    if zoom, note=[note ' Controlled-response zoom; baseline or caps may leave view.']; end
    caption={['All eight delivered motor torques - ' plant],note, ...
        'Total torque includes the standing holding torque.'};
    if isfield(results.controller,'pushForce')
        caption{end+1}=sprintf('Push: forward %.3g N, sideways %.3g N, vertical %.3g N',results.controller.pushForce);
    end
    title(layout,caption);
    exportgraphics(f,fullfile(outputFolder,['motor_torques_' plant suffix '.png']),'Resolution',170);
    savefig(f,fullfile(outputFolder,['motor_torques_' plant suffix '.fig']));
end
end
