function md_sf_plot_comparison(cases,mode,root,options)
% The same three physical trajectories in an overview and a vertical zoom.
keys={'openLoop','stateTrue','stateObserver'};
labels={'Fixed standing targets','State feedback: true states','State feedback + observer'};
colors=[.38 .38 .38;0 .38 .70;.85 .30 .08]; styles={'--','-','-.'};
f=figure('Color','w','Position',[70 70 1350 900],'Name',['MicroDuck comparison - ' mode]);
tiledlayout(2,2,'TileSpacing','loose','Padding','loose');
for axisIndex=1:2
    nexttile; draw_angles(cases,keys,labels,colors,styles,axisIndex);
end
nexttile; s=cases.stateObserver; w=rows(s.output.worldPush); plot(s.output.worldPush.Time,w(:,1:3),'LineWidth',1.3);
title('Same external force in all three cases'); xlabel('Time (s)'); ylabel('World force (N)'); grid on;
legend('Fx','Fy','Fz','Location','southoutside');
nexttile; hold on;
for j=1:3
    s=cases.(keys{j}); plot(s.time,max(abs(s.state(:,29:36)),[],2),'LineStyle',styles{j}, ...
        'Color',colors(j,:),'LineWidth',1.4,'DisplayName',labels{j});
end
cap=cases.stateObserver.parameters.p.tauMax;
if max(cap)-min(cap)<1e-10, yline(cap(1),':','Torque cap','HandleVisibility','off'); end
title('Largest absolute actual torque across eight servos'); xlabel('Time (s)'); ylabel('Torque (Nm)'); grid on;
legend('show','Location','southoutside','FontSize',9);
sgtitle(sprintf('MicroDuck pole placement | %s plant | fixed targets vs state feedback vs observer feedback',mode));
if options.exportFigures, exportgraphics(f,fullfile(root,'figures',[mode '_response.png']),'Resolution',180); end

detail=figure('Color','w','Position',[100 100 1350 590],'Name',['MicroDuck angle detail - ' mode]);
tiledlayout(1,2,'TileSpacing','loose','Padding','loose');
for axisIndex=1:2
    nexttile; draw_angles(cases,keys,labels,colors,styles,axisIndex);
    target=cases.stateObserver.targetDeg(axisIndex);
    extent=max([.02;abs(cases.stateTrue.anglesDeg(:,axisIndex)-target);abs(cases.stateObserver.anglesDeg(:,axisIndex)-target)]);
    ylim(target+1.2*[-extent extent]);
    xline(cases.stateObserver.pushEnd,':','Practical pulse end','HandleVisibility','off','LabelVerticalAlignment','bottom');
end
sgtitle({['MicroDuck | ' mode ' | actual angle responses: vertical zoom'], ...
    'All three traces retained; the uncontrolled response can leave this view'});
if options.exportFigures, exportgraphics(detail,fullfile(root,'figures',[mode '_angle_detail.png']),'Resolution',180); end
end
function draw_angles(cases,keys,labels,colors,styles,axisIndex)
hold on;
for j=1:3
    s=cases.(keys{j}); label=labels{j};
    if ~s.complete, label=sprintf('%s (validity stop %.2f s)',label,s.time(end)); end
    plot(s.time,s.anglesDeg(:,axisIndex),'LineStyle',styles{j},'Color',colors(j,:), ...
        'LineWidth',1.5,'DisplayName',label);
    if ~s.complete
        plot(s.time(end),s.anglesDeg(end,axisIndex),'x','Color',colors(j,:),'HandleVisibility','off','MarkerSize',9);
    end
end
s=cases.stateObserver;
extent=0;
for j=1:numel(keys), extent=max(extent,max(abs(cases.(keys{j}).anglesDeg(:,axisIndex)))); end
if extent<0.02, ylim([-0.02 0.02]); end
yline(s.targetDeg(axisIndex),':','Target','Color',[.1 .45 .2],'LineWidth',1.3,'DisplayName','Target');
titles={'Roll: actual response','Pitch: actual response'}; title(titles{axisIndex});
xlabel('Time (s)'); ylabel('Change from standing (deg)'); grid on; legend('show','Location','southoutside','FontSize',9);
end
function values=rows(ts)
if ts.IsTimeFirst, values=reshape(ts.Data,numel(ts.Time),[]); else, values=reshape(ts.Data,[],numel(ts.Time))'; end
end

