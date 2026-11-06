function md_sf_plot_cross(results,root,options)
% Compare the two plant representations under each identical controller case.
keys={'openLoop','stateTrue','stateObserver'};
names={'Fixed standing joint targets','State feedback using true states','State feedback using observer states'};
angles={'Roll','Pitch'}; styles={'-','--'}; colors=[0 .38 .70;.85 .30 .08];
f=figure('Color','w','Position',[100 40 1300 1000],'Name','Pole placement: linear versus nonlinear');
tiledlayout(3,2,'TileSpacing','loose','Padding','loose');
for j=1:3
    for axisIndex=1:2
        nexttile; hold on;
        extent=0;
        for k=1:2
            modes={'linear','nonlinear'}; mode=modes{k};
            s=results.(mode).cases.(keys{j}); label=mode;
            extent=max(extent,max(abs(s.anglesDeg(:,axisIndex))));
            if ~s.complete, label=sprintf('%s (validity stop %.2f s)',mode,s.time(end)); end
            plot(s.time,s.anglesDeg(:,axisIndex),'LineStyle',styles{k},'Color',colors(k,:), ...
                'LineWidth',1.4,'DisplayName',label);
        end
        yline(0,':','Target','Color',[.1 .45 .2],'DisplayName','Target');
        if extent<0.02, ylim([-0.02 0.02]); end % avoid magnifying numerical roundoff
        title([angles{axisIndex} ' | ' names{j}]); xlabel('Time (s)'); ylabel('Change from standing (deg)');
        grid on; legend('show','Location','southoutside','FontSize',9);
    end
end
sgtitle('Same pole-placement gain, observer and physical parameters: linear versus nonlinear plant');
if options.exportFigures
    exportgraphics(f,fullfile(root,'figures','linear_vs_nonlinear.png'),'Resolution',180);
end
end
