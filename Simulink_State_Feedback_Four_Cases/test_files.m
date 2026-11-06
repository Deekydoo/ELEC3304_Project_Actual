%% Six push tests, both plant models, four sensor/controller cases per test
% Run: test_files
% pushForce is measured in N (not N m): 1, 5 and 10 N in each direction.
root=fileparts(mfilename('fullpath'));
cd(root); addpath(root,fullfile(root,'helpers'));
outputFolder=fullfile(root,'figures_new');
if ~exist(outputFolder,'dir'), mkdir(outputFolder); end
baseConfiguration=sf_parameters;
pushMagnitudes=[1 5 10];
pushDirections={'forward','sideways'};
plants={'nonlinear','linear'};
keepFiguresOpen=false; % Set true to keep every batch figure open.
allResults=struct; savedPlotFiles={};
allPerformance=table;

%% Run every push setting without editing sf_parameters.m
for directionIndex=1:numel(pushDirections)
    direction=pushDirections{directionIndex};
    for magnitude=pushMagnitudes
        testName=sprintf('%s_%gN',direction,magnitude);
        testFolder=fullfile(outputFolder,testName);
        if ~exist(testFolder,'dir'), mkdir(testFolder); end
        testConfiguration=baseConfiguration;
        if strcmp(direction,'forward'), testConfiguration.test.pushForce=[magnitude;0;0];
        else, testConfiguration.test.pushForce=[0;magnitude;0]; end
        testConfiguration.test.pushMoment=zeros(3,1);
        save(fullfile(testFolder,'test_configuration.mat'),'testConfiguration');
        for plantIndex=1:numel(plants)
            plant=plants{plantIndex}; plantFolder=fullfile(testFolder,plant);
            if ~exist(plantFolder,'dir'), mkdir(plantFolder); end
            fprintf('\n=== %s | %s plant ===\n',testName,plant);
            figuresBefore=findall(groot,'Type','figure');
            caseResults=run_four_cases(plant,[],true,testConfiguration,plantFolder);
            allResults.(testName).(plant)=caseResults;
            % Evaluate controller performance (actual roll and pitch).
            performance=evaluate_push_performance(caseResults,testConfiguration);
            performance.Test=repmat(string(testName),height(performance),1);
            performance.Plant=repmat(string(plant),height(performance),1);
            writetable(performance,fullfile(plantFolder,'controller_performance.csv'));
            allPerformance=[allPerformance;performance];
            writetable(allPerformance,fullfile(outputFolder,'controller_performance_all.csv'));
            disp(performance);
            if strcmp(plant,'linear'), linearResults=caseResults;
            else, nonlinearResults=caseResults; end
            savedPlotFiles=[savedPlotFiles;save_new_plots(figuresBefore,plantFolder,keepFiguresOpen)];
        end
    end
end

%% Record and tabulate 1, 5 and 10 N forward/sideways performance
% Run this section alone AFTER the simulations to rebuild tables from CSVs.
% Force is N, not N m. Forward tests use pitch; sideways tests use roll.
% Rise/overshoot below are the pulse-response analogues defined in
% evaluate_push_performance.m, not nonzero-reference step metrics.
root=fileparts(mfilename('fullpath'));
outputFolder=fullfile(root,'figures_new');
performanceTable=table;
for direction=["forward","sideways"]
    for forceN=[1 5 10]
        for plant=["nonlinear","linear"]
            csvFile=fullfile(outputFolder,sprintf('%s_%gN',direction,forceN), ...
                plant,'controller_performance.csv');
            if ~isfile(csvFile)
                warning('Missing results: %s. Run the simulation section first.',csvFile);
                continue;
            end
            recorded=readtable(csvFile,'TextType','string');
            if direction=="forward", responseAxis="Pitch"; else, responseAxis="Roll"; end
            selected=recorded(recorded.Axis==responseAxis,:);
            rows=table(repmat(direction,height(selected),1), ...
                repmat(forceN,height(selected),1),repmat(plant,height(selected),1), ...
                selected.Case,selected.Axis,selected.ExcursionRiseTime_s, ...
                selected.PeakTime_s,selected.SettlingTime_s, ...
                selected.ReverseOvershoot_pct,selected.RunCompleted, ...
                selected.SettledObserved,'VariableNames', ...
                {'Direction','Force_N','Plant','Case','Axis','ExcursionRiseTime_s', ...
                'PeakTime_s','SettlingTime_s','ReverseOvershoot_pct', ...
                'RunCompleted','SettledObserved'});
            performanceTable=[performanceTable;rows]; %#ok<AGROW>
        end
    end
end
if ~isempty(performanceTable)
    % All four cases, both plants: 48 rows when all simulations are present.
    writetable(performanceTable,fullfile(outputFolder,'push_performance_table.csv'));
    disp('Push performance: all four controller/sensor cases');
    disp(performanceTable);
    % Six-row clean-sensor controller comparison for each plant model.
    nominalControllerTable=performanceTable(performanceTable.Case=="controller",:);
    writetable(nominalControllerTable,fullfile(outputFolder,'controller_six_push_summary.csv'));
    for plant=["nonlinear","linear"]
        fprintf('\n%s plant: state feedback + observer, six push tests\n',plant);
        disp(nominalControllerTable(nominalControllerTable.Plant==plant,:));
    end
    save(fullfile(outputFolder,'push_performance_tables.mat'), ...
        'performanceTable','nominalControllerTable');
end

%% Pole placement is identical across push amplitudes/directions: save once
poleFolder=fullfile(outputFolder,'pole_placement');
figuresBefore=findall(groot,'Type','figure');
poleDetails=show_pole_movement(poleFolder);
savedPlotFiles=[savedPlotFiles;save_new_plots(figuresBefore,poleFolder,keepFiguresOpen)];

fprintf('\nSaved %d PNG plots to:\n%s\n',numel(savedPlotFiles),outputFolder);

function files=save_new_plots(figuresBefore,outputFolder,keepOpen)
figuresNow=findall(groot,'Type','figure');
newFigures=figuresNow(~ismember(figuresNow,figuresBefore));
plots=dir(fullfile(outputFolder,'*.png'));
files=arrayfun(@(p)fullfile(p.folder,p.name),plots,'UniformOutput',false);
files=files(:);
for k=1:numel(newFigures)
    f=newFigures(k);
    % Plotting functions already save PNG and editable FIG files here.
    if ~keepOpen, close(f); end
end
end
