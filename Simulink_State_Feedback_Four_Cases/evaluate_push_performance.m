function metrics=evaluate_push_performance(results,cfg)
% Push recovery is not a reference-step response. Standard step rise time
% and percentage overshoot are undefined for a zero-to-zero reference.
% Instead report explicitly named pulse-response analogues:
% ExcursionRiseTime_s: first 10%-90% crossings towards largest excursion.
% PeakTime_s: largest absolute excursion, timed from push-window start.
% SettlingTime_s: first time after that peak which remains within +/-2% of
% peak tilt about upright for the rest of the run (at least 0.5 s observed).
% ReverseOvershoot_pct: opposite-side excursion AFTER the largest peak,
% divided by that peak. This is NOT lecture step-response overshoot.
% Gaussian push-window start = centre - 3*width (clamped to zero).
startTime=max(0,cfg.test.pushCentre-3*cfg.test.pushWidth);
keys=fieldnames(results); axesNames=["Roll" "Pitch"]; metrics=table;
for k=1:numel(keys)
    r=results.(keys{k}); t=r.time(:);
    completed=t(end)>=cfg.test.duration-1e-6;
    for a=1:2
        use=t>=startTime; tt=t(use); y=r.anglesDeg(use,a);
        rise=NaN; peakTime=NaN; peak=NaN; settling=NaN;
        reverse=NaN; band=NaN; finalTilt=NaN; settled=false;
        if ~isempty(tt) && all(isfinite(y))
            finalTilt=y(end); [peak,ip]=max(abs(y));
            if peak>1e-8
                peakTime=tt(ip)-startTime;
                signed=y*sign(y(ip));
                t10=crossing_time(tt(1:ip),signed(1:ip),0.1*peak);
                t90=crossing_time(tt(1:ip),signed(1:ip),0.9*peak);
                rise=t90-t10;
                reverse=100*max(0,-min(signed(ip:end)))/peak;
                band=0.02*peak;
                lastOutside=find(abs(y)>band,1,'last');
                candidate=max(ip,lastOutside+1);
                if completed && candidate<=numel(tt) && tt(end)-tt(candidate)>=0.5
                    settling=tt(candidate)-startTime; settled=true;
                end
            end
        end
        row=table(string(keys{k}),axesNames(a),rise,peakTime,settling, ...
            reverse,peak,band,finalTilt,settled,completed,t(end),NaN,NaN, ...
            'VariableNames',{'Case','Axis','ExcursionRiseTime_s','PeakTime_s', ...
            'SettlingTime_s','ReverseOvershoot_pct','PeakAbsTilt_deg', ...
            'SettlingBand_deg','FinalTilt_deg','SettledObserved','RunCompleted', ...
            'SimulationEnd_s','StepRiseTime_s','StepOvershoot_pct'});
        metrics=[metrics;row]; %#ok<AGROW>
    end
end
end

function tc=crossing_time(t,y,threshold)
i=find(y>=threshold,1); tc=NaN;
if isempty(i), return; end
% A crossing before the recorded window cannot be timed reliably.
if i==1, return; end
tc=t(i-1)+(threshold-y(i-1))*(t(i)-t(i-1))/(y(i)-y(i-1));
end
