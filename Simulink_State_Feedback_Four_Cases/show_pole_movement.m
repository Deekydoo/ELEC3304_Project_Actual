function details=show_pole_movement(outputFolder)
% SHOW_POLE_MOVEMENT Visualise the existing sampled controller's pole placement.
% Run this function from MATLAB; it opens one figure and saves PNG/FIG copies.
% No simulation, retuning, plant changes or controller changes are performed.
%
% IMPORTANT: feedback STABILISES an already controllable plant. It does not
% make an uncontrollable plant controllable. Because the controller is digital,
% the actual closed-loop matrix is Ad-Bd*K, not A-B*K using the sampled gain.
% Continuous-equivalent slow poles are displayed using s=log(z)/Ts.
root=fileparts(mfilename('fullpath')); addpath(root,fullfile(root,'helpers'));
cfg=sf_parameters; r=md_sf_prepare(cfg);
A=r.observer.continuousA; B=r.observer.continuousB;
Ad=r.observer.Ad; Bd=r.observer.Bd; K=r.feedback.K; Ts=r.observer.sampleTime;
openS=eig(A); openZ=eig(Ad); closedZ=eig(Ad-Bd*K);
observerZ=eig(r.observer.errorMatrix);

% Reconstruct exactly the slow-mode coordinates used by md_sf_prepare.
[U,T]=schur(Ad,'real'); fast=abs(ordeig(T))<=exp(-cfg.feedback.slowModeCutoffRate*Ts);
[U,T]=ordschur(U,T,fast); nf=sum(fast); V=U(:,nf+1:end);
Aslow=V'*Ad*V; Bslow=V'*Bd; Kslow=K*V; ns=size(Aslow,1);
assert(ns==numel(cfg.feedback.assignedDecayRates),'Selected-mode count differs from the current design.');

% Show the actual eigenvalue trajectories while smoothly applying the
% EXISTING gain: u=-g*K*xhat, with 0<=g<=1. This is not a new gain design.
gain=linspace(0,1,301); zPath=complex(zeros(ns,numel(gain)));
initial=eig(Aslow); [~,order]=sort(real(initial),'descend'); zPath(:,1)=initial(order);
choices=perms(1:ns);
for j=2:numel(gain)
    poles=eig(Aslow-gain(j)*Bslow*Kslow); best=Inf;
    for k=1:size(choices,1)
        candidate=poles(choices(k,:)); candidate=candidate(:);
        cost=sum(abs(candidate-zPath(:,j-1)).^2);
        if cost<best, best=cost; selected=candidate; end
    end
    zPath(:,j)=selected;
end
sPath=log(zPath)/Ts;
protected=ordeig(T(1:nf,1:nf)); protectedRadius=max([0;abs(protected)]);
radius=max([max(abs(zPath),[],1);protectedRadius*ones(size(gain))],[],1);

% PBH rank checks avoid the high matrix powers in ctrb() for a stiff model.
[plantRank,minSingular]=pbh_reachability(A,B);
[slowBeforeRank,~]=pbh_reachability(Aslow,Bslow);
[slowAfterRank,~]=pbh_reachability(Aslow-Bslow*Kslow,Bslow);
assert(plantRank==size(A,1) && slowBeforeRank==ns && slowAfterRank==ns, ...
    'Review the PBH rank checks before claiming controllability.');
assert(max(abs(closedZ))<1 && max(abs(observerZ))<1,'The nominal controller/observer is not stable.');
assert(max(abs(sort(real(zPath(:,end)))-sort(real(r.feedback.assignedSlowPoles))))<1e-6, ...
    'The displayed final poles do not match the existing controller.');

movement=table(sPath(:,1),sPath(:,end),zPath(:,1),zPath(:,end), ...
    'VariableNames',{'OpenPole_s','ClosedEquivalentPole_s','OpenPole_z','ClosedPole_z'});
fprintf('\nThe plant is already controllable; feedback makes the closed loop stable.\n');
fprintf('Continuous balance-plant PBH rank: %d / %d (minimum singular value %.6g).\n',plantRank,size(A,1),minSingular);
fprintf('Selected sampled subsystem PBH ranks: %d / %d before; %d / %d after feedback.\n',slowBeforeRank,ns,slowAfterRank,ns);
fprintf('Open-loop unstable pole: +%.6f /s.\n',max(real(openS)));
fprintf('Existing assigned decay rates: '); fprintf('%g ',cfg.feedback.assignedDecayRates); fprintf('/s.\n');
fprintf('Actual sampled closed-loop spectral radius: %.9f < 1.\n',max(abs(closedZ)));
fprintf('Observer error spectral radius: %.9f < 1.\n',max(abs(observerZ)));
fprintf('The s-plane closed-loop values below are log(z)/Ts equivalents.\n');
disp(movement);

f=figure('Color','w','Name','How the existing controller stabilises the plant', ...
    'NumberTitle','off','Position',[50 120 1500 490]);
tiledlayout(1,3,'TileSpacing','loose','Padding','loose');
colors=lines(ns); colors(1,:)=[.80 .15 .12];
nexttile; hold on; xline(0,'k:','Stability boundary','HandleVisibility','off');
for k=1:ns
    plot(real(sPath(k,:)),imag(sPath(k,:)),'Color',colors(k,:),'LineWidth',1.6,'HandleVisibility','off');
end
plot(real(sPath(:,1)),imag(sPath(:,1)),'rx','MarkerSize',10,'LineWidth',1.8,'DisplayName','Before feedback');
plot(real(sPath(:,end)),imag(sPath(:,end)),'bo','MarkerSize',8,'LineWidth',1.6,'DisplayName','With existing feedback');
ylim(1.3*[-1 1]*max(.4,max(abs(imag(sPath)),[],'all')));
xlabel('Real part (1/s)'); ylabel('Imaginary part (rad/s)'); grid on;
title({'Slow balance-mode pole movement','Continuous equivalents: s = log(z)/T_s'});
text(real(sPath(1,1)),imag(sPath(1,1))+.12,sprintf('  +%.3f: unstable',real(sPath(1,1))),'FontSize',9);
legend('Location','southoutside');

nexttile; hold on; theta=linspace(0,2*pi,500);
plot(cos(theta),sin(theta),'k:','DisplayName','Unit circle');
plot(real(openZ),imag(openZ),'rx','MarkerSize',7,'LineWidth',1.2,'DisplayName','Plant: before feedback');
plot(real(closedZ),imag(closedZ),'bo','MarkerSize',6,'LineWidth',1.2,'DisplayName','Plant: with feedback');
plot(real(observerZ),imag(observerZ),'g.','MarkerSize',8,'DisplayName','Observer error poles');
plot(real(zPath(1,:)),imag(zPath(1,:)),'Color',colors(1,:),'LineWidth',1.4,'HandleVisibility','off');
axis equal; xlim([-1.1 1.12]); ylim([-1.1 1.1]); grid on;
xlabel('Real part'); ylabel('Imaginary part'); title({'Actual sampled poles','Stable when |z| < 1'});
legend('Location','southoutside','FontSize',8);

nexttile; hold on; plot(gain,radius,'Color',[0 .35 .7],'LineWidth',1.8);
yline(1,'r--','Stability boundary'); plot(1,radius(end),'bo','MarkerFaceColor','b');
grid on; xlabel('Applied fraction of the existing gain, g'); ylabel('Largest plant-pole magnitude');
title({'Applying the existing controller','g = 0: no outer feedback; g = 1: full feedback'});
text(.98,radius(end),sprintf('  %.6f',radius(end)),'HorizontalAlignment','right','VerticalAlignment','top');
sgtitle({'The original plant is controllable; state feedback stabilises it', ...
    'Red trajectory starts at the unstable mode. Other slow modes move too; fast stable modes are retained.'});

folder=fullfile(root,'figures');
if nargin>=1 && ~isempty(outputFolder), folder=outputFolder; end
if ~exist(folder,'dir'), mkdir(folder); end
exportgraphics(f,fullfile(folder,'pole_movement.png'),'Resolution',180);
savefig(f,fullfile(folder,'pole_movement.fig'));
writetable(movement,fullfile(folder,'pole_movement.csv'));
details=struct('openContinuousPoles',openS,'openDiscretePoles',openZ, ...
    'closedDiscretePoles',closedZ,'observerPoles',observerZ,'movement',movement, ...
    'gainFraction',gain,'slowPolePaths_z',zPath,'slowPolePaths_sEquivalent',sPath, ...
    'plantSpectralRadius',radius,'continuousPlantPBHRank',plantRank, ...
    'slowPBHRankBefore',slowBeforeRank,'slowPBHRankAfter',slowAfterRank);
end

function [minimumRank,minimumSingular]=pbh_reachability(A,B)
n=size(A,1); minimumRank=n; minimumSingular=Inf;
for pole=eig(A).'
    M=[pole*eye(n)-A B]; s=svd(M);
    tolerance=max(size(M))*eps(max(s));
    minimumRank=min(minimumRank,sum(s>tolerance)); minimumSingular=min(minimumSingular,min(s));
end
end
