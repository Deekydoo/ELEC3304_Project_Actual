function r=md_sf_prepare(c,varargin)
% Rebuild both model representations from one physical configuration.
% Uses the same rigid-body helper as the visible nonlinear geometry blocks.
% Public evaluation interface reuses these exact equations in report diagnostics.
if ischar(c) || isstring(c)
    [dx,aux]=dynamics(varargin{1},varargin{2},varargin{3},varargin{4});
    if strcmp(c,'rhs'), r=dx; elseif strcmp(c,'diagnostics'), r=aux;
    else, error('Unknown plant evaluation action.'); end
    return
end
persistent previousConfiguration previousDesign
if isequaln(c,previousConfiguration), r=previousDesign; return; end
validateattributes(c.sampleTime,{'numeric'},{'scalar','positive','finite'});
validateattributes(c.test.duration,{'numeric'},{'scalar','positive','finite'});
validateattributes(c.test.pushWidth,{'numeric'},{'scalar','positive','finite'});
validateattributes(c.test.pushCentre,{'numeric'},{'scalar','nonnegative','finite'});
assert(c.test.duration>c.test.pushCentre+3*c.test.pushWidth,'Test must include the push and its practical end.');
c.test.pushForce=column(c.test.pushForce,3); c.test.pushMoment=column(c.test.pushMoment,3);
cfg.robot=c.robot; cfg.servo=c.servo; cfg.contact=c.contact;
for field={'kp','kd','timeConstant','maxTorque','jointDamping','minCommandDeg','maxCommandDeg'}
    cfg.servo.(field{1})=column(cfg.servo.(field{1}),8);
end
assert(all(cfg.servo.kp>0) && all(cfg.servo.kd>=0) && all(cfg.servo.timeConstant>0) && ...
    all(cfg.servo.maxTorque>0) && all(cfg.servo.jointDamping>=0),'Invalid servo parameters.');
assert(all(cfg.servo.minCommandDeg<cfg.servo.maxCommandDeg),'Invalid joint command limits.');
for field={'massTorso','massCarrier','massThigh','massShin','massFoot','thighLength','shinLength','footLength','footWidth'}
    validateattributes(cfg.robot.(field{1}),{'numeric'},{'scalar','positive','finite'});
end
for field={'stiffness','damping','frictionVelocity'}
    validateattributes(cfg.contact.(field{1}),{'numeric'},{'scalar','positive','finite'});
end
validateattributes(cfg.contact.friction,{'numeric'},{'scalar','nonnegative','finite'});
cfg.sim=struct('sampleTime',c.sampleTime,'stopTime',c.test.duration, ...
    'maxStep',min([c.solver.maxStep,c.sampleTime,c.test.pushWidth/8]),'relTol',c.solver.relTol,'absTol',c.solver.absTol);
cfg.stop=struct('enabled',true,'maxTiltDeg',c.validity.maxTiltDeg,'minHipHeight',c.validity.minHipHeight);
cfg.measurements.includeJointVelocity=false;
p=trim_standing(make_parameters(cfg));
assert(norm(dynamics(p.xeq,p.ueq,zeros(6,1),p),inf)<1e-5,'Standing equilibrium is inconsistent.');
[M,~,~]=md_rigid_body_math(p.xeq(1:14),zeros(14,1),p); assert(min(eig(M))>0,'Mass matrix is not positive definite.');

% Derivatives of the same nominal nonlinear RHS and ideal sensor equations.
nx=36; A=zeros(nx); B=zeros(nx,8); C=zeros(14,nx); D=zeros(14,8); E=zeros(nx,6); Fw=zeros(14,6);
x=p.xeq; u=p.ueq; w=zeros(6,1);
for j=1:nx
    e=zeros(nx,1); e(j)=1e-6*max(1,abs(x(j)));
    [fp,yp]=dynamics(x+e,u,w,p); [fm,ym]=dynamics(x-e,u,w,p);
    A(:,j)=(fp-fm)/(2*e(j)); C(:,j)=(yp.yIdeal-ym.yIdeal)/(2*e(j));
end
for j=1:8
    e=zeros(8,1); e(j)=1e-6;
    [fp,yp]=dynamics(x,u+e,w,p); [fm,ym]=dynamics(x,u-e,w,p);
    B(:,j)=(fp-fm)/(2*e(j)); D(:,j)=(yp.yIdeal-ym.yIdeal)/(2*e(j));
end
for j=1:6
    e=zeros(6,1); e(j)=1e-5;
    [fp,yp]=dynamics(x,u,e,p); [fm,ym]=dynamics(x,u,-e,p);
    E(:,j)=(fp-fm)/(2*e(j)); Fw(:,j)=(yp.yIdeal-ym.yIdeal)/(2*e(j));
end
[~,measurements]=dynamics(x,u,w,p); y0=measurements.yIdeal;
assert(norm(D,'fro')<1e-10,'This teaching interface requires zero command-to-sensor feedthrough.');
scale=[.01;.01;.01;.1;.1;.1;.1*ones(8,1);.1;.1;.1;ones(3,1);ones(8,1);.1*ones(8,1)];
lin=struct('A',A,'B',B,'C',C(1:6,:),'D',D(1:6,:),'C_with_joint_angles',C, ...
    'D_with_joint_angles',D,'C_measurements',C,'D_measurements',D,'xeq',x,'ueq',u, ...
    'yeq',y0(1:6),'yeq_measurements',y0,'stateScale',scale,'controlSampleTime',c.sampleTime);
keep=setdiff(1:36,[1 2 4],'stable'); S=diag(scale(keep)); As=S\A(keep,keep)*S; Bs=S\B(keep,:);
assert(norm(S\A(keep,[1 2 4]),'fro')<1e-8*max(1,norm(As,'fro')),'Retained dynamics depend on omitted coordinates.');
yScale=[ones(3,1);9.81*ones(3,1);.1*ones(8,1)]; Cn=diag(1./yScale)*C(:,keep)*S; Dn=diag(1./yScale)*D;
d=c2d(ss(As,Bs,Cn,Dn),c.sampleTime,'zoh');
settings=c.observer; settings.initialEstimateDelta=zeros(36,1);
settings.initialEstimateDelta([6 5])=column(c.observer.initialAngleErrorDeg,2)*pi/180;
if isfield(c,'reportPlantOnly') && c.reportPlantOnly
    % Analysis of perturbed plants with FIXED nominal K/L: do not redesign.
    r=struct('p',p,'lin',lin,'aux',struct('E',E,'Fw',Fw), ...
        'scaled',struct('Ad',d.A,'Bd',d.B,'Cn',Cn,'Dn',Dn,'As',As,'Bs',Bs,'S',S,'keep',keep));
    return
end
% Lectures 9-11: sampled pole-placement state feedback and Luenberger observer.
rates=column(c.feedback.assignedDecayRates,numel(c.feedback.assignedDecayRates));
assert(all(rates>0),'Controller decay rates must be positive.');
[Uc,Tc]=schur(d.A,'real'); cutoff=exp(-c.feedback.slowModeCutoffRate*c.sampleTime);
fastController=abs(ordeig(Tc))<=cutoff; [Uc,Tc]=ordschur(Uc,Tc,fastController);
nfc=sum(fastController); Vc=Uc(:,nfc+1:end); Acs=Vc'*d.A*Vc; Bcs=Vc'*d.B;
assert(size(Acs,1)==numel(rates),'The number of selected controller modes changed; revise the pole list/cutoff.');
requestedSlow=eig(Acs); [~,order]=sort(real(requestedSlow),'descend');
assert(all(abs(imag(requestedSlow))<1e-8), ...
    'Chosen slow controller modes are complex; revise the requested pole set.');
requestedSlow(order)=exp(-rates*c.sampleTime);
[Kslow,precision]=place(Acs,Bcs,requestedSlow); Ksf=Kslow*Vc';
requested=[ordeig(Tc(1:nfc,1:nfc));requestedSlow];
assert(norm(Vc'*d.A-Acs*Vc','fro')<1e-8,'Controller invariant coordinates are inconsistent.');
closedPoles=eig(d.A-d.B*Ksf); remaining=closedPoles; error=zeros(33,1);
[~,sortOrder]=sort(abs(requested),'descend');
for k=sortOrder'
    [error(k),which]=min(abs(remaining-requested(k))); remaining(which)=[];
end
assert(max(error)<c.feedback.poleMatchTolerance && max(abs(closedPoles))<1-1e-8, ...
    'Controller pole placement failed accuracy or stability verification.');

% The dual pole-placement calculation is applied only to slow observer modes.
% A real Schur transformation isolates the already-fast stable invariant space.
[U,T]=schur(d.A','real'); threshold=exp(-c.observer.slowModeCutoffRate*c.sampleTime);
fast=abs(ordeig(T))<=threshold; [U,T]=ordschur(U,T,fast);
nf=sum(fast); V=U(:,nf+1:end); ns=33-nf;
Ao=V'*d.A'*V; Bo=V'*Cn';
requestedObserver=exp(-(c.observer.firstDecayRate+ ...
    c.observer.decayRateSpacing*(0:ns-1))*c.sampleTime);
assert(c.observer.firstDecayRate>0 && c.observer.decayRateSpacing>0);
[Lslow,observerPrecision]=place(Ao,Bo,requestedObserver);
L=(Lslow*V')'; F=d.A-L*Cn;
assert(norm(V'*d.A'-Ao*V','fro')<1e-8,'Observer invariant coordinates are inconsistent.');
assert(max(abs(eig(F)))<1-1e-8,'Observer pole placement is unstable.');
o=struct('Ad',d.A,'Bd',d.B,'C',Cn,'D',Dn,'L',L, ...
    'errorMatrix',F,'poles',eig(F),'continuousA',As,'continuousB',Bs, ...
    'y0',y0,'yScale',yScale,'keep',keep,'S',S,'settings',settings,'sampleTime',c.sampleTime, ...
    'requestedSlowPoles',requestedObserver,'retainedFastPoleCount',nf,'placementPrecision',observerPrecision);
H=zeros(2,33); H(1,find(keep==6))=1; H(2,find(keep==5))=1;
feedback=struct('K',Ksf,'poles',closedPoles,'requestedPoles',requested, ...
    'assignedSlowPoles',requestedSlow,'retainedFastPoleCount',nfc, ...
    'poleMatchError',max(error),'placementPrecision',precision, ...
    'radius',max(abs(closedPoles)),'reference',zeros(2,1), ...
    'maxCorrectionRad',c.feedback.maxCorrectionDeg*pi/180,'sampleTime',c.sampleTime);
assert(isscalar(feedback.maxCorrectionRad) && feedback.maxCorrectionRad>0);
test=c.test; test.pushEnabled=true; test.pushPointBody=zeros(3,1);
r=struct('p',p,'lin',lin,'aux',struct('E',E,'Fw',Fw),'feedback',feedback,'observer',o,'test',test, ...
    'H',H,'sharedConfiguration',c,'sharedVersion',1,'mode','nonlinear');
previousConfiguration=c; previousDesign=r;
fprintf('Shared state-feedback plant: mass %.4g kg; trim %.3g; pole-placement radius %.6g; observer radius %.6g.\n',sum(p.mass),p.trimResidual,feedback.radius,max(abs(eig(F))));
end
function v=column(v,n)
validateattributes(v,{'numeric'},{'real','finite','nonempty'});
if isscalar(v), v=repmat(v,n,1); else, v=v(:); end
assert(numel(v)==n,'Parameter vector has the wrong length.');
end
function H=skew(a)
H=[0 -a(3) a(2);a(3) 0 -a(1);-a(2) a(1) 0];
end

function p=make_parameters(cfg)
r=cfg.robot; p.cfg=cfg; p.g=r.gravity; p.nq=14; p.S=[zeros(6,8);eye(8)];
p.kp=cfg.servo.kp; p.kd=cfg.servo.kd; p.Ta=cfg.servo.timeConstant;
p.tauMax=cfg.servo.maxTorque; p.bjoint=cfg.servo.jointDamping;
p.jointMin=cfg.servo.minCommandDeg*pi/180; p.jointMax=cfg.servo.maxCommandDeg*pi/180;
p.kn=cfg.contact.stiffness; p.cn=cfg.contact.damping;
p.mu=cfg.contact.friction; p.vs=cfg.contact.frictionVelocity;
p.imuOffset=r.imuOffset;
p.parent=[0 1 2 3 4 1 6 7 8]; p.jointIndex=[0 7 8 9 10 11 12 13 14];
p.axis=[zeros(3,1),[1;0;0],[0;1;0],[0;1;0],[0;1;0], ...
                      [1;0;0],[0;1;0],[0;1;0],[0;1;0]];
p.offset=[zeros(3,1),[0;r.hipHalfSpacing;0],[0;0;-r.hipPitchDrop], ...
    [0;0;-r.thighLength],[0;0;-r.shinLength], ...
    [0;-r.hipHalfSpacing;0],[0;0;-r.hipPitchDrop], ...
    [0;0;-r.thighLength],[0;0;-r.shinLength]];
p.mass=[r.massTorso repmat([r.massCarrier r.massThigh r.massShin r.massFoot],1,2)];
legCom=[[0;0;-r.hipPitchDrop/2],[0;0;-r.thighLength/2], ...
    [0;0;-r.shinLength/2],[0;0;-r.soleDepth+r.footThickness/2]];
p.com=[r.torsoCOM legCom legCom];
legDims=[r.carrierDimensions';r.thighWidth r.thighWidth r.thighLength; ...
    r.shinWidth r.shinWidth r.shinLength;r.footLength r.footWidth r.footThickness];
dims=[r.torsoDimensions';legDims;legDims]; p.inertia=zeros(3,3,9);
for b=1:9
    d=dims(b,:); m=p.mass(b);
    p.inertia(:,:,b)=diag(m/12*[d(2)^2+d(3)^2,d(1)^2+d(3)^2,d(1)^2+d(2)^2]);
end
p.footBodies=[5 9];
p.corners=[r.footLength/2*[-1 -1 1 1];r.footWidth/2*[-1 1 -1 1]; ...
    -r.soleDepth*ones(1,4)];
p.qnom=zeros(14,1); p.qnom(7:14)=r.standingJointDeg*pi/180;
% Geometry-derived initial height/roll/pitch, then full force-balance trim.
% Ensure the left sole starts flat even after the nominal pitch angles change.
j=p.qnom(7:10); p.qnom(5)=-sum(j(2:4)); p.qnom(6)=-j(1);
[~,~,K]=md_rigid_body_math(p.qnom,zeros(14,1),p);
heights=zeros(8,1); k=0;
for foot=p.footBodies
    for c=1:4, k=k+1; pos=K(foot).p+K(foot).R*p.corners(:,c); heights(k)=pos(3); end
end
sag=sum(p.mass)*abs(p.g(3))/(8*p.kn);
p.qnom(3)=-mean(heights)-sag;
end

%% ================= DISTURBANCES AND COMMANDS ============================
function p=trim_standing(p)
% Hold nominal leg angles; solve torso Z/pitch/roll for zero base force/moment.
% Symmetry makes X/Y force and yaw moment zero at zero velocity.
q=p.qnom; active=[3 5 6];
for k=1:60
    r=static_residual(q,p); rr=r(active);
    if norm(rr,inf)<1e-8, break; end
    H=zeros(3); epsq=1e-6;
    for j=1:3
        e=zeros(14,1); e(active(j))=epsq;
        rp=static_residual(q+e,p); rm=static_residual(q-e,p);
        H(:,j)=(rp(active)-rm(active))/(2*epsq);
    end
    if rcond(H)<1e-12, error('Trim Jacobian singular: inspect initial foot contact.'); end
    step=-H\rr; accepted=false;
    for a=2.^(-(0:15))
        qt=q; qt(active)=q(active)+a*step;
        rt=static_residual(qt,p);
        if norm(rt(active))<norm(rr), q=qt; accepted=true; break; end
    end
    if ~accepted, error('Standing trim did not converge.'); end
end
[r,h,Qc,fn]=static_residual(q,p);
p.trimResidual=norm(r(1:6),inf); p.trimNormal=fn;
if p.trimResidual>1e-6 || min(fn)<0.01
    error('No suitable all-corners standing equilibrium; revise parameters/contact.');
end
tau=h(7:14)-Qc(7:14);
u=q(7:14)+tau./p.kp;
if any(abs(tau)>=p.tauMax) || any(u<=p.jointMin) || any(u>=p.jointMax)
    error('Standing trim exceeds servo limits.');
end
p.xeq=[q;zeros(14,1);tau]; p.ueq=u;
dx=dynamics(p.xeq,p.ueq,zeros(6,1),p);
if norm(dx,inf)>1e-5, error('Equilibrium dynamics residual too large.'); end
end

function [r,h,Qc,fn]=static_residual(q,p)
[M,h,K]=md_rigid_body_math(q,zeros(14,1),p); %#ok<ASGLU>
[Qc,fn]=contact(K,zeros(14,1),p); r=Qc-h;
end


function [dx,a]=dynamics(x,u,w,p,n)
if nargin<5, n=zero_servo_noise(); end
x=x(:); u=u(:); w=w(:);
assert(numel(x)==36 && numel(u)==8 && numel(w)==6,'Expected x(36), u(8), wrench(6).');
q=x(1:14); v=x(15:28); tau=x(29:36);
[M,h,K]=md_rigid_body_math(q,v,p); [Qc,normal,points]=contact(K,v,p);
Qw=K(1).Jv'*w(1:3)+K(1).Jw'*w(4:6);
uApplied=min(max(u+n.command,p.jointMin),p.jointMax);
targetRaw=p.kp.*(uApplied-q(7:14)-n.feedbackPosition) ...
          -p.kd.*(v(7:14)+n.feedbackVelocity)+n.torque;
target=min(max(targetRaw,-p.tauMax),p.tauMax);
Qd=[zeros(6,1);p.bjoint.*v(7:14)];
acceleration=M\(p.S*tau+Qc+Qw-h-Qd);
dx=[v;acceleration;(target-tau)./p.Ta];
if nargout>1
    B=K(1); r=B.R*p.imuOffset; J=B.Jv-skew(r)*B.Jw;
    aBias=B.a+cross(B.alpha,r)+cross(B.w,cross(B.w,r));
    imu=[B.R'*B.w;B.R'*(J*acceleration+aBias-p.g)];
    measured=[imu;q(7:14)];
    if p.cfg.measurements.includeJointVelocity, measured=[measured;v(7:14)]; end
    com=zeros(3,1);
    for b=1:9, com=com+p.mass(b)*(K(b).p+K(b).R*p.com(:,b)); end
    com=com/sum(p.mass); cop=nan(3,1);
    if sum(normal)>1e-9, cop=[points(1:2,:)*normal/sum(normal);0]; end
    a=struct('yIdeal',measured,'imu',imu,'projectedGravityIdeal',B.R'*[0;0;-1], ...
        'acceleration',acceleration, ...
        'uApplied',uApplied,'targetTorque',target,'normalForces',normal, ...
        'CoM',com,'CoP',cop,'torqueSaturated',abs(targetRaw)>=p.tauMax, ...
        'commandLimited',abs(u+n.command-uApplied)>1e-12);
end
end

function [Qc,normal,points]=contact(K,v,p)
Qc=zeros(14,1); normal=zeros(8,1); points=zeros(3,8); i=0;
for foot=p.footBodies
    B=K(foot);
    for j=1:4
        i=i+1; r=B.R*p.corners(:,j); pos=B.p+r;
        J=B.Jv-skew(r)*B.Jw; vel=J*v; fn=0;
        if pos(3)<0, fn=max(0,-p.kn*pos(3)-p.cn*vel(3)); end
        ft=-p.mu*fn*vel(1:2)/sqrt(vel(1:2)'*vel(1:2)+p.vs^2);
        Qc=Qc+J'*[ft;fn]; normal(i)=fn; points(:,i)=pos;
    end
end
end

function n=zero_servo_noise()
n=struct('command',zeros(8,1),'feedbackPosition',zeros(8,1), ...
    'feedbackVelocity',zeros(8,1),'torque',zeros(8,1));
end



