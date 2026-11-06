function [M,h,K]=md_rigid_body_math(q,v,p)
% NINE-BODY GEOMETRY AND INERTIA - no integration or controller inside.
% Inputs: q(14) in m/rad, v(14) in m/s or rad/s, mechanical parameters p.
% Outputs:
%   M (14x14): mass/inertia matrix relating acceleration to force.
%   h (14x1): gravity and velocity-dependent generalized forces.
%   K (9 bodies): world position/orientation, velocity and Jacobian data.
% The source equations are unchanged; comments explain the three stages below.
% Several visible Simulink geometry blocks request the identical calculation
% at one solver evaluation. Reuse only an EXACT match of all inputs.
persistent lastQ lastV lastP lastM lastH lastK
if isequal(q,lastQ) && isequal(v,lastV) && isequaln(p,lastP)
    M=lastM; h=lastH; K=lastK; return
end
[M,h,K]=mechanics(q,v,p);
lastQ=q; lastV=v; lastP=p; lastM=M; lastH=h; lastK=K;
end
function [M,h,K]=mechanics(q,v,p)
% World-frame recursive kinematics, Jacobian mass assembly, and inverse-
% dynamics bias at qddot=0. No numerical derivatives inside the ODE.
n=14; I=eye(3); z=[0;0;1]; y=[0;1;0]; ex=[1;0;0];
% 1. TORSO: form yaw-pitch-roll rotation and angular velocity.
% Euler coordinate rates are not generally identical to body gyro readings.
Rz=rotation(z,q(4)); Ry=rotation(y,q(5)); Rx=rotation(ex,q(6));
s4=z; s5=Rz*y; s6=Rz*Ry*ex;
w4=s4*v(4); w5=w4+s5*v(5);
K(1)=struct('p',q(1:3),'R',Rz*Ry*Rx,'w',w5+s6*v(6), ...
    'a',zeros(3,1),'alpha',cross(w4,s5)*v(5)+cross(w5,s6)*v(6), ...
    'Jv',[I zeros(3,11)],'Jw',[zeros(3,3) s4 s5 s6 zeros(3,8)]);
% 2. LEGS: walk down each serial chain from the torso to each foot.
% Each body inherits its parent's motion and adds its own joint rotation.
% Jv/Jw map all 14 coordinate rates into linear/angular world velocity.
for b=2:9
    P=K(p.parent(b)); idx=p.jointIndex(b);
    r=P.R*p.offset(:,b); s=P.R*p.axis(:,b);
    Jw=P.Jw; Jw(:,idx)=Jw(:,idx)+s;
    K(b)=struct('p',P.p+r,'R',P.R*rotation(p.axis(:,b),q(idx)), ...
        'w',P.w+s*v(idx), ...
        'a',P.a+cross(P.alpha,r)+cross(P.w,cross(P.w,r)), ...
        'alpha',P.alpha+cross(P.w,s)*v(idx), ...
        'Jv',P.Jv-skew(r)*P.Jw,'Jw',Jw);
end
M=zeros(n); h=zeros(n,1);
% 3. ASSEMBLY: add each body's translational and rotational contributions.
% Mbody = mass*Jcom'*Jcom + Jomega'*Iworld*Jomega.
% hbody includes gravity, acceleration bias, and gyroscopic effects.
for b=1:9
    B=K(b); rc=B.R*p.com(:,b);
    Jc=B.Jv-skew(rc)*B.Jw;
    ac=B.a+cross(B.alpha,rc)+cross(B.w,cross(B.w,rc));
    Iw=B.R*p.inertia(:,:,b)*B.R';
    M=M+p.mass(b)*(Jc'*Jc)+B.Jw'*Iw*B.Jw;
    h=h+Jc'*(p.mass(b)*(ac-p.g))+ ...
        B.Jw'*(Iw*B.alpha+cross(B.w,Iw*B.w));
end
M=(M+M')/2;
% Symmetrize to remove floating-point asymmetry, not to change the mechanics.
end


function R=rotation(axis,a)
H=skew(axis); R=eye(3)+sin(a)*H+(1-cos(a))*(H*H);
end
function H=skew(a)
H=[0 -a(3) a(2);a(3) 0 -a(1);-a(2) a(1) 0];
end



