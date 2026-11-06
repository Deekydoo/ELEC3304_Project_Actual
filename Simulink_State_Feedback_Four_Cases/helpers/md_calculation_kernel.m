function md_calculation_kernel(b)
% Stateless geometry kernels only. Integration, forces, state feedback and observer are blocks.
% Dialog parameters: numeric/mechanical parameters p, operation name.
%
% HOW TO READ THIS FILE
% Simulink calls outputs() whenever it needs the current geometry. This file
% stores no physical state and does not advance time. The visible Integrator
% and Unit Delay blocks elsewhere perform all state updates.
%
% Every operation begins with q(14), v(14): torso position and Euler angles,
% eight leg joint angles, and the corresponding coordinate rates.
% md_rigid_body_math returns the mass matrix M, bias forces h, and body geometry K.
% Look for the operation selected by the block's second dialog parameter:
%   mass         -> M(:) followed by h (210 numbers)
%   points       -> eight sets of position, velocity and point Jacobian
%   wrench       -> external push mapped into 14 generalized forces
%   imu_geometry -> rotation, angular velocity and IMU lever-arm geometry
b.NumDialogPrms=2; op=b.DialogPrm(2).Data;
switch op
    case 'mass', ni=28; no=210;
    case 'points', ni=28; no=384;
    case 'wrench', ni=34; no=14;
    case 'imu', ni=42; no=14;
    case 'imu_geometry', ni=28; no=57;
    otherwise, error('Unknown calculation');
end
b.NumInputPorts=1; b.NumOutputPorts=1;
b.SetPreCompInpPortInfoToDynamic; b.SetPreCompOutPortInfoToDynamic;
b.InputPort(1).Dimensions=ni; b.InputPort(1).DirectFeedthrough=true;
b.OutputPort(1).Dimensions=no; b.SampleTimes=[0 0];
b.RegBlockMethod('Outputs',@outputs);
end
function outputs(b)
p=b.DialogPrm(1).Data; op=b.DialogPrm(2).Data; u=b.InputPort(1).Data;
q=u(1:14); v=u(15:28); [M,h,K]=md_rigid_body_math(q,v,p);
switch op
    case 'mass'
        % M couples all body and joint accelerations. h contains gravity and
        % velocity-dependent forces. The force-balance folder subtracts h,
        % adds motors/contact/push, then solves M*acceleration = net force.
        y=[M(:);h];
    case 'points'
        % Locate the four corners of each foot in world coordinates.
        % J maps generalized velocity to the corner's linear velocity.
        % This operation supplies geometry only: spring/damper/friction
        % forces are calculated by the visible sole-point blocks.
        y=zeros(384,1); i=0;
        for foot=p.footBodies
            B=K(foot);
            for j=1:4
                i=i+1; rr=B.R*p.corners(:,j); pos=B.p+rr;
                J=B.Jv-skew(rr)*B.Jw; vel=J*v;
                y((i-1)*48+(1:48))=[pos;vel;J(:)];
            end
        end
    case 'wrench'
        % Virtual-work mapping: Q = Jlinear'*force + Jangular'*moment.
        % The six supplied wrench components use WORLD axes.
        y=K(1).Jv'*u(29:31)+K(1).Jw'*u(32:34);
    case 'imu'
        % Compatibility operation combining the complete ideal sensor law.
        % Teaching models instead use imu_geometry and visible matrix blocks.
        B=K(1); rr=B.R*p.imuOffset; J=B.Jv-skew(rr)*B.Jw;
        bias=B.a+cross(B.alpha,rr)+cross(B.w,cross(B.w,rr));
        y=[B.R'*B.w;B.R'*(J*u(29:42)+bias-p.g);q(7:14)];
    case 'imu_geometry'
        % R: rotation from trunk to world; R' therefore maps world to trunk.
        % The offset rr creates angular/centripetal acceleration at the IMU.
        % bias excludes generalized acceleration; the visible J*acceleration
        % path supplies it, then gravity is subtracted to form specific force.
        B=K(1); rr=B.R*p.imuOffset; J=B.Jv-skew(rr)*B.Jw;
        bias=B.a+cross(B.alpha,rr)+cross(B.w,cross(B.w,rr));
        y=[B.R(:);B.w;J(:);bias];
end
b.OutputPort(1).Data=y;
end
function H=skew(a)
H=[0 -a(3) a(2);a(3) 0 -a(1);-a(2) a(1) 0];
end

