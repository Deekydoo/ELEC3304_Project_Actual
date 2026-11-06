%%\ MICRODUCK C1: 81-corner open-loop eigenvalue sweep (balance subsystem A_b)
clear; clc;
% Reproduces (report Sec. 3.1, Tables I-II, criterion C1):
%   nominal pole +2.9013; 81-corner range +1.9887 to +5.0605 rad/s;
%   K_d sensitivity: 2.9013 / 2.3573 / 1.9448 rad/s.
% Requires microduck8_plant.m on the MATLAB path. Runs in a few minutes.


% ---- Parameter values (must match Table II, criterion C1, in the report) ----
massScale = [0.95 1.00 1.05];            % scales ALL link masses -> total mass +-5 %
comValues = 0.055 + [-0.008 0 0.008];    % m, upper-body COM height, +-8 mm
kpValues  = [0.386 0.52 0.55];           % Nm/rad, calibrated native-PD sets
TaValues  = [0.02 0.03 0.08];            % s, servo lag (0.03 s = nominal)
bjValues  = [0.041 0.048 0.053]; % Nms/rad, paired with kpValues (same sets)

tol  = 1e-6;                             % positive-real-part threshold
Tsel = setdiff(1:36,[1 2 4]);            % drop X, Y, yaw -> 33-state A_b

massFields = {'massTorso','massCarrier','massThigh','massShin','massFoot'};

N = 81;
results = nan(N,8);
failed = false(N,1);
failureMessage = strings(N,1);
row = 0;

fprintf('Running 81-corner MicroDuck eigenvalue sweep...\n'); %[output:573ac1fe]

for im = 1:3 %[output:group:33da29b4]
    for ic = 1:3
        for ik = 1:3
            for it = 1:3
                row = row + 1;

                cfg = microduck8_plant('config'); %[output:5a96c59f] %[output:726e9e95] %[output:397f30b8] %[output:519c9f1e] %[output:6a3dbdf7] %[output:03a11bc5] %[output:9ed9e3ed] %[output:8fe2e794] %[output:5255cf51] %[output:0c8b100c] %[output:23244a72] %[output:41b2a3a3] %[output:16c20f68] %[output:4a86f4b6] %[output:1550e1ec] %[output:3c7f9225] %[output:5c2eb6f5] %[output:2c9c92f0] %[output:2f285761] %[output:46a94d01] %[output:838a6ac4] %[output:8ecfec0c] %[output:12bd82b5] %[output:57977cdd] %[output:74b990c0] %[output:04d923c2] %[output:260ccfe6] %[output:31947cf6] %[output:6e70c2a1] %[output:89e267af] %[output:4e2b2562] %[output:5a605bd3] %[output:7c7b158e] %[output:661de521] %[output:87fa34cb] %[output:4b21c969] %[output:37d9b172] %[output:012b6831] %[output:49f439c9] %[output:99cc779c] %[output:74686713] %[output:86cf64a5] %[output:6734dded] %[output:0ce78301] %[output:293e0ec7] %[output:041e2848] %[output:506d7407] %[output:761dbc2a] %[output:6de30120] %[output:88092a89] %[output:460dc8b5] %[output:6f599c09] %[output:3794635f] %[output:1bb82710] %[output:5f9c924d] %[output:599d49f6] %[output:47a1063a] %[output:4757ece7] %[output:6e113d84] %[output:50d13d8b] %[output:3f4429b4] %[output:63109716] %[output:25d5f1f8] %[output:817686c0] %[output:14727f34] %[output:770d7d2d] %[output:11cb67e9] %[output:4e1c62e2] %[output:5ccd87b9] %[output:116acc12] %[output:362f8fa1] %[output:9413bde4] %[output:14b7c057] %[output:16ba5e12] %[output:48b9e753] %[output:6b9d54bc] %[output:23c24c00] %[output:3f03c9ea] %[output:2034d0ac] %[output:916eda24] %[output:7b0aa4bc]
                cfg.sim.plot = false;
                base = cfg.robot;

                % Apply parameter combination
                for f = 1:numel(massFields)
                    cfg.robot.(massFields{f}) = base.(massFields{f}) * massScale(im);
                end
                cfg.robot.torsoCOM(3)  = comValues(ic);
                cfg.servo.kp           = kpValues(ik);
                cfg.servo.timeConstant = TaValues(it);
                cfg.servo.jointDamping = bjValues(ik);

                try
                    lin   = microduck8_plant('linearize',cfg); %[output:4dcbe97d] %[output:8926fb7e] %[output:8dc6c130] %[output:3207340f] %[output:8236b88e] %[output:07f975fc] %[output:354fc904] %[output:146985a6] %[output:078a6b4f] %[output:63bafc67] %[output:126cf14f] %[output:4c50f935] %[output:64b05d1a] %[output:4d5ea79d] %[output:793b8278] %[output:204a681a] %[output:5fe5d817] %[output:3c953fb6] %[output:1f1b3e3a] %[output:383cf599] %[output:7104de5e] %[output:644384ae] %[output:30e63270] %[output:5c9b135a] %[output:10ddf21f] %[output:0a0a7263] %[output:4e3c0497] %[output:5c49f9f4] %[output:45014db6] %[output:04e91ba5] %[output:07d24384] %[output:78648b31] %[output:8f8fbe72] %[output:6c35e172] %[output:06f2e947] %[output:01d6c6d2] %[output:2869d102] %[output:52d6ec04] %[output:062fdaf2] %[output:4060547b] %[output:86fae555] %[output:84e22f92] %[output:98d102da] %[output:5808159b] %[output:1c59b957] %[output:6bdd6b17] %[output:15fc4d7b] %[output:8e1683b2] %[output:6633e38d] %[output:77025685] %[output:8475ae0a] %[output:5f9b19c5] %[output:0ed2a997] %[output:980efbce] %[output:6c1d82ea] %[output:12964bf3] %[output:7cfd5c76] %[output:67455022] %[output:717ff19d] %[output:2511b94b] %[output:3ef23bd2] %[output:81146926] %[output:91d4bbe5] %[output:5062f20d] %[output:97b090a6] %[output:4f0003d3] %[output:0070cd69] %[output:202c33bf] %[output:38db7aa8] %[output:431bd177] %[output:036fcaa5] %[output:46e1404e] %[output:000006f6] %[output:79b0870b] %[output:971efe80] %[output:1af5aa69] %[output:2f3f916a] %[output:8ffb017d] %[output:71689092] %[output:04fe03cf] %[output:287b9452] %[output:84dac7d9] %[output:23743faa] %[output:2f8d0a7e] %[output:3bd31f4e] %[output:3a1f3974] %[output:2edacb7a] %[output:01c7c0ca] %[output:3e4f45fc] %[output:11644cb9] %[output:004dffdb] %[output:7ae21dff] %[output:4ca6cccb] %[output:8421ddf3] %[output:77806ba3] %[output:122348cf] %[output:694b6805] %[output:6864115a] %[output:8d4332b0] %[output:888b0584] %[output:42b7183f] %[output:00ecd570] %[output:3e6d8b15] %[output:36decb7b] %[output:699a5da7] %[output:0cfe4b15] %[output:7db76b10] %[output:077731d3] %[output:2e274f7c] %[output:31456110] %[output:7abceea0] %[output:3d3e9b53] %[output:0e07edd1] %[output:2df727e7] %[output:24d86fb2] %[output:6b1bfa0b] %[output:27897c5f] %[output:302de06c] %[output:95be8964] %[output:301605b9] %[output:0497e6bb] %[output:58239779] %[output:704ff402] %[output:16ca8771] %[output:94854d87] %[output:71dc10ff] %[output:0e9ac17c] %[output:49805a8a] %[output:704f3d98] %[output:21f54b91] %[output:7443edc0] %[output:9dac9487] %[output:3b71f4d6] %[output:9f271e3d] %[output:41288ac7] %[output:91b5cbd8] %[output:41f30e7f] %[output:5009e7f9] %[output:691d705f] %[output:50e1f7af] %[output:3b8dee87] %[output:64e50094] %[output:19ee5138] %[output:783dcd8f] %[output:1ac2e4e6] %[output:60d10cc5] %[output:536b82f0] %[output:791bae29] %[output:1f58f5b7] %[output:32ff8fe4] %[output:47ae75f0] %[output:40a10461] %[output:0021da17] %[output:74310d0f] %[output:6cf85dbb] %[output:97dd0fb6] %[output:06a046ec] %[output:10e0cf7c] %[output:3d8b07dc] %[output:15970851] %[output:6e2f0ff4] %[output:9125f03d]
                    poles = eig(lin.A(Tsel,Tsel));

                    unstable  = real(poles) > tol;
                    nUnstable = sum(unstable);
                    maxReal   = max(real(poles));

                    if nUnstable > 0
                        up = poles(unstable);
                        [~,idx] = max(real(up));
                        dominantPole = up(idx);
                    else
                        dominantPole = complex(NaN,NaN);
                    end

                    results(row,:) = [massScale(im), comValues(ic), ...
                        kpValues(ik), bjValues(ik), TaValues(it), nUnstable, maxReal, ...
                        real(dominantPole), imag(dominantPole)];

                catch ME
                    failed(row) = true;
                    failureMessage(row) = string(ME.message);
                    fprintf('Corner %d FAILED: %s\n',row,ME.message);
                end
            end
        end
    end
end %[output:group:33da29b4]

%%\ Display results
T = array2table(results,'VariableNames',{ ...
    'MassScale','COMHeight_m','Kp', 'bj','Ta_s','UnstablePoleCount', ...
    'MaxRealPole','DominantPoleReal','DominantPoleImag'});
T.Failed = failed;
disp(T); %[output:38ab33e4]

%%\ Summary
valid  = ~failed;
nValid = sum(valid);

fprintf('\n========== C1 OPEN-LOOP SWEEP (A_b, 33 states) ==========\n'); %[output:44856c7c]
fprintf('Total corners: %d\n',N); %[output:9c2c047e]
fprintf('Successful corners: %d\n',nValid); %[output:180388d9]
fprintf('Failed corners: %d\n',sum(failed)); %[output:33100326]

if nValid > 0 %[output:group:9374e9f1]
    counts  = T.UnstablePoleCount(valid);
    maxReal = T.MaxRealPole(valid);

    fprintf('Minimum unstable pole count: %d\n',min(counts)); %[output:2e37c68c]
    fprintf('Maximum unstable pole count: %d\n',max(counts)); %[output:45c6650d]
    fprintf('Smallest max-real-part: %.4f rad/s\n',min(maxReal)); %[output:27c819f4]
    fprintf('Largest  max-real-part: %.4f rad/s\n',max(maxReal)); %[output:9b8a4a31]
    fprintf('All corners have exactly one unstable pole: %s\n',string(all(counts == 1))); %[output:5d64d145]

    % Nominal corner (scale 1.00, COM 55 mm, Kp 0.55, Ta 0.03)
    nomRow = find(T.MassScale==1 & abs(T.COMHeight_m-0.055)<1e-9 & ...
                  T.Kp==0.55 & T.Ta_s==0.03);
    fprintf('\nNominal corner:\n'); disp(T(nomRow,:)); %[output:1e341145]

    % Fastest unstable corner
    validRows = find(valid);
    [~,idx] = max(maxReal);
    fprintf('Fastest unstable corner:\n'); disp(T(validRows(idx),:)); %[output:3065b0d0]

    writetable(T,'microduck8_C1_sweep_results.csv');
    fprintf('Results saved to microduck8_C1_sweep_results.csv\n'); %[output:0620e66b]
end %[output:group:9374e9f1]

%%\ K_d sensitivity (one-at-a-time, nominal corner)
for kd = [0 0.0265 0.053] %[output:group:8d0150b6]
    cfg = microduck8_plant('config'); cfg.sim.plot = false; %[output:0e8e0116] %[output:56cad733] %[output:46771539]
    cfg.servo.kd = kd;
    lin = microduck8_plant('linearize',cfg); %[output:6fc8c9b2] %[output:7dc2b020] %[output:1cadf568] %[output:729dd762] %[output:0845a6d0] %[output:333d4fd7]
    pk  = eig(lin.A(Tsel,Tsel));
    fprintf('Kd=%.4f: %d unstable, max real %.4f\n', kd, sum(real(pk)>tol), max(real(pk))); %[output:8d73f25c] %[output:251a9ec2] %[output:8b9595ae]
end %[output:group:8d0150b6]

if any(failed)
    fprintf('\nFailed configurations:\n'); disp(failureMessage(failed));
end
fprintf('=========================================================\n'); %[output:11c5d3b0]

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright"}
%---
%[output:573ac1fe]
%   data: {"dataType":"text","outputData":{"text":"Running 81-corner MicroDuck eigenvalue sweep...\n","truncated":false}}
%---
%[output:5a96c59f]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:4dcbe97d]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:8926fb7e]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:726e9e95]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:8dc6c130]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:3207340f]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:397f30b8]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:8236b88e]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:07f975fc]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:519c9f1e]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:354fc904]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:146985a6]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:6a3dbdf7]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:078a6b4f]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:63bafc67]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:03a11bc5]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:126cf14f]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:4c50f935]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:9ed9e3ed]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:64b05d1a]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:4d5ea79d]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:8fe2e794]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:793b8278]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:204a681a]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:5255cf51]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:5fe5d817]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:3c953fb6]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:0c8b100c]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:1f1b3e3a]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:383cf599]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:23244a72]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:7104de5e]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:644384ae]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:41b2a3a3]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:30e63270]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:5c9b135a]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:16c20f68]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:10ddf21f]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:0a0a7263]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:4a86f4b6]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:4e3c0497]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:5c49f9f4]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:1550e1ec]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:45014db6]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:04e91ba5]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:3c7f9225]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:07d24384]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:78648b31]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:5c2eb6f5]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:8f8fbe72]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:6c35e172]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:2c9c92f0]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:06f2e947]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:01d6c6d2]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:2f285761]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:2869d102]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:52d6ec04]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:46a94d01]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:062fdaf2]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:4060547b]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:838a6ac4]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:86fae555]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:84e22f92]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:8ecfec0c]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:98d102da]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:5808159b]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:12bd82b5]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:1c59b957]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:6bdd6b17]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:57977cdd]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:15fc4d7b]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:8e1683b2]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:74b990c0]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:6633e38d]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:77025685]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:04d923c2]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:8475ae0a]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:5f9b19c5]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:260ccfe6]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:0ed2a997]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:980efbce]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:31947cf6]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:6c1d82ea]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:12964bf3]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:6e70c2a1]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:7cfd5c76]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:67455022]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:89e267af]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:717ff19d]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:2511b94b]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:4e2b2562]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:3ef23bd2]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:81146926]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:5a605bd3]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:91d4bbe5]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:5062f20d]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:7c7b158e]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:97b090a6]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:4f0003d3]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:661de521]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:0070cd69]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:202c33bf]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:87fa34cb]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:38db7aa8]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:431bd177]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:4b21c969]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:036fcaa5]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:46e1404e]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:37d9b172]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:000006f6]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:79b0870b]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:012b6831]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:971efe80]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:1af5aa69]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:49f439c9]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:2f3f916a]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:8ffb017d]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:99cc779c]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:71689092]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:04fe03cf]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:74686713]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:287b9452]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:84dac7d9]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:86cf64a5]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:23743faa]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:2f8d0a7e]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:6734dded]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:3bd31f4e]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:3a1f3974]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:0ce78301]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:2edacb7a]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:01c7c0ca]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:293e0ec7]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:3e4f45fc]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:11644cb9]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:041e2848]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:004dffdb]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:7ae21dff]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:506d7407]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:4ca6cccb]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:8421ddf3]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:761dbc2a]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:77806ba3]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:122348cf]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:6de30120]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:694b6805]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:6864115a]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:88092a89]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:8d4332b0]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:888b0584]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:460dc8b5]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:42b7183f]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:00ecd570]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:6f599c09]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:3e6d8b15]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:36decb7b]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:3794635f]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:699a5da7]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:0cfe4b15]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:1bb82710]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:7db76b10]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:077731d3]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:5f9c924d]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:2e274f7c]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:31456110]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:599d49f6]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:7abceea0]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:3d3e9b53]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:47a1063a]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:0e07edd1]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:2df727e7]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:4757ece7]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:24d86fb2]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:6b1bfa0b]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:6e113d84]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:27897c5f]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:302de06c]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:50d13d8b]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:95be8964]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:301605b9]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:3f4429b4]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:0497e6bb]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:58239779]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:63109716]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:704ff402]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:16ca8771]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:25d5f1f8]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:94854d87]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:71dc10ff]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:817686c0]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:0e9ac17c]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:49805a8a]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:14727f34]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:704f3d98]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:21f54b91]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:770d7d2d]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:7443edc0]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:9dac9487]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:11cb67e9]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:3b71f4d6]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:9f271e3d]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:4e1c62e2]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:41288ac7]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:91b5cbd8]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:5ccd87b9]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:41f30e7f]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:5009e7f9]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:116acc12]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:691d705f]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:50e1f7af]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:362f8fa1]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:3b8dee87]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:64e50094]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:9413bde4]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:19ee5138]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:783dcd8f]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:14b7c057]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:1ac2e4e6]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:60d10cc5]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:16ba5e12]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:536b82f0]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:791bae29]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:48b9e753]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:1f58f5b7]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:32ff8fe4]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:6b9d54bc]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:47ae75f0]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:40a10461]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:23c24c00]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:0021da17]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:74310d0f]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:3f03c9ea]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:6cf85dbb]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:97dd0fb6]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:2034d0ac]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:06a046ec]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:10e0cf7c]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:916eda24]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:3d8b07dc]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:15970851]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:7b0aa4bc]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:6e2f0ff4]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:9125f03d]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:38ab33e4]
%   data: {"dataType":"text","outputData":{"text":"    <strong>MassScale<\/strong>    <strong>COMHeight_m<\/strong>     <strong>Kp<\/strong>      <strong>Ta_s<\/strong>    <strong>UnstablePoleCount<\/strong>    <strong>MaxRealPole<\/strong>    <strong>DominantPoleReal<\/strong>    <strong>DominantPoleImag<\/strong>    <strong>Failed<\/strong>\n    <strong>_________<\/strong>    <strong>___________<\/strong>    <strong>_____<\/strong>    <strong>____<\/strong>    <strong>_________________<\/strong>    <strong>___________<\/strong>    <strong>________________<\/strong>    <strong>________________<\/strong>    <strong>______<\/strong>\n\n      0.95          0.047       0.386    0.02            1              3.8397            3.8397                0            false \n      0.95          0.047       0.386    0.03            1              3.9458            3.9458                0            false \n      0.95          0.047       0.386    0.08            1              4.3996            4.3996                0            false \n      0.95          0.047        0.52    0.02            1              2.4327            2.4327                0            false \n      0.95          0.047        0.52    0.03            1              2.5582            2.5582                0            false \n      0.95          0.047        0.52    0.08            1              3.1724            3.1724                0            false \n      0.95          0.047        0.55    0.02            1              1.9887            1.9887                0            false \n      0.95          0.047        0.55    0.03            1              2.1072            2.1072                0            false \n      0.95          0.047        0.55    0.08            1              2.7253            2.7253                0            false \n      0.95          0.055       0.386    0.02            1              4.0901            4.0901                0            false \n      0.95          0.055       0.386    0.03            1              4.1887            4.1887                0            false \n      0.95          0.055       0.386    0.08            1              4.6025            4.6025                0            false \n      0.95          0.055        0.52    0.02            1              2.8564            2.8564                0            false \n      0.95          0.055        0.52    0.03            1               2.978             2.978                0            false \n      0.95          0.055        0.52    0.08            1              3.5431            3.5431                0            false \n      0.95          0.055        0.55    0.02            1              2.4635            2.4635                0            false \n      0.95          0.055        0.55    0.03            1              2.5826            2.5826                0            false \n      0.95          0.055        0.55    0.08            1               3.161             3.161                0            false \n      0.95          0.063       0.386    0.02            1              4.2905            4.2905                0            false \n      0.95          0.063       0.386    0.03            1              4.3818            4.3818                0            false \n      0.95          0.063       0.386    0.08            1                4.76              4.76                0            false \n      0.95          0.063        0.52    0.02            1              3.1954            3.1954                0            false \n      0.95          0.063        0.52    0.03            1              3.3107            3.3107                0            false \n      0.95          0.063        0.52    0.08            1              3.8286            3.8286                0            false \n      0.95          0.063        0.55    0.02            1              2.8437            2.8437                0            false \n      0.95          0.063        0.55    0.03            1              2.9589            2.9589                0            false \n      0.95          0.063        0.55    0.08            1              3.4937            3.4937                0            false \n         1          0.047       0.386    0.02            1              4.0621            4.0621                0            false \n         1          0.047       0.386    0.03            1              4.1647            4.1647                0            false \n         1          0.047       0.386    0.08            1              4.5972            4.5972                0            false \n         1          0.047        0.52    0.02            1              2.7676            2.7676                0            false \n         1          0.047        0.52    0.03            1              2.8928            2.8928                0            false \n         1          0.047        0.52    0.08            1              3.4821            3.4821                0            false \n         1          0.047        0.55    0.02            1              2.3563            2.3563                0            false \n         1          0.047        0.55    0.03            1              2.4779            2.4779                0            false \n         1          0.047        0.55    0.08            1              3.0784            3.0784                0            false \n         1          0.055       0.386    0.02            1              4.2899            4.2899                0            false \n         1          0.055       0.386    0.03            1              4.3849            4.3849                0            false \n         1          0.055       0.386    0.08            1              4.7788            4.7788                0            false \n         1          0.055        0.52    0.02            1              3.1488            3.1488                0            false \n         1          0.055        0.52    0.03            1              3.2681            3.2681                0            false \n         1          0.055        0.52    0.08            1              3.8072            3.8072                0            false \n         1          0.055        0.55    0.02            1              2.7827            2.7827                0            false \n         1          0.055        0.55    0.03            1              2.9013            2.9013                0            false \n         1          0.055        0.55    0.08            1              3.4568            3.4568                0            false \n         1          0.063       0.386    0.02            1              4.4712            4.4712                0            false \n         1          0.063       0.386    0.03            1              4.5589            4.5589                0            false \n         1          0.063       0.386    0.08            1              4.9186            4.9186                0            false \n         1          0.063        0.52    0.02            1              3.4542            3.4542                0            false \n         1          0.063        0.52    0.03            1              3.5664            3.5664                0            false \n         1          0.063        0.52    0.08            1              4.0591            4.0591                0            false \n         1          0.063        0.55    0.02            1              3.1253            3.1253                0            false \n         1          0.063        0.55    0.03            1              3.2384            3.2384                0            false \n         1          0.063        0.55    0.08            1              3.7493            3.7493                0            false \n      1.05          0.047       0.386    0.02            1              4.2611            4.2611                0            false \n      1.05          0.047       0.386    0.03            1              4.3603            4.3603                0            false \n      1.05          0.047       0.386    0.08            1               4.773             4.773                0            false \n      1.05          0.047        0.52    0.02            1              3.0605            3.0605                0            false \n      1.05          0.047        0.52    0.03            1              3.1841            3.1841                0            false \n      1.05          0.047        0.52    0.08            1              3.7483            3.7483                0            false \n      1.05          0.047        0.55    0.02            1              2.6762            2.6762                0            false \n      1.05          0.047        0.55    0.03            1              2.7982            2.7982                0            false \n      1.05          0.047        0.55    0.08            1              3.3777            3.3777                0            false \n      1.05          0.055       0.386    0.02            1               4.469             4.469                0            false \n      1.05          0.055       0.386    0.03            1              4.5605            4.5605                0            false \n      1.05          0.055       0.386    0.08            1              4.9361            4.9361                0            false \n      1.05          0.055        0.52    0.02            1              3.4063            3.4063                0            false \n      1.05          0.055        0.52    0.03            1              3.5227            3.5227                0            false \n      1.05          0.055        0.52    0.08            1              4.0371            4.0371                0            false \n      1.05          0.055        0.55    0.02            1              3.0628            3.0628                0            false \n      1.05          0.055        0.55    0.03            1              3.1798            3.1798                0            false \n      1.05          0.055        0.55    0.08            1              3.7122            3.7122                0            false \n      1.05          0.063       0.386    0.02            1              4.6333            4.6333                0            false \n      1.05          0.063       0.386    0.03            1              4.7177            4.7177                0            false \n      1.05          0.063       0.386    0.08            1              5.0605            5.0605                0            false \n      1.05          0.063        0.52    0.02            1              3.6832            3.6832                0            false \n      1.05          0.063        0.52    0.03            1              3.7919            3.7919                0            false \n      1.05          0.063        0.52    0.08            1              4.2614            4.2614                0            false \n      1.05          0.063        0.55    0.02            1              3.3738            3.3738                0            false \n      1.05          0.063        0.55    0.03            1              3.4841            3.4841                0            false \n      1.05          0.063        0.55    0.08            1              3.9723            3.9723                0            false \n\n","truncated":false}}
%---
%[output:44856c7c]
%   data: {"dataType":"text","outputData":{"text":"\n========== C1 OPEN-LOOP SWEEP (A_b, 33 states) ==========\n","truncated":false}}
%---
%[output:9c2c047e]
%   data: {"dataType":"text","outputData":{"text":"Total corners: 81\n","truncated":false}}
%---
%[output:180388d9]
%   data: {"dataType":"text","outputData":{"text":"Successful corners: 81\n","truncated":false}}
%---
%[output:33100326]
%   data: {"dataType":"text","outputData":{"text":"Failed corners: 0\n","truncated":false}}
%---
%[output:2e37c68c]
%   data: {"dataType":"text","outputData":{"text":"Minimum unstable pole count: 1\n","truncated":false}}
%---
%[output:45c6650d]
%   data: {"dataType":"text","outputData":{"text":"Maximum unstable pole count: 1\n","truncated":false}}
%---
%[output:27c819f4]
%   data: {"dataType":"text","outputData":{"text":"Smallest max-real-part: 1.9887 rad\/s\n","truncated":false}}
%---
%[output:9b8a4a31]
%   data: {"dataType":"text","outputData":{"text":"Largest  max-real-part: 5.0605 rad\/s\n","truncated":false}}
%---
%[output:5d64d145]
%   data: {"dataType":"text","outputData":{"text":"All corners have exactly one unstable pole: true\n","truncated":false}}
%---
%[output:1e341145]
%   data: {"dataType":"text","outputData":{"text":"\nNominal corner:\n    <strong>MassScale<\/strong>    <strong>COMHeight_m<\/strong>     <strong>Kp<\/strong>     <strong>Ta_s<\/strong>    <strong>UnstablePoleCount<\/strong>    <strong>MaxRealPole<\/strong>    <strong>DominantPoleReal<\/strong>    <strong>DominantPoleImag<\/strong>    <strong>Failed<\/strong>\n    <strong>_________<\/strong>    <strong>___________<\/strong>    <strong>____<\/strong>    <strong>____<\/strong>    <strong>_________________<\/strong>    <strong>___________<\/strong>    <strong>________________<\/strong>    <strong>________________<\/strong>    <strong>______<\/strong>\n\n        1           0.055       0.55    0.03            1              2.9013            2.9013                0            false \n\n","truncated":false}}
%---
%[output:3065b0d0]
%   data: {"dataType":"text","outputData":{"text":"Fastest unstable corner:\n    <strong>MassScale<\/strong>    <strong>COMHeight_m<\/strong>     <strong>Kp<\/strong>      <strong>Ta_s<\/strong>    <strong>UnstablePoleCount<\/strong>    <strong>MaxRealPole<\/strong>    <strong>DominantPoleReal<\/strong>    <strong>DominantPoleImag<\/strong>    <strong>Failed<\/strong>\n    <strong>_________<\/strong>    <strong>___________<\/strong>    <strong>_____<\/strong>    <strong>____<\/strong>    <strong>_________________<\/strong>    <strong>___________<\/strong>    <strong>________________<\/strong>    <strong>________________<\/strong>    <strong>______<\/strong>\n\n      1.05          0.063       0.386    0.08            1              5.0605            5.0605                0            false \n\n","truncated":false}}
%---
%[output:0620e66b]
%   data: {"dataType":"text","outputData":{"text":"Results saved to microduck8_C1_sweep_results.csv\n","truncated":false}}
%---
%[output:0e8e0116]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:6fc8c9b2]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:7dc2b020]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:8d73f25c]
%   data: {"dataType":"text","outputData":{"text":"Kd=0.0000: 1 unstable, max real 2.9013\n","truncated":false}}
%---
%[output:56cad733]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:1cadf568]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:729dd762]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:251a9ec2]
%   data: {"dataType":"text","outputData":{"text":"Kd=0.0265: 1 unstable, max real 2.3573\n","truncated":false}}
%---
%[output:46771539]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:0845a6d0]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"cfg","value":"             sim: [1×1 struct]\n    disturbances: [1×1 struct]\n         command: [1×1 struct]\n         initial: [1×1 struct]\n            stop: [1×1 struct]\n           robot: [1×1 struct]\n           servo: [1×1 struct]\n"}}
%---
%[output:333d4fd7]
%   data: {"dataType":"text","outputData":{"text":"A: 36x36, B: 36x8, C: 6x36, D: 6x8.\nUse perturbations x-xeq, u-ueq, y-yeq.\n","truncated":false}}
%---
%[output:8b9595ae]
%   data: {"dataType":"text","outputData":{"text":"Kd=0.0530: 1 unstable, max real 1.9448\n","truncated":false}}
%---
%[output:11c5d3b0]
%   data: {"dataType":"text","outputData":{"text":"=========================================================\n","truncated":false}}
%---
