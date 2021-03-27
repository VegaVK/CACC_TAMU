% Load Engine and Brake Maps:
load('BrkEngMaps.mat');
load('Validate1NoEco.mat');
% load('Validate1Eco.mat');
% load('ValidateRandom.mat');
% load('Valid2020Jun_bag2.mat');

SampleTime=0.05;
InitVel=VehSpeed.Data(1);
% RunTime=min([BrkTrqCmd.Time(end),ThrCmd.Time(end),VehAccel.Time(end),VehSpeed.Time(end)]);
RunTime=90;
clear out;
out=sim('MKZ_Model');

figure; hold on; grid on;
% title('Speed Comparision');
plot(out.VelSim.Time,squeeze(out.VelSim.Data));
plot(VehSpeed.Time,VehSpeed.Data);
legend('Simulation','Collected Data','location','nw');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
axis([0 90 -1 25]);
figure; hold on; grid on;
% title('Accel Comparision');
plot(out.AccelSim.Time,squeeze(out.AccelSim.Data));
plot(VehAccel.Time,VehAccel.Data);
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
legend('Simulation','Experimental');
axis([0 90 -5 5]);
%% Misc Plots 1 - for paper
figure; hold on; grid on;
subplot(2,1,1);
hold on; grid on;
% title('Accel Comparision');
plot(out.AccelSim.Time,squeeze(out.AccelSim.Data));
plot(VehAccel.Time,VehAccel.Data);
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
legend('Simulation','Collected Data');
axis([0 90 -5 5]);
subplot(2,1,2);
grid on; hold on;
plot(out.VelSim.Time,squeeze(out.VelSim.Data));
plot(VehSpeed.Time,VehSpeed.Data);
legend('Simulation','Experimental','location','nw');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
axis([0 90 -1 25]);
%% Misc Plots 2 - for paper
figure; hold on; grid on;
xlabel('Time (s)');
yyaxis left
ylabel({'Brake Input';'(Unitless)'});
plot(BrkTrqCmd);

yyaxis right
plot(ThrCmd);
axis([0 90 0.15 0.6]);
xlabel('Time (s)');
ylabel({'Throttle Input';'(Unitless)'});
