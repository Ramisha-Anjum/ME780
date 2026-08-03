%% ME780 Assignment 3 - Part 3(a)
% Desired yaw-rate calculation and PI yaw-moment controller design

clear;
clc;
close all;

%% 1. Vehicle data
ms  = 1590;       % sprung mass [kg]
muf = 102.2;      % front unsprung mass [kg]
mur = 115;        % rear unsprung mass [kg]
mt  = 28;         % mass of one tire [kg]

g  = 9.81;
L  = 2.950;
as = 1.180;
bs = 1.770;

Iz = 2687.1;      % reduced-model yaw inertia [kg m^2]

%% 2. Total mass and static axle-supported masses
m = ms + muf + mur + 4*mt;

msf = ms*bs/L;
msr = ms*as/L;

mf = msf + muf + 2*mt;
mr = msr + mur + 2*mt;

Fzf = mf*g/2;
Fzr = mr*g/2;

%% 3. Approximate total-vehicle CG location
a = L*mr/m;
b = L-a;

%% 4. Read CarSim lateral tire-force table
M = readmatrix('TireFy.csv');

% Remove fully blank columns
M(:,all(~isfinite(M),1)) = [];

Fz_grid   = M(1,2:end);
alpha_deg = M(2:end,1);
Fy_map    = M(2:end,2:end);

valid = isfinite(alpha_deg);
alpha_deg = alpha_deg(valid);
Fy_map = Fy_map(valid,:);

%% 5. Interpolate tire forces at front and rear static loads
Fy_front = interp1(Fz_grid,Fy_map.',Fzf,'linear').';
Fy_rear  = interp1(Fz_grid,Fy_map.',Fzr,'linear').';

%% 6. Fit cornering stiffness over the small-angle region
fitMask = alpha_deg <= 1.5;

alpha_fit = deg2rad(alpha_deg(fitMask));
Fy_front_small = Fy_front(fitMask);
Fy_rear_small  = Fy_rear(fitMask);

% Least-squares slope constrained through the origin
Caf_tire = ...
    (alpha_fit.'*Fy_front_small)/(alpha_fit.'*alpha_fit);

Car_tire = ...
    (alpha_fit.'*Fy_rear_small)/(alpha_fit.'*alpha_fit);

Cf = 2*Caf_tire;
Cr = 2*Car_tire;

%% 7. Understeer coefficient and yaw-rate gain
Kus = (m/L)*(b/Cf-a/Cr);

Vx = 60/3.6;      % design speed [m/s]

yawGain = Vx/(L+Kus*Vx^2);

rLimit055 = 0.55*g/Vx;
rLimit090 = 0.90*g/Vx;

%% 8. Bicycle model with yaw-moment input
A = [-(Cf+Cr)/(m*Vx), ...
     -1-(a*Cf-b*Cr)/(m*Vx^2);

     -(a*Cf-b*Cr)/Iz, ...
     -(a^2*Cf+b^2*Cr)/(Iz*Vx)];

Bdelta = [Cf/(m*Vx);
          a*Cf/Iz];

BM = [0;
      1/Iz];

Cyaw = [0 1];

G_Mz = ss(A,BM,Cyaw,0);

%% 9. PI controller design
wc = 4;           % selected crossover frequency [rad/s]
wz = 2;           % selected PI-zero frequency [rad/s]

Gjw = squeeze(freqresp(G_Mz,wc));

Kp = 1/(abs(Gjw)*sqrt(1+(wz/wc)^2));
Ki = Kp*wz;

C_PI = tf([Kp Ki],[1 0]);

T_yaw = feedback(C_PI*G_Mz,1);
T_moment = feedback(C_PI,G_Mz);

%% 10. Small closed-loop verification test
% A small 0.05 rad/s reference step is used only to verify
% controller stability within a reasonable moment range.
t = (0:0.001:8)';
r_ref = 0.05*ones(size(t));

r_response = lsim(T_yaw,r_ref,t);
Mz_command = lsim(T_moment,r_ref,t);

%% 11. Display numerical results
fprintf('\nVEHICLE PARAMETERS\n');
fprintf('Total vehicle mass       = %.2f kg\n',m);
fprintf('Front tire static load   = %.2f N\n',Fzf);
fprintf('Rear tire static load    = %.2f N\n',Fzr);
fprintf('CG to front axle, a      = %.4f m\n',a);
fprintf('CG to rear axle, b       = %.4f m\n',b);

fprintf('\nCORNERING STIFFNESSES\n');
fprintf('Front tire stiffness     = %.2f N/rad\n',Caf_tire);
fprintf('Rear tire stiffness      = %.2f N/rad\n',Car_tire);
fprintf('Front axle stiffness Cf  = %.2f N/rad\n',Cf);
fprintf('Rear axle stiffness Cr   = %.2f N/rad\n',Cr);

fprintf('\nDESIRED YAW-RATE MODEL\n');
fprintf('Understeer coefficient   = %.8e s^2/m\n',Kus);
fprintf('Yaw-rate gain at 60 kph  = %.5f 1/s\n',yawGain);
fprintf('Yaw limit, mu=0.55       = %.5f rad/s\n',rLimit055);
fprintf('Yaw limit, mu=0.90       = %.5f rad/s\n',rLimit090);

fprintf('\nPI CONTROLLER\n');
fprintf('Kp = %.2f\n',Kp);
fprintf('Ki = %.2f\n',Ki);

disp('Open-loop vehicle poles:')
disp(eig(A))

disp('Closed-loop poles:')
disp(pole(T_yaw))

%% 12. Cornering-stiffness plots
figure;
plot(alpha_deg,Fy_front,'o');
hold on;
plot(rad2deg(alpha_fit),Caf_tire*alpha_fit,'LineWidth',1.5);
grid on;
xlabel('Slip angle [deg]');
ylabel('Lateral force [N]');
legend('Interpolated CarSim data','Linear fit','Location','best');
title('Front Tire Cornering-Stiffness Calculation');

figure;
plot(alpha_deg,Fy_rear,'o');
hold on;
plot(rad2deg(alpha_fit),Car_tire*alpha_fit,'LineWidth',1.5);
grid on;
xlabel('Slip angle [deg]');
ylabel('Lateral force [N]');
legend('Interpolated CarSim data','Linear fit','Location','best');
title('Rear Tire Cornering-Stiffness Calculation');

%% 13. Controller verification plots
figure;
plot(t,r_ref,'--','LineWidth',1.4);
hold on;
plot(t,r_response,'LineWidth',1.4);
grid on;
xlabel('Time [s]');
ylabel('Yaw rate [rad/s]');
legend('Desired yaw rate','Controlled yaw rate','Location','best');
title('PI Yaw-Rate Controller Verification');

figure;
plot(t,Mz_command,'LineWidth',1.4);
hold on;
yline(2800,'--');
yline(-2800,'--');
grid on;
xlabel('Time [s]');
ylabel('\Delta M_z [N m]');
title('Corrective Yaw-Moment Command');
legend('\Delta M_z','Upper limit','Lower limit','Location','best');