%% ME 780 assignment 3 part 2 simulation

M = 1000;   % kg
Iz = 2000;  % kg m^2
a = 1.2;    % m
b = 1.4;    % m
L = a + b;  % m
U = 20;     % m/s

Cf = 50000; % N/rad
Cr = 50000; % N/rad

% convert steering angle to radians
delta0 = deg2rad(2);

% Bicycle-model state-space matrices
A = [-(Cf + Cr)/(M*U), ...
     -(U + (a*Cf - b*Cr)/(M*U));
     -(a*Cf - b*Cr)/(Iz*U), ...
     -(a^2*Cf + b^2*Cr)/(Iz*U)];

B = [Cf/M;
     a*Cf/Iz];

% States: z = [v; r; psi; X; Y]
model = @(t,z) [A(1,1)*z(1) + A(1,2)*z(2) + B(1)*delta0;
                A(2,1)*z(1) + A(2,2)*z(2) + B(2)*delta0;
                z(2);
                U*cos(z(3)) - z(1)*sin(z(3));
                U*sin(z(3)) + z(1)*cos(z(3))];

% Initial conditions and simulation
z0 = zeros(5,1);
tspan = 0:0.01:5;

[t,z] = ode45(model,tspan,z0);

v   = z(:,1);
r   = z(:,2);
psi = z(:,3);
X   = z(:,4);
Y   = z(:,5);

v_dot = A(1,1)*v + A(1,2)*r + B(1)*delta0;
ay = v_dot + U*r;

beta = -atan(v/U);
alpha_f = delta0 - (v + a*r)/U;
alpha_r = -(v - b*r)/U;

% Plot results
figure;

subplot(3,2,1)
plot(t,rad2deg(r),'LineWidth',1.5)
grid on
xlabel('Time [s]')
ylabel('Yaw rate [deg/s]')
title('Yaw Rate')

subplot(3,2,2)
plot(t,v,'LineWidth',1.5)
grid on
xlabel('Time [s]')
ylabel('Lateral velocity [m/s]')
title('Lateral Velocity')

subplot(3,2,3)
plot(t,ay,'LineWidth',1.5)
grid on
xlabel('Time [s]')
ylabel('Lateral acceleration [m/s^2]')
title('Lateral Acceleration')

subplot(3,2,4)
plot(X,Y,'LineWidth',1.5)
grid on
axis equal
xlabel('X [m]')
ylabel('Y [m]')
title('Vehicle Path')

subplot(3,2,5)
plot(t,rad2deg(beta),'LineWidth',1.5)
grid on
xlabel('Time [s]')
ylabel('Sideslip angle [deg]')
title('Vehicle Sideslip Angle')

subplot(3,2,6)
plot(t,rad2deg(alpha_f),'LineWidth',1.5)
hold on
plot(t,rad2deg(alpha_r),'LineWidth',1.5)
grid on
xlabel('Time [s]')
ylabel('Slip angle [deg]')
title('Tire Slip Angles')
legend('Front','Rear','Location','best')

sgtitle('Bicycle-Model Step-Steer Response')