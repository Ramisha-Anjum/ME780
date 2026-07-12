% Bicycle-model data already stored in the table "results"

figure;
plot(results.Time_s, results.YawRate_radps, ...
    'LineWidth', 1.4);
hold on;
plot(out.vdb_r.Time, squeeze(out.vdb_r.Data), '--', ...
    'LineWidth', 1.4);
grid on;
xlabel('Time [s]');
ylabel('Yaw rate [rad/s]');
legend('Analytical bicycle model', ...
       'Vehicle Body 3DOF');

figure;
plot(results.Time_s, results.LateralAcceleration_mps2, ...
    'LineWidth', 1.4);
hold on;
plot(out.vdb_ay.Time, squeeze(out.vdb_ay.Data), '--', ...
    'LineWidth', 1.4);
grid on;
xlabel('Time [s]');
ylabel('Lateral acceleration [m/s^2]');
legend('Analytical bicycle model', ...
       'Vehicle Body 3DOF');

figure;
plot(results.Time_s, rad2deg(results.Sideslip_rad), ...
    'LineWidth', 1.4);
hold on;
plot(out.vdb_beta.Time, rad2deg(squeeze(out.vdb_beta.Data)), '--', ...
    'LineWidth', 1.4);
grid on;
xlabel('Time [s]');
ylabel('Sideslip angle [deg]');
legend('Analytical bicycle model', ...
       'Vehicle Body 3DOF');

figure;
plot(results.X_m, results.Y_m, ...
    'LineWidth', 1.4);
hold on;
plot(squeeze(out.vdb_X.Data), squeeze(out.vdb_Y.Data), '--', ...
    'LineWidth', 1.4);
grid on;
axis equal;
xlabel('Longitudinal position, X [m]');
ylabel('Lateral position, Y [m]');
legend('Analytical bicycle model', ...
       'Vehicle Body 3DOF');