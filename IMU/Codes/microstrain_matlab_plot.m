% analyze_walking_trial.m
%
% Loads a walking_trial_*.csv (t, roll_deg, pitch_deg, yaw_deg,
% sensor_ang_vel_deg_s, derived_ang_vel_deg_s, phase_var), zeros the
% timestamp so it starts at 0s, and plots pitch, angular velocity
% (sensor vs derived), phase portrait, and phase variable over time.
%
% Usage:
%   analyze_walking_trial('walking_trial_20260710_145719.csv')

function analyze_walking_trial(csv_file)

if nargin < 1
    csv_file = 'walking_trial_20260713_150826.csv';
end

T = readtable(csv_file);

t = T.t - T.t(1);   % zero the timestamp - raw t is a Unix timestamp
pitch = T.pitch_deg;
sensor_rate = T.sensor_ang_vel_deg_s;
derived_rate = T.derived_ang_vel_deg_s;
phase = T.phase_var;

%% Pitch over time
figure('Name', 'pitch over time');
plot(t, pitch, 'b-');
xlabel('Time (s)');
ylabel('Pitch (deg)');
title('Thigh angle over session');
grid on;

%% Angular velocity: sensor vs derived
figure('Name', 'Angular velocity: sensor vs derived');
plot(t, sensor_rate, 'r-', 'DisplayName', 'Sensor (estAngularRateY)');
hold on;
plot(t, derived_rate, 'g-', 'DisplayName', 'Derived (d(pitch)/dt, filtered)');
hold off;
xlabel('Time (s)');
ylabel('Angular velocity (deg/s)');
title('Angular velocity: sensor channel vs derived');
legend('show');
grid on;

%% Phase portrait
figure('Name', 'Phase portrait');
plot(pitch, derived_rate, '.', 'MarkerSize', 4);
xlabel('Pitch (deg)');
ylabel('Derived angular velocity (deg/s)');
title('Phase portrait (pitch vs derived ang. vel.)');
axis equal;
grid on;

%% Phase variable over time
figure('Name', 'Phase variable over time');
plot(t, phase, 'b-', 'MarkerSize', 4);
xlabel('Time (s)');
ylabel('Phase variable (0-1)');
title('Phase variable - should look like a repeating sawtooth');
ylim([0 1]);
grid on;

end
