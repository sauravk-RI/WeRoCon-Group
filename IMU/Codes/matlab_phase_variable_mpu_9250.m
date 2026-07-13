% plot_phase_var.m
% Verifies that phase_var moves from 0 to 1 correctly over time
% Load CSV output from imu_thigh_angle_phase.py and plot phase_var vs time

clear; clc; close all;

%% ── Load data ────────────────────────────────────────────────────────
data = readtable('imu_phase_15june_7th.csv');

t         = data.time_s;
phase_var = data.phase_var;
pitch     = data.pitch_centered_deg;
gyro_filt = data.gyro_pitch_filt;

%% ── Plot 1: phase_var vs time ────────────────────────────────────────
figure('Name', 'Phase Variable vs Time', 'NumberTitle', 'off');

subplot(3,1,1);
plot(t, phase_var, 'b', 'LineWidth', 1.2);
yline(0, 'k--', 'LineWidth', 0.8);
yline(1, 'k--', 'LineWidth', 0.8);
ylim([-0.1 1.1]);
xlabel('Time (s)');
ylabel('\phi (0 \rightarrow 1)');
title('Phase Variable over Time');
grid on;

subplot(3,1,2);
plot(t, pitch, 'r', 'LineWidth', 1.0);
xlabel('Time (s)');
ylabel('Pitch centered (deg)');
title('pitch\_centered\_deg  [A = 20 deg]');
yline(20,  'r--', '+A');
yline(-20, 'r--', '-A');
grid on;

subplot(3,1,3);
plot(t, gyro_filt, 'm', 'LineWidth', 1.0);
xlabel('Time (s)');
ylabel('Gyro filtered (deg/s)');
title('gyro\_pitch\_filt  [B = 100 deg/s]');
yline(100,  'm--', '+B');
yline(-100, 'm--', '-B');
grid on;

sgtitle('Phase Variable Verification  (A=20 deg, B=100 deg/s)');

%% ── Plot 2: phase portrait with phase colour ─────────────────────────
figure('Name', 'Phase Portrait coloured by phi', 'NumberTitle', 'off');
scatter(pitch, gyro_filt, 8, phase_var, 'filled');
colormap(hsv);
cb = colorbar;
cb.Label.String = '\phi (0 \rightarrow 1)';
xlabel('pitch\_centered\_deg');
ylabel('gyro\_pitch\_filt (deg/s)');
title('Phase Portrait — colour = phase variable \phi');
xline(0, 'k--'); yline(0, 'k--');
grid on;

%% ── Print summary stats ──────────────────────────────────────────────
fprintf('\n── Phase variable stats ──\n');
fprintf('  Min  : %.4f\n', min(phase_var));
fprintf('  Max  : %.4f\n', max(phase_var));
fprintf('  Range: %.4f\n', max(phase_var) - min(phase_var));
fprintf('  Mean : %.4f\n', mean(phase_var));
fprintf('\n  If range ≈ 1.0 and plot shows sawtooth → phase is working correctly.\n');
fprintf('  If range is small → A or B needs adjustment (signal not reaching limits).\n\n');
