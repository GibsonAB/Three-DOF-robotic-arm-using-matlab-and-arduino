clc; clear; close all;

% Arm parameters
L1 = 1.6; L2 = 1; L3 = 0.9;

% Target points
xt = [0, 0, 1];
yt = [-2, -1, 2];
zt = [1, 2, 0];

% Pitch angle (in radians)
pitch = deg2rad(45);

% Serial Communication
sp = serialport("COM10", 115200);
configureTerminator(sp, "LF");  % Arduino sends println with LF by default
flush(sp);                      % Clear old boot data
disp("Waiting for Arduino...");

% Wait for 'READY' from Arduino
ready = false;
timeout = 10;  % seconds
tic;
while toc < timeout
    if sp.NumBytesAvailable > 0
        line = readline(sp);
        disp("Arduino: " + line);
        if contains(line, "READY", 'IgnoreCase', true)
            ready = true;
            break;
        end
    end
end

if ~ready
    error("Arduino did not respond with READY in time.");
end

disp("Arduino is ready. Starting control...");
% Then send your angles
% Initial joint angles (start from home position)
theta1_prev = 0;
theta2_prev = 0;
theta3_prev = 0;

% Animation frames per move
nFrames = 100;

figure;

for i = 1:length(xt)
    % Target position
    x = xt(i); y = yt(i); z = zt(i);

    % Base rotation
    theta1 = atan2(y, x);

    % Project into arm plane (after base rotation)
    r = sqrt(x^2 + y^2);
    x_plane = r - L3 * cos(pitch);
    z_plane = z - L3 * sin(pitch);

    % Solve 2-link planar IK
    D = (x_plane^2 + z_plane^2 - L1^2 - L2^2) / (2 * L1 * L2);
    if abs(D) > 1
        error('Target is unreachable in 3D');
    end

    theta3 = atan2(sqrt(1 - D^2), D);  % Elbow down
    k1 = L1 + L2 * cos(theta3);
    k2 = L2 * sin(theta3);
    theta2 = atan2(z_plane, x_plane) - atan2(k2, k1);

    % Animate motion from previous angles to new angles
    for t = linspace(0, 1, nFrames)
        s = 0.5 - 0.5 * cos(pi * t);  % Smooth interpolation
        t1 = (1 - s) * theta1_prev + s * theta1;
        t2 = (1 - s) * theta2_prev + s * theta2;
        t3 = (1 - s) * theta3_prev + s * theta3;

        % Joint positions
        j1 = [0, 0, 0];
        j2 = j1 + [L1 * cos(t1) * cos(t2), L1 * sin(t1) * cos(t2), L1 * sin(t2)];
        j3 = j2 + [L2 * cos(t1) * cos(t2 + t3), L2 * sin(t1) * cos(t2 + t3), L2 * sin(t2 + t3)];
        eff = j3 + [L3 * cos(t1) * cos(pitch), L3 * sin(t1) * cos(pitch), L3 * sin(pitch)];

        % Plot
        clf;
        plot3([j1(1), j2(1), j3(1), eff(1)], ...
              [j1(2), j2(2), j3(2), eff(2)], ...
              [j1(3), j2(3), j3(3), eff(3)], ...
              '-o', 'LineWidth', 4, 'MarkerSize', 6, 'Color', 'k');
        grid on;
        axis equal;
        xlim([-4, 4]); ylim([-4, 4]); zlim([-1, 4]);
        xlabel('X'); ylabel('Y'); zlabel('Z');
        title(['Moving to Target ', num2str(i)]);
        drawnow;
        angles_deg = rad2deg([t1; t2; t3]);
        msg = sprintf('%.2f,%.2f,%.2f\n', angles_deg(1), angles_deg(2), angles_deg(3));
       % flush(sp);
        write(sp, msg, "string");
        pause(0.01);
    end

    % Update previous angles for next move
    theta1_prev = theta1;
    theta2_prev = theta2;
    theta3_prev = theta3;

    % Optional pause at each target
    pause(1.5);
end