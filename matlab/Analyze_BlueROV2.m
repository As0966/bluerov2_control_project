%% BlueROV2 4-DOF Controller Performance Analysis Suite
% Project: Mathematical Modeling and Controller Design for a BlueROV2-Inspired 4-DOF Underwater Vehicle
% Description: This script processes simulation data from Gazebo, calculates
% performance metrics, and generates high-fidelity visualizations for 
% various control architectures (MRAC, Backstepping, FL, etc.)

clear; clc; close all;

%% 1. Setup and Path Configuration
results_dir = '../results/';
figures_dir = './figures/';
if ~exist(figures_dir, 'dir'), mkdir(figures_dir); end

% Define Controllers and Trajectories
controllers = {'hasten_mrac', 'hasten_adaptive_backstepping', 'hasten_fl_input', 'hasten_fl_io', 'hasten_inn', 'hasten_state_feedback', 'observer_linear'};
trajectories = {'circle', 'figure8', 'spline'};

% Color Palette for Plots (Professional)
colors = [0 0.4470 0.7410; 0.8500 0.3250 0.0980; 0.9290 0.6940 0.1250; ...
          0.4940 0.1840 0.5560; 0.4660 0.6740 0.1880; 0.3010 0.7450 0.9330; ...
          0.6350 0.0780 0.1840];

%% 2. Trajectory Comparison (3D)
for t = 1:length(trajectories)
    traj_name = trajectories{t};
    figure('Name', ['3D Trajectory - ', traj_name], 'Units', 'normalized', 'Position', [0.1 0.1 0.6 0.7]);
    hold on; grid on; box on;
    
    legend_entries = {};
    first_plot = true;
    
    for c = 1:length(controllers)
        file_name = sprintf('%s%s_%s_gazebo.csv', results_dir, controllers{c}, traj_name);
        
        if exist(file_name, 'file')
            data = readtable(file_name);
            
            % Plot Reference (only once)
            if first_plot
                plot3(data.x_d, data.y_d, data.z_d, 'k--', 'LineWidth', 2);
                legend_entries{end+1} = 'Desired';
                first_plot = false;
            end
            
            % Plot Actual
            plot3(data.x, data.y, data.z, 'Color', colors(c,:), 'LineWidth', 1.5);
            legend_entries{end+1} = strrep(controllers{c}, '_', ' ');
        end
    end
    
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    title(['3D Trajectory Tracking: ', traj_name]);
    legend(legend_entries, 'Location', 'bestoutside');
    view(45, 30);
    saveas(gcf, fullfile(figures_dir, ['3d_traj_', traj_name, '.png']));
end

%% 3. Error Analysis and Metrics
% Loading the pre-computed metrics summary if available
summary_file = [results_dir, 'metrics_summary.csv'];
if exist(summary_file, 'file')
    metrics = readtable(summary_file);
    fprintf('Displaying Performance Metrics Summary:\n');
    disp(metrics);
    
    % Generate Bar Chart for RMSE
    figure('Name', 'RMSE Comparison', 'Units', 'normalized', 'Position', [0.2 0.2 0.5 0.5]);
    % Filter or pivot data for plotting if necessary
    % (Assuming metrics has columns: Controller, Trajectory, RMSE_pos, etc.)
    % This part can be customized based on the actual metrics_summary.csv structure
end

%% 4. Time Series: State Tracking (Example for first trajectory)
traj_name = trajectories{1};
figure('Name', ['Tracking Detail - ', traj_name], 'Units', 'normalized', 'Position', [0.05 0.05 0.9 0.8]);
tiledlayout(2,2, 'TileSpacing', 'Compact');

% Select a controller to highlight (e.g., MRAC)
file_name = [results_dir, 'hasten_mrac_', traj_name, '_gazebo.csv'];
if exist(file_name, 'file')
    data = readtable(file_name);
    
    % X tracking
    nexttile;
    plot(data.t, data.x_d, 'k--', data.t, data.x, 'b', 'LineWidth', 1.5);
    ylabel('X [m]'); grid on; title('X-Position');
    
    % Y tracking
    nexttile;
    plot(data.t, data.y_d, 'k--', data.t, data.y, 'r', 'LineWidth', 1.5);
    ylabel('Y [m]'); grid on; title('Y-Position');
    
    % Z tracking
    nexttile;
    plot(data.t, data.z_d, 'k--', data.t, data.z, 'g', 'LineWidth', 1.5);
    ylabel('Z [m]'); grid on; title('Z-Position');
    
    % Psi tracking
    nexttile;
    plot(data.t, rad2deg(data.psi_d), 'k--', data.t, rad2deg(data.psi), 'm', 'LineWidth', 1.5);
    ylabel('Psi [deg]'); grid on; title('Heading (Psi)');
    
    lgd = legend('Desired', 'Actual');
    lgd.Layout.Tile = 'East';
end

%% 5. Control Effort (Thrust Analysis)
figure('Name', 'Control Inputs (Thrust)', 'Units', 'normalized', 'Position', [0.1 0.1 0.8 0.6]);
tiledlayout(2,2);

nexttile; plot(data.t, data.tau_u); title('\tau_u (Surge)'); grid on; ylabel('Force [N]');
nexttile; plot(data.t, data.tau_v); title('\tau_v (Sway)'); grid on; ylabel('Force [N]');
nexttile; plot(data.t, data.tau_w); title('\tau_w (Heave)'); grid on; ylabel('Force [N]');
nexttile; plot(data.t, data.tau_r); title('\tau_r (Yaw)'); grid on; ylabel('Torque [Nm]');

fprintf('Analysis Complete. Figures saved to %s\n', figures_dir);
