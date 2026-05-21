function matlab_force_torque_receiver()
    % MATLAB script to receive and visualize force/torque data from Python
    % This script creates a TCP server to receive data from the Python robot monitor
    
    % Configuration
    PORT = 12345;
    BUFFER_SIZE = 4096;
    
    % Initialize data storage
    master_data = struct('timestamps', [], 'forces', [], 'torques', [], 'joints', []);
    slave_data = struct('timestamps', [], 'forces', [], 'torques', [], 'joints', []);
    
    % Create figure for real-time plotting
    fig = figure('Name', 'Robot Force/Torque Monitor', 'Position', [100, 100, 1200, 800]);
    
    % Create subplots
    subplot(2, 2, 1);
    h_master_force = plot(NaN, NaN, 'b-', 'LineWidth', 2);
    title('Master Arm - Force Magnitude');
    xlabel('Time (s)');
    ylabel('Force (N)');
    grid on;
    
    subplot(2, 2, 2);
    h_slave_force = plot(NaN, NaN, 'r-', 'LineWidth', 2);
    title('Slave Arm - Force Magnitude');
    xlabel('Time (s)');
    ylabel('Force (N)');
    grid on;
    
    subplot(2, 2, 3);
    h_master_torque = plot(NaN, NaN, 'b-', 'LineWidth', 2);
    title('Master Arm - Torque Magnitude');
    xlabel('Time (s)');
    ylabel('Torque (Nm)');
    grid on;
    
    subplot(2, 2, 4);
    h_slave_torque = plot(NaN, NaN, 'r-', 'LineWidth', 2);
    title('Slave Arm - Torque Magnitude');
    xlabel('Time (s)');
    ylabel('Torque (Nm)');
    grid on;
    
    % Create text display for current values
    annotation('textbox', [0.02, 0.95, 0.3, 0.05], 'String', 'Waiting for data...', ...
        'FontSize', 12, 'FontWeight', 'bold', 'EdgeColor', 'none');
    
    % Create TCP server
    try
        server = tcpserver('localhost', PORT);
        fprintf('✓ MATLAB server started on port %d\n', PORT);
        fprintf('Waiting for Python connection...\n');
        
        % Set timeout for accepting connections
        server.Timeout = 10;
        
        % Main data reception loop
        while true
            try
                % Accept connection from Python
                client = accept(server);
                fprintf('✓ Connected to Python client\n');
                
                % Set client timeout
                client.Timeout = 1;
                
                % Data reception loop
                while isvalid(client)
                    try
                        % Read data from Python
                        data = readline(client);
                        
                        if ~isempty(data)
                            % Parse JSON data
                            json_data = jsondecode(data);
                            
                            % Extract data
                            timestamp = json_data.timestamp;
                            master_force = json_data.master.force;
                            master_torque = json_data.master.torque;
                            master_joints = json_data.master.joints;
                            slave_force = json_data.slave.force;
                            slave_torque = json_data.slave.torque;
                            slave_joints = json_data.slave.joints;
                            
                            % Store data
                            master_data.timestamps = [master_data.timestamps, timestamp];
                            master_data.forces = [master_data.forces, master_force];
                            master_data.torques = [master_data.torques, master_torque];
                            master_data.joints = [master_data.joints; master_joints];
                            
                            slave_data.timestamps = [slave_data.timestamps, timestamp];
                            slave_data.forces = [slave_data.forces, slave_force];
                            slave_data.torques = [slave_data.torques, slave_torque];
                            slave_data.joints = [slave_data.joints; slave_joints];
                            
                            % Update plots
                            update_plots(h_master_force, h_slave_force, h_master_torque, h_slave_torque, ...
                                       master_data, slave_data);
                            
                            % Update status text
                            status_text = sprintf('Time: %.1fs | Master: %.2fN, %.2fNm | Slave: %.2fN, %.2fNm', ...
                                timestamp, master_force, master_torque, slave_force, slave_torque);
                            annotation('textbox', [0.02, 0.95, 0.8, 0.05], 'String', status_text, ...
                                'FontSize', 12, 'FontWeight', 'bold', 'EdgeColor', 'none');
                            
                            % Force figure update
                            drawnow;
                        end
                        
                    catch ME
                        if contains(ME.message, 'timeout')
                            % Timeout is normal, continue
                            continue;
                        else
                            fprintf('Error reading data: %s\n', ME.message);
                            break;
                        end
                    end
                end
                
                % Close client connection
                clear client;
                fprintf('Client disconnected\n');
                
            catch ME
                if contains(ME.message, 'timeout')
                    fprintf('No connection received. Retrying...\n');
                    continue;
                else
                    fprintf('Error in connection: %s\n', ME.message);
                    break;
                end
            end
        end
        
    catch ME
        fprintf('Error starting server: %s\n', ME.message);
        fprintf('Make sure no other application is using port %d\n', PORT);
    end
    
    % Cleanup
    if exist('server', 'var') && isvalid(server)
        clear server;
    end
    
    fprintf('Server stopped\n');
end

function update_plots(h_master_force, h_slave_force, h_master_torque, h_slave_torque, master_data, slave_data)
    % Update the real-time plots with new data
    
    % Update force plots
    if ~isempty(master_data.timestamps)
        set(h_master_force, 'XData', master_data.timestamps, 'YData', master_data.forces);
        xlim(h_master_force.Parent, [max(0, master_data.timestamps(end)-10), master_data.timestamps(end)]);
    end
    
    if ~isempty(slave_data.timestamps)
        set(h_slave_force, 'XData', slave_data.timestamps, 'YData', slave_data.forces);
        xlim(h_slave_force.Parent, [max(0, slave_data.timestamps(end)-10), slave_data.timestamps(end)]);
    end
    
    % Update torque plots
    if ~isempty(master_data.timestamps)
        set(h_master_torque, 'XData', master_data.timestamps, 'YData', master_data.torques);
        xlim(h_master_torque.Parent, [max(0, master_data.timestamps(end)-10), master_data.timestamps(end)]);
    end
    
    if ~isempty(slave_data.timestamps)
        set(h_slave_torque, 'XData', slave_data.timestamps, 'YData', slave_data.torques);
        xlim(h_slave_torque.Parent, [max(0, slave_data.timestamps(end)-10), slave_data.timestamps(end)]);
    end
    
    % Auto-scale Y axes
    if ~isempty(master_data.forces)
        ylim(h_master_force.Parent, [0, max(master_data.forces) * 1.1]);
    end
    if ~isempty(slave_data.forces)
        ylim(h_slave_force.Parent, [0, max(slave_data.forces) * 1.1]);
    end
    if ~isempty(master_data.torques)
        ylim(h_master_torque.Parent, [0, max(master_data.torques) * 1.1]);
    end
    if ~isempty(slave_data.torques)
        ylim(h_slave_torque.Parent, [0, max(slave_data.torques) * 1.1]);
    end
end 