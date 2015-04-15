clear
clc
format long g
clf

% SETTINGS
plot_position = 0; % 0 for velocity
has_acceleration = 1; % if CSV includes acceleration

%while 1
    sampled_data = csvread('trajectory_3.csv',1,0);
    
    %assign columns into seperate variable names - this is made quickly using the
    %header names and find-replace command
    i=1;
    timestamp=sampled_data(:,i);i=i+1;
    
    gantry_pos=sampled_data(:,i);i=i+1;gantry_vel=sampled_data(:,i);i=i+1;
    if (has_acceleration) gantry_acc=sampled_data(:,i);i=i+1; end
    
    joint1_pos=sampled_data(:,i);i=i+1;joint1_vel=sampled_data(:,i);i=i+1;
    if (has_acceleration) joint1_acc=sampled_data(:,i);i=i+1; end

    joint2_pos=sampled_data(:,i);i=i+1;joint2_vel=sampled_data(:,i);i=i+1;
    if (has_acceleration) joint2_acc=sampled_data(:,i);i=i+1; end
    
    joint3_pos=sampled_data(:,i);i=i+1;joint3_vel=sampled_data(:,i);i=i+1;
    if (has_acceleration) joint3_acc=sampled_data(:,i);i=i+1; end
    
    joint4_pos=sampled_data(:,i);i=i+1;joint4_vel=sampled_data(:,i);i=i+1;
    if (has_acceleration) joint4_acc=sampled_data(:,i);i=i+1; end
    
    joint5_pos=sampled_data(:,i);i=i+1;joint5_vel=sampled_data(:,i);i=i+1;
    if (has_acceleration) joint5_acc=sampled_data(:,i);i=i+1; end
    
    joint6_pos=sampled_data(:,i);i=i+1;joint6_vel=sampled_data(:,i);i=i+1;
    if (has_acceleration) joint6_acc=sampled_data(:,i);i=i+1; end
    
    time_start_end = [timestamp(1,1) timestamp(end,1)];
    joint2_upper = 0;
    joint3_lower = -3.92699081699;
    joint3_upper = 0.785398163397;
    
    if (plot_position)
        plot(...
             timestamp', gantry_pos,'r', ...%time_start_end, [-pi, -pi], 'r--', time_start_end, [pi, pi], 'r--',...
             timestamp', joint1_pos,'y', ...%time_start_end, [-pi,-pi], 'yo',time_start_end, [joint2_upper,joint2_upper], 'y--',...
             timestamp', joint2_pos,'m', ...%time_start_end, [joint3_lower,joint3_lower], 'm--',time_start_end, [joint3_upper,joint3_upper], 'm--',...
             timestamp', joint3_pos,'c',...
             timestamp', joint4_pos,'g',...
             timestamp', joint5_pos,...
             timestamp', joint6_pos)

        xlabel('Time')
        ylabel('Position')
        legend('Gantry Pos',...%'Continous Low', 'Continuous High',...
                'Joint 1 Pos',...%'Joint 2 Low', 'Joint 2 High',...
                'Joint 2 Pos',...%'Joint 3 Low', 'Joint 3 High',...
                'Joint 3 Pos',...
                'Joint 4 Pos',...
                'Joint 5 Pos',...
                'Joint 6 Pos',...
                'Location','northwest')

        title('Position Trajectory')
    else
        plot(...
             timestamp', gantry_vel,'r', ...%time_start_end, [-pi, -pi], 'r--', time_start_end, [pi, pi], 'r--',...
             timestamp', joint1_vel,'y', ...%time_start_end, [-pi,-pi], 'yo',time_start_end, [joint2_upper,joint2_upper], 'y--',...
             timestamp', joint2_vel,'m', ...%time_start_end, [joint3_lower,joint3_lower], 'm--',time_start_end, [joint3_upper,joint3_upper], 'm--',...
             timestamp', joint3_vel,'c',...
             timestamp', joint4_vel,'g',...
             timestamp', joint5_vel,...
             timestamp', joint6_vel)

        xlabel('Time')
        ylabel('Velocity')
        legend('Gantry vel',...%'Continous Low', 'Continuous High',...
                'Joint 1 vel',...%'Joint 2 Low', 'Joint 2 High',...
                'Joint 2 vel',...%'Joint 3 Low', 'Joint 3 High',...
                'Joint 3 vel',...
                'Joint 4 vel',...
                'Joint 5 vel',...
                'Joint 6 vel',...
                'Location','northwest')

        title('Velocity Trajectory')
    end
    
    % Stats
    fprintf('Gantry  diff %.6f \n', gantry_pos(end,1) - gantry_pos(1,1))
    fprintf('Joint 1 diff %.6f \n', joint1_pos(end,1) - joint1_pos(1,1))
    fprintf('Joint 2 diff %.6f \n', joint2_pos(end,1) - joint2_pos(1,1))
    fprintf('Joint 3 diff %.6f \n', joint3_pos(end,1) - joint3_pos(1,1))
    fprintf('Joint 4 diff %.6f \n', joint4_pos(end,1) - joint4_pos(1,1))
    fprintf('Joint 5 diff %.6f \n', joint5_pos(end,1) - joint5_pos(1,1))
    fprintf('Joint 6 diff %.6f \n', joint6_pos(end,1) - joint6_pos(1,1))
    
             
    
    
    pause(1)
%end