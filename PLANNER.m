% ------------------------------- Mission Planner for prequalification ----------------------------------%
% Fs for mission planner is 100 times per second
function[] = PLANNER()

global Fs; global r; global imuFs;

gate_z = 1.5; % Gate level in z for case 0 (first finite state)

% Subscriber queue from SLAM
global slam_msg;
Feedback = slam_msg; % feedback of states and variances 

% Subscriber queue from DeepLayer
global end_msg;
EOT = end_msg;

% Subscriber queue from Vertical controller
global vert_msg;
ACTIVE = vert_msg(4,1);

% Subscriber queue from quat_to_rad
global quat_msg;
angles = quat_msg;

% Subscriber queue from imu node
global imu_msg;
imu = imu_msg;

persistent count; % Finite State Counter
if isempty(count)
  count = 0; % If counter not initialized, set to zero
end


persistent Wait_count; % Waiting Counter
if isempty(Wait_count)
  Wait_count = 0; % If counter not initialized, set to zero
end

persistent prev_state; % What was the previous state ??
if isempty(prev_state)
  prev_state = 0; % If previous state not initialized, set to zero
end

persistent sign; 
if isempty(sign)
  sign = 1; % If sign not initialized, set to positive
end

% Finite State Machine
switch count
    case 0 % State 0 in FSM (Descend, and wait till you're in gate level)
        
        msg_height = gate_z; % Construct height message
        Height_publish(msg_height) % Publish Function
        
        
        if(abs(Feedback(10,1)-gate_z)<=0.05 && abs(Feedback(3,1))<=0.005) % if height is okay
            count = 1; % Move on to next finite state
        end
        
    case 1 % Searching for gate & path planning towards gate
        
        % Adjust Height to gate level
        msg_height = gate_z; % Construct height message
        Height_publish(msg_height) % Publish Function
        
        % Check whether our belief in gate position is certain enough??
        % Are we stable enough to make our next move??
        if(Feedback(14,1)<= 10^-2 && Feedback(15,1)<= 10^-2 && Feedback(16,1)<= 10^-2 && ACTIVE == true)
            
              % Plan Your Trajectory & point towards gate
              theta_target = angles(3,1) + atan(Feedback(5,1)/(Feedback(4,1))); % angle AUV needs to rotate
              
              if(abs(angles(3,1)-theta_target)>=(1/180)*pi || abs(imu(4,1)*imuFs)>=0.1) % If angle difference is large
                  
                trajStart = angles(3,1);
                dyaw = (theta_target-trajStart)/(2*Fs);
                N = 2*Fs;
                Traj = trajStart + (0:N-1)*dyaw;
                Traj = [Traj'; theta_target*ones(1*Fs,1)]; % Trajectory to follow (yaw)
                x0 = [angles(3,1);imu(4,1)*imuFs;0]; % initial yaw conditions
                T_sim = 3; % future time
                limit_m = 26; % Moment limit
                mode = true;
                TRAJYAW(Traj,x0,T_sim,limit_m,mode); % Call trajyaw service
                
                Traj = zeros(3*Fs,1); % Trajectory to follow (velocity)
                x0 = [0; imu(1,1)]; % initial velocity conditions
                limit_t = 14; % Force limit
                mode = ['v';'X'];
                TRAJV(Traj,x0,T_sim,limit_t,mode); % Call trajv service
                
                % Go to the waiting state
                Wait_count = 1;
                count = 10;
                
              else
                  
                  % Plan your trajectory in and out of gate 
                  % Penetrate then move for 6 m with constant speed 0.5 m/s
                  % If you're coming from state 4 move for 2.5 m instead**
                  % Then stop after two seconds
                  T_sim = ((round(Feedback(4,1)) + 6) / 0.5) + 2; % Total planning time
                  if(prev_state == 4)
                     T_sim = ((round(Feedback(4,1)) + 2.5) / 0.5) + 2; % Total planning time
                  end
                  Traj = ones(T_sim*Fs,1) * 0.5; 
                  Traj = [Traj; zeros(2*Fs,1)]; % Velocity Trajectory
                  x0 = [0;imu(1,1)];
                  limit_t = 42.42;
                  mode = ['v';'X'];
                  TRAJV(Traj,x0,T_sim,limit_t,mode); % Call trajv service
                  
                  Traj = zeros(T_sim*Fs,1); 
                  Traj(1:1:(round(Feedback(4,1))/0.5)*Fs) = angles(3,1);
                  Traj((round(Feedback(4,1))/0.5)*Fs+1:1:T_sim*Fs) = 0; % Fill yaw trajectory 
                  if(prev_state == 4)
                     Traj((round(Feedback(4,1))/0.5)*Fs+1:1:T_sim*Fs) = -sign * pi; % Fill yaw trajectory 
                  end
                  x0 = [angles(3,1);imu(4,1)*imuFs;0];
                  limit_m = 60*r;
                  mode = true;
                  TRAJYAW(Traj,x0,T_sim,limit_m,mode); % Call trajyaw service
                  count = 2; % Move on to next state
                  if(prev_state == 4)
                     count = 5;
                  end
                  
              end
   
              
        elseif((Feedback(14,1)> 10^-2 || Feedback(15,1)> 10^-2 || Feedback(16,1)> 10^-2) && ACTIVE == true)
                trajStart = angles(3,1);
                dyaw = (pi/2)/(1*Fs);
                N = 1*Fs;
                Traj = trajStart + (0:N-1)*dyaw;
                Traj = [Traj'; (angles(3,1) + pi/2)*ones(0.5*Fs,1)]; % Trajectory to follow (yaw)
                x0 = [angles(3,1);imu(4,1)*imuFs;0]; % initial yaw conditions
                T_sim = 1.5; % future time
                limit_m = 26; % Moment limit
                mode = true;
                TRAJYAW(Traj,x0,T_sim,limit_m,mode); % Call trajyaw service
                
                Traj = zeros(1.5*Fs,1); % Trajectory to follow (velocity)
                x0 = [0; imu(1,1)]; % initial velocity conditions
                limit_t = 14; % Force limit
                mode = ['v';'X'];
                TRAJV(Traj,x0,T_sim,limit_t,mode); % Call trajv service
                
                % Go to the waiting state
                Wait_count = 1;
                count = 10;
        end
    
    case 10 % Waiting state
        
        % Adjust Height to gate level
        msg_height = gate_z; % Construct height message
        Height_publish(msg_height) % Publish Function
        
        % Exit if trajectory following is done
        if(EOT == true || ACTIVE == false)
            count = Wait_count; % return to the state you want
        end
        
    case 2 % Wait till you finish trajectory without diturbance
        
        % Adjust Height to gate level
        msg_height = gate_z; % Construct height message
        Height_publish(msg_height) % Publish Function
        
        % If disturbance happens, check whether you see gate or not
        % If you see gate, get back to state 1
        % If you don't see gate, move on to case 3
        % If no disturbance happens and trajectory is fulfilled, go to 3
        if(Feedback(14,1)<= 10^-2 && Feedback(15,1)<= 10^-2 && Feedback(16,1)<= 10^-2 && ACTIVE == false)
            count = 1;
        end
        if(((Feedback(14,1)>= 10^-2 || Feedback(15,1)>= 10^-2 || Feedback(16,1)>= 10^-2) && ACTIVE == false) || EOT == true)
            count = 3;
        end
        
        % Exit if trajectory following is done
        if(EOT == true)
            count = 3; % get to the next state
            prev_state = 2; % record current state as past state
        end
        
    case 3 % Search for khazoo2 & plan your trajectory around it
        
        % Adjust Height to gate level
        msg_height = gate_z; % Construct height message
        Height_publish(msg_height) % Publish Function
        
        % Check whether our belief in khazoo2 position is certain enough??
        % Are we stable enough to make our next move??
        if(Feedback(17,1)<= 10^-2 && Feedback(18,1)<= 10^-2 && Feedback(19,1)<= 10^-2 && ACTIVE == true)
            
              % Plan Your Trajectory & point towards khazoo2
              theta_target = angles(3,1) + atan(Feedback(8,1)/(Feedback(7,1))); % angle AUV needs to rotate
              
              if(abs(angles(3,1)-theta_target)>=(1/180)*pi || abs(imu(4,1)*imuFs)>=0.1) % If angle difference is large
                  
                trajStart = angles(3,1);
                dyaw = (theta_target-trajStart)/(2*Fs);
                N = 2*Fs;
                Traj = trajStart + (0:N-1)*dyaw;
                Traj = [Traj'; theta_target*ones(1*Fs,1)]; % Trajectory to follow (yaw)
                x0 = [angles(3,1);imu(4,1)*imuFs;0]; % initial yaw conditions
                T_sim = 3; % future time
                limit_m = 26; % Moment limit
                mode = true;
                TRAJYAW(Traj,x0,T_sim,limit_m,mode); % Call trajyaw service
                
                Traj = zeros(3*Fs,1); % Trajectory to follow (velocity)
                x0 = [0; imu(1,1)]; % initial velocity conditions
                limit_t = 14; % Force limit
                mode = ['v';'X'];
                TRAJV(Traj,x0,T_sim,limit_t,mode); % Call trajv service
                
                % Go to the waiting state
                Wait_count = 3;
                count = 10;
                
              else
                  
                  % Plan your trajectory around the khazoo2 
                  % move forward then revolve in a circular motion
                  % head towards gate for 6 meters
                  % Then stop after two seconds
                  
                  %first: move towards target till you're within 1 m radius
                  trajStart = 0;
                  dv = ((Feedback(7,1) - 1 - 0.5)/4.5) / (1*Fs);
                  N = 1*Fs;
                  Traj_vel1 = (trajStart + (0:N-1)*dv)';
                  Traj_vel1 = [Traj_vel1 ; ((Feedback(7,1) - 1 - 0.5)/4.5) * ones(4.5*Fs,1) ; flip(Traj_vel1)];
                  Traj_yaw1 = angles(3,1) * ones(5*Fs,1); 
                  T_sim1 = 5;
                  
                  %second: rotate by 90 degrees
                  trajStart = angles(3,1);
                  if(prev_state == 2)
                      sign = -1 * (abs(angles(3,1))/(angles(3,1)));
                  end
                  dyaw = (sign * pi/2) / (2*Fs);
                  N = 2*Fs;
                  Traj_yaw2 = (trajStart + (0:N-1)*dyaw)';
                  Traj_vel2 = zeros(2*Fs,1);
                  T_sim2 = 2;
                   
                  %third: plan your circulation trajectory (v = 0.7 m/s)
                  T_sim3 = round((1 * (2*pi - abs(angles(3,1)))) / 0.7);
                  Traj_vel3 = 0.7 * ones(T_sim3*Fs,1);
                  trajStart = angles(3,1) + sign*(pi/2);
                  dyaw = -sign * ((1.5*pi - trajStart) / (T_sim3*Fs));
                  N = T_sim3*Fs;
                  Traj_yaw3 = (trajStart + (0:N-1)*dyaw)';
                  
                  %fourth: head back to gate for 7 meters and stop
                  T_sim4 = round(7/0.7) + 1;
                  Traj_vel4 = 0.7 * ones((T_sim4-1)*Fs,1);
                  Traj_vel4 = [Traj_vel4 ; zeros(1*Fs,1)];
                  Traj_yaw4 = -sign * pi * ones(T_sim4*Fs,1);
                  
                  T_sim = T_sim1 + T_sim2 + T_sim3 + T_sim4; 
                  
                  Traj_vel = [Traj_vel1;Traj_vel2;Traj_vel3;Traj_vel4];
                  
                  x0 = [0;imu(1,1)];
                  limit_t = 42.42;
                  mode = ['v';'X'];
                  TRAJV(Traj_vel,x0,T_sim,limit_t,mode); % Call trajv service
                  
                  Traj_yaw = [Traj_yaw1;Traj_yaw2;Traj_yaw3;Traj_yaw4];
                  x0 = [angles(3,1);imu(4,1)*imuFs;0];
                  limit_m = 60*r;
                  mode = true;
                  TRAJYAW(Traj_yaw,x0,T_sim,limit_m,mode); % Call trajyaw service
                  count = 4; % Move on to next state
                  
              end
   
              
        elseif((Feedback(17,1)> 10^-2 || Feedback(18,1)> 10^-2 || Feedback(19,1)> 10^-2) && ACTIVE == true)
                trajStart = angles(3,1);
                dyaw = (pi/2)/(1*Fs);
                N = 1*Fs;
                Traj = trajStart + (0:N-1)*dyaw;
                Traj = [Traj'; (angles(3,1) + pi/2)*ones(0.5*Fs,1)]; % Trajectory to follow (yaw)
                x0 = [angles(3,1);imu(4,1)*imuFs;0]; % initial yaw conditions
                T_sim = 1.5; % future time
                limit_m = 26; % Moment limit
                mode = true;
                TRAJYAW(Traj,x0,T_sim,limit_m,mode); % Call trajyaw service
                
                Traj = zeros(1.5*Fs,1); % Trajectory to follow (velocity)
                x0 = [0; imu(1,1)]; % initial velocity conditions
                limit_t = 14; % Force limit
                mode = ['v';'X'];
                TRAJV(Traj,x0,T_sim,limit_t,mode); % Call trajv service
                
                % Go to the waiting state
                Wait_count = 3;
                count = 10;
        end
        
    case 4 % Wait till you achieve circulation without disturbance
        
        % Adjust Height to gate level
        msg_height = gate_z; % Construct height message
        Height_publish(msg_height) % Publish Function
        
        % If disturbance happens, go back to case 3
        % If no disturbance happens and trajectory is fulfilled, go to 5
        if(ACTIVE == false)
            count = 3;
            prev_state = 4; % record current state as past state
        end
        
        % Exit if trajectory following is done
        if(EOT == true)
            count = 1; % get to the next state
            prev_state = 4; % record current state as past state
        end
        
    case 5 % end of plan ( ???? ???? ) 
        
        % Adjust Height to gate level
        msg_height = gate_z; % Construct height message
        Height_publish(msg_height) % Publish Function
        
end
end