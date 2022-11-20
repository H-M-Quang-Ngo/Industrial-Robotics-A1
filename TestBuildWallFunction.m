%close all
clear

myRobot = LinearUR3withGripper;
brickPile = BrickPile();
surf([-2,-2;2,2],[-2,2;-2,2],[0.01,0.01;0.01,0.01],...
     'CData',imread('ground.jpg'),'FaceColor','texturemap');
lighting flat

q_ini = zeros(1,7);
offsetGrip = 0.09; % the offset above the brick that the gripper will start gripping or releasing
steps = 30; % steps for interpolating trajectory

%% Test all brick
for brick_num = 1:9
    % calibrate the arm position, of 'upPick' m above the brick position to pick them
    if (1<= brick_num) <= 3
        upPick = 0.2;
    elseif (4 <= brick_num) <= 6
        upPick = 0.15;
    else
        upPick = 0.1;
    end
    %% Picking Route
    pose_pick = transl(0,0,offsetGrip)* brickPile.brick{brick_num}.brickPose; % end-effector's pose to start picking a brick
    pose_drop = transl(0,0,offsetGrip)* brickPile.wallLocation(:,:,brick_num); % end-effector's pose to start dropping a brick

        
        % define the arm of the position of upPick m above the pick pose
        T_mid = transl(0,0,upPick)* pose_pick;

        q_mid = myRobot.modelUR3.ikcon(T_mid,q_ini); % solve for joint-coordinates without moving on rail

        %q_mid = [0 q_mid];

            % a joint-coordinate trajectory from 'ini' to 'mid':
            q_ini2mid = mstraj([q_ini;q_mid],ones(1,7)*0.5 , [] , q_ini, 0.15 , 0);
               % mask the prismatic joint (column 1) to 0 if the 'mstraj' 
               % created some positive values (outside joint lim [-0.8 0]) 
               q_ini2mid(:,1) = min(q_ini2mid(:,1),0);
               
            % Cartesian trajectory from 'mid' to 'pick', solving inv kinematic
            % by UR3 arm only
            
            T_mid2pick = ctraj(T_mid, pose_pick, steps); % trajectory in transform 
            q_mid2pick = myRobot.modelUR3.ikcon(T_mid2pick,q_mid); %joint-trajectory

            %q_mid2pick = [zeros(numrows(q_mid2pick),1) q_mid2pick];

        % move from initial position to the picking position and pick the brick
        myRobot.JointMove(q_ini2mid);
        myRobot.GripperOpen();
        myRobot.JointMove(q_mid2pick);
        myRobot.GripperGrip();

    %% Intermediate path to the dropping position with the brick on Gripper:

        % Cartesian trajectory to lift the brick back to above 20cm
        q_pick2mid = zeros(steps,7);
        for i =1:steps 
            q_pick2mid(i,:) = q_mid2pick(steps - (i-1),:); 
        end

        q_inter_first =  q_pick2mid(end,:); % taking the joint state at the picking mid point
    
        T_mid_2 = transl(0,0,0.1)* pose_drop; % The pose of 0.1m above the dropping position
    
        q_inter_mid = myRobot.modelUR3.ikcon(T_mid_2,q_ini); % joint-state at the start dropping position

        % joint coordinate trajectory from picking mid point 
        % to dropping mid point, with the initial position(arm) in between:
             q_ini_noRail = [q_pick2mid(end,1) q_ini(:,2:7)]; 
             q_inter2mid = mstraj([q_inter_first; q_ini_noRail; q_inter_mid],ones(1,7)*0.5,...
             [],q_inter_first , 0.15 ,0);  

        % move from mid picking position to mid dropping position, with the brick 
             q_interTraj = [q_pick2mid; q_inter2mid];

             for i = 1:numrows(q_interTraj)
                 myRobot.JointMove(q_interTraj(i,:));
                 end_effectorPose = myRobot.modelUR3.fkine(q_interTraj(i,:));
                 brickPile.brick{brick_num}.Update(end_effectorPose * transl(0,0,offsetGrip));
             end

     %% Cartesian Move to drop the brick:
        % To reach the drop position, we try to ignore the rail movement and solve
        % for the UR3 motion only. To do that, the 'onlyUR3' property of the class, which is 
        % an UR3 arm going along with that one on the rail, is used to solve for inverse kinematic.

        % Cartesian trajectory to drop the brick, solve without moving on rail
             T_mid2drop = ctraj(T_mid_2, pose_drop, steps);
             q_mid2drop = [ones(steps,1)*q_inter2mid(end,1) zeros(steps,6)];
             q_mid2drop(:,2:7) = myRobot.onlyUR3.ikcon(T_mid2drop, q_inter_mid(:,2:7));
             %q_mid2drop = [ones(steps,1)*q_inter2mid(end,1) q_mid2drop];

        % move from mid dropping position to final dropping position, with the brick 

             for i = 1:numrows(q_mid2drop)
                 myRobot.JointMove(q_mid2drop(i,:));
                 end_effectorPose = myRobot.modelUR3.fkine(q_mid2drop(i,:)); 
                 brickPile.brick{brick_num}.Update(end_effectorPose * transl(0,0,offsetGrip));

                 % forcing the brick at the dropping pose to eliminate the small error
                 if i == numrows(q_mid2drop)
                     brickPile.brick{brick_num}.Update(transl(0,0,-offsetGrip)* pose_drop);
                 end
             end 
             myRobot.GripperGripRev();
    %% go back to initial position:
        q_back = [q_interTraj; q_mid2drop];

        for i =numrows(q_back):-1:1
            myRobot.JointMove(q_back(i,:));
            
            % Close the gripper
            if i == (numrows(q_back) - numrows(q_mid2drop))
                myRobot.GripperOpenRev();
            end
            
            % forcing the arm at the initial position to eliminate small error
            if  norm(q_back(i,:) - q_ini_noRail) <= 0.01 
                myRobot.JointMove(q_ini_noRail);
                break;
            end
        end
        
        % rail movement joint trajectory:
        railReturn = lspb(q_ini_noRail(end,1),0,steps/2);
         % mask the prismatic joint (column 1) to 0 if the 'lspb' 
         % created some positive values (outside joint lim [-0.8 0])
         railReturn = min(railReturn,0);
        q_railReturn = [railReturn zeros(steps/2,6)];
        myRobot.JointMove(q_railReturn);
end       
    
 