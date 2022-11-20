classdef Lab1Assignment < handle
    properties
        robot
        brickPile
        wallLocation
        L = log4matlab('Log4BuildWallFunction.log');
    end
    
    methods 
        function self = Lab1Assignment()
            %clf
            self.CreateEnvironment();
            %self.PlaceRobot();
        end
        
        %% Set up Working Environment
        function CreateEnvironment(~)
            figure('name','IR_A1_Workspace');
            axis equal
            xlim([-2.7 2]);
            ylim([-2 2]);
            zlim([0 2]);
            view([116.61 21.01])
            hold on

            camlight
            % Safety fence
            PlaceObject('SafetyFence.ply',[0 0 -0.1]);
            
            % Ground and wall
            surf([-2.7,-2.7;2,2],[-2,2;-2,2],[0,0;0,0],...
            'CData',imread('ground.jpg'),'FaceColor','texturemap');

            surf([-2.7,-2.7;-2.7,-2.7],[-2,2;-2,2],[0,0;2.5,2.5],...
            'CData',imread('wall.jpg'),'FaceColor','texturemap');
            
            surf([2,2;-2.7,-2.7],[-2,-2;-2,-2],[0,2;0,2],...
            'CData',imread('sidewall.jpg'),'FaceColor','texturemap');

            % Danger signs
            PlaceObject('DangerSign.ply',[-2.828 -0.42 1]);
            PlaceObject('DangerSign.ply',[-2.828 -2.59 1]);
            
            h1 = PlaceObject('DangerSign.ply',[-2.828 -0.42 1]);
            verts1 = [get(h1,'Vertices'), ones(size(get(h1,'Vertices'),1),1)] * trotz(pi);
            set(h1,'Vertices',verts1(:,1:3));
            
            h2 = PlaceObject('DangerSign.ply',[-2.828 -2.59 1]);
            verts2 = [get(h2,'Vertices'), ones(size(get(h1,'Vertices'),1),1)] * trotz(pi);
            set(h2,'Vertices',verts2(:,1:3));

            % Control box
            PlaceObject('ControlBox.ply',[-2.7 1.7 1.5]);
            
            % Fire extinguisher and first aid kit
            PlaceObject('FireExtinguisher.ply',[-2.7 1.8 0.2]);
            
            % Beacon lights
            PlaceObject('BeaconLight.ply',[-1 -1.05 0.07]);
            PlaceObject('BeaconLight.ply',[-1  1.05 0.07]);
            % Observer
            PlaceObject('Observer.ply',[-2.1 1 0]);
          
        end
    
        %% Put Robot into Environment
        function PlaceRobot(self,givenPose)
            % Set up the given pose limit
            if nargin == 1 || (givenPose(1,4) < -0.2 || 0.8 < givenPose(1,4)) || (givenPose(2,4) < -0.6 || 0.6 < givenPose(2,4))
                givenPose = eye(4);
            end
            
            % Delete the current robot and its figure on workspace if exists
            if ~isempty(self.robot)
                LinearUR3_h = findobj('Tag',self.robot.modelUR3.name);
                gripper_h1 = findobj('Tag',self.robot.modelFinger{1}.name);
                gripper_h2 = findobj('Tag',self.robot.modelFinger{2}.name);
     
                delete(LinearUR3_h);
                delete(gripper_h1);
                delete(gripper_h2);
                delete(self.robot);
            end

            % Call and place the robot
            self.robot = LinearUR3withGripper();
            self.robot.modelUR3.base = givenPose* self.robot.modelUR3.base;
            self.robot.JointMove(zeros(1,7));
            
            % create the Wall location based on the robot's position
            self.GetWallLocation();
            
        end
        
        %% Relative position x,y of the Brick Pile, with respect to the robot base, in case there is a robot in the workspace
        function PlaceBricks(self,x,y)
            
            % Delete current brickPile and its figure on workspace if exists
            if ~isempty(self.brickPile)
                for i = 1:9
                    delete(self.brickPile.brick{i}.brickMesh_h);
                end
            end
            delete(self.brickPile);
            
            % Limit the relative position so that the robot can pick up each brick
            if nargin == 1 || (x < 0.25 || 0.3 < x) || (y < -0.25 || -0.2 < y)
                x = 0.28;
                y = -0.25;
            end
            
            if ~isempty(self.robot)
                x = x + self.robot.modelUR3.base(1,4);
                y = y + self.robot.modelUR3.base(2,4);
                self.brickPile = BrickPile(x,y); % place the brick pile at x,y position with respect to the robot base
            else
                self.brickPile = BrickPile(x,y); % place the brick pile at x,y global position if there is no robot
            end
            
        end
        %% Create the Wall Location with respect to the robot position
        function GetWallLocation(self)     
            
            self.wallLocation = NaN(4,4,9); 

            % wall location with respect to the current position of the robot
            currentRobot =  transl(self.robot.modelUR3.base);  
            self.wallLocation(:,:,1) = transl(-0.75 + currentRobot(1),0.4 + (currentRobot(2)),0.033)...
                * trotx(pi)*trotz(pi/2);

            % get the first row
            for i = 1:2
                    self.wallLocation(:,:,i+1) = transl(-0.13343,0,0) * self.wallLocation(:,:,i);
            end

            % get the other rows based on first row
            for i = 1:6
                    self.wallLocation(:,:,i+3) = transl(0,0,0.033) * self.wallLocation(:,:,i);
            end

    end
        %% Build the Wall
        function BuildWall(self)
            
            if ~isempty(self.robot) && ~isempty(self.brickPile)
            
            q_ini = zeros(1,7);
            offsetGrip = 0.09; % the offset above the brick that the gripper will start gripping or releasing it
            steps = 30; % steps for interpolating trajectory

            for brickNum = 1:9
                % calibrate the arm position, of 'upPick' m above the brick's position 
                if (1<= brickNum) <= 3
                    upPick = 0.2;
                elseif (4 <= brickNum) <= 6
                    upPick = 0.15;
                else
                    upPick = 0.1;
                end
                
                posePick = transl(0,0,offsetGrip)* self.brickPile.brick{brickNum}.brickPose; % end-effector's pose to pick a brick
                poseDrop = transl(0,0,offsetGrip)* self.wallLocation(:,:,brickNum); % end-effector's pose to drop a brick

                %% Picking Route
                    % define the arm's position of 'upPick' m above the pick pose
                    T_mid = transl(0,0,upPick)* posePick;
                    q_mid = self.robot.modelUR3.ikcon(T_mid,q_ini); % solve for joint-coordinates


                    % a joint-coordinate trajectory from 'initial' to 'mid':
                    q_ini2mid = mstraj([q_ini;q_mid],ones(1,7)*0.5 , [] , q_ini, 0.15 , 0);
                       % mask the prismatic joint (column 1) to 0 if the 'mstraj' 
                       % created some positive values (outside joint lim [-0.8 0]) 
                        q_ini2mid(:,1) = min(q_ini2mid(:,1),0);

                    % Cartesian trajectory from 'mid' to 'pick':
                    T_mid2pick = ctraj(T_mid, posePick, steps); % trajectory in transform 
                    q_mid2pick = self.robot.modelUR3.ikcon(T_mid2pick,q_mid); % joint-trajectory

                    % move from initial position to the picking position and pick the brick
                    self.robot.JointMove(q_ini2mid);
                    self.robot.GripperOpen(); % open the gripper
                    self.robot.JointMove(q_mid2pick);
                    self.robot.GripperGrip(); % close the gripper to pick the brick

                    % log the transform of end-effector and brick 
                    self.GetLog(brickNum);

                %% Intermediate path to the dropping position with the brick on Gripper:

                    % Cartesian trajectory to lift the brick back above 'upPick' m
                    q_pick2mid = zeros(steps,7);
                    for i =1:steps 
                        q_pick2mid(i,:) = q_mid2pick(steps - (i-1),:); % reverse the mid2pick trajectory
                    end

                    q_inter_first =  q_pick2mid(end,:); % taking the joint state at the picking mid point

                    T_mid_2 = transl(0,0,0.1)* poseDrop; % The pose of 0.1m above the dropping position
                    q_inter_mid = self.robot.modelUR3.ikcon(T_mid_2,q_ini); % joint-state at the above position

                    q_ini_noRail = [q_pick2mid(end,1) q_ini(:,2:7)]; % the initial pose with the current position on rail
                    
                    % joint coordinate trajectory from picking mid point 
                    % to dropping mid point, with the initial position(arm) in between:
                    q_inter2mid = mstraj([q_inter_first; q_ini_noRail; q_inter_mid],ones(1,7)*0.5,...
                                    [],q_inter_first , 0.15 ,0);  

                    % trajectory to move from mid picking position to mid dropping position, with the brick 
                    q_interTraj = [q_pick2mid; q_inter2mid];

                    for i = 1:numrows(q_interTraj)
                         
                         
                         self.robot.JointMove(q_interTraj(i,:));
                         end_effectorPose = self.robot.modelUR3.fkine(q_interTraj(i,:));
                         self.brickPile.brick{brickNum}.Update(end_effectorPose * transl(0,0,offsetGrip));
                         
                    end

                 %% Cartesian Move to drop the brick:
                    % To reach the drop position, we try to ignore the rail movement and solve
                    % for the UR3 motion only. To do that, the 'onlyUR3' property of the class, which is 
                    % an UR3 arm going along with that one on the rail, is used to solve for inverse kinematic.

                    % Cartesian trajectory to drop the brick, solve without moving on rail
                    T_mid2drop = ctraj(T_mid_2, poseDrop, steps);
                    q_mid2drop = [ones(steps,1)*q_inter2mid(end,1) zeros(steps,6)];
                    q_mid2drop(:,2:7) = self.robot.onlyUR3.ikcon(T_mid2drop, q_inter_mid(:,2:7));

                    % move from mid dropping position to final dropping position, with the brick 
                    for i = 1:numrows(q_mid2drop)
                         self.robot.JointMove(q_mid2drop(i,:));
                         end_effectorPose = self.robot.modelUR3.fkine(q_mid2drop(i,:)); 
                         self.brickPile.brick{brickNum}.Update(end_effectorPose * transl(0,0,offsetGrip));

                         % forcing the brick at the dropping pose to eliminate the small error
                         if i == numrows(q_mid2drop)
                             self.brickPile.brick{brickNum}.Update(transl(0,0,-offsetGrip)* poseDrop);
                         end
                    end 
                    self.robot.GripperGripRev(); % release the brick
                    
                    % log the transform of end-effector and brick 
                    self.GetLog(brickNum);

                %% go back to initial position:
                    q_back = [q_interTraj; q_mid2drop];

                    for i =numrows(q_back):-1:1
                        self.robot.JointMove(q_back(i,:));
                    
                        % Close the gripper
                        if i == (numrows(q_back) - numrows(q_mid2drop))
                            self.robot.GripperOpenRev();
                        end
                        
                        % Forcing the arm at the initial position to eliminate small error
                        if  norm(q_back(i,:) - q_ini_noRail) <= 0.01 
                            self.robot.JointMove(q_ini_noRail);
                            break;
                        end
                    end

                    % rail movement joint trajectory to initial position on rail
                    railReturn = lspb(q_ini_noRail(end,1),0,steps/2);
                        % mask the prismatic joint (column 1) to 0 if the 'lspb' 
                        % created some positive values (outside joint lim [-0.8 0]) 
                        railReturn = min(railReturn,0);
                    
                    q_railReturn = [railReturn zeros(steps/2,6)];
                    self.robot.JointMove(q_railReturn);
            end
            
                disp('Wall built successful!')     
            else
                disp('Error! No Robot or Brick Pile.')
            end
        end
        
        %% Calculate the Robot workspace
        function [taskVolume_1, taskVolume_2] = CalculateWorkspace(self)
            self.PlaceRobot();
            % the file includes 'robotTr_points', which is the workspace for the robot
            load('Taskspace.mat','robotTr_points');

            % if the robotTr_points exists then plot it
            if exist('robotTr_points','var') == 1
                gcf
                taskSpace_h = plot3(robotTr_points(:,1),robotTr_points(:,2),robotTr_points(:,3),':r');
            
            else
            % calculate the workspace by stepping through each joints (except joint 7) 
            % if the file hasn't existed
                qlim = self.robot.modelUR3.qlim;
                num = 1;
                steps = pi/3;
                robotTr_points = zeros(424944,3);
                
                for i = qlim(1,1):0.1:qlim(1,2)
                    for j = qlim(2,1):steps:qlim(2,2)
                        for k = qlim(3,1):steps:qlim(3,2)
                            for l = qlim(4,1):steps:qlim(4,2)
                                for m = qlim(5,1):steps:qlim(5,2)
                                    for n = qlim(6,1):steps:qlim(6,2)
                                        robotTr = self.robot.modelUR3.fkine([i,j,k,l,m,n,0]);
                                        if robotTr(3,4) <=0 
                                            continue;
                                        end
                                        robotTr_points(num,:) = robotTr(1:3,4)';
                                        num = num +1;
                                    end
                                end
                            end
                        end
                    end
                end
                save('Taskspace.mat','robotTr_points');
                gcf
                taskSpace_h = plot3(robotTr_points(:,1),robotTr_points(:,2),robotTr_points(:,3),':r');
            end
            
            railVolume = 0.0075; % rail volume to be subtracted when calculating workspace

            % Calculate the task volume, which can be estimated as a half-cylinder with two 
            % ends are 2 quarters of a sphere (a semi-sphere)
            sphereRadius = max(robotTr_points(:,2));
            cylinderHeight = 0.8; % the length of the rail
            
            taskVolume_1 = 2/3*pi*(sphereRadius^3) + 0.5*cylinderHeight*pi*(sphereRadius^2) - railVolume;
            
            fprintf('The workspace volume estimated using half-sphere and half-cylinder is %f cubic-meters.\n',round(taskVolume_1,3));
            
            % Calculate the task volume, using 'convhull' function:
            [~,taskVolume_2] =  convhull(robotTr_points(:,1),robotTr_points(:,2),robotTr_points(:,3));
            taskVolume_2 = taskVolume_2 - railVolume;
            
            fprintf('The workspace volume estimated using convex hull is %f cubic-meters.\n',round(taskVolume_2,3));
            
            pause()
            delete(taskSpace_h);
        end

        %% Logging the BuildWall function to compare the end-effector and bricks' transforms

        function GetLog(self,brickNum)
            
            self.L.SetLoggerLevel('BuildWall',self.L.DEBUG)
            self.L.mlog = {self.L.DEBUG,'BuildWall','This is a debug message:'};
            % self.L.mlog = {self.L.WARN,'BuildWall','This is a warning message'};
            % self.L.mlog = {self.L.ERROR,'BuildWall','This is an error message'};
            
            % Get the transforms of the end-effector and the brick
            ePose = self.robot.modelUR3.fkine(self.robot.modelUR3.getpos); % end-effector transform
            brPose = self.brickPile.brick{brickNum}.brickPose; % brick's transform
            
            % Extract the position vector [x y z]:
            eVec = ePose(1:3,4); % end-effector
            eVec(3) = eVec(3) - 0.09; % minus back the offset grip of 0.09m, which is seen from the BuildWall Function
            brVec = brPose(1:3,4); % brick
            
            % Calculate the difference(mm):
            dif = norm(eVec - brVec)*1000;

            % Log the transform of end-effector and brick 
            self.L.mlog = {self.L.DEBUG,'BuildWall',['The brick ',num2str(brickNum),'-th position is:',self.L.MatrixToString(brPose)]};
            self.L.mlog = {self.L.DEBUG,'BuildWall',['The end-effector position is:',self.L.MatrixToString(ePose)]};
            
            % Printout the differences:
            self.L.mlog = {self.L.DEBUG,'BuildWall',['The difference is ',num2str(dif),'mm.']};
        end
        %% Play the ROS bag
        function PlayROSbag(self)
            if ~isempty(self.robot)
                bag = rosbag('2018-03-20-18-34-46.bag'); 

                % retrieve the joint states
                bSel = select(bag,'Topic','/joint_states');
                
                % read the joint_states in form of struct
                msgStructs = readMessages(bSel,'DataFormat','struct');
                
                % Create the trajectory holder
                q = zeros(numrows(msgStructs),7);
                
                % Trajectory retrieve from the ros bag:
                for i = 1:numrows(msgStructs)
                    q(i,2:7) = msgStructs{i}.Position';
                    
                    % Calibrate some joints due to different offsets:
                    q(i,2) = q(i,2) - pi/2;
                    q(i,3) = q(i,3) + pi/2;
                    q(i,5) = q(i,5) - pi;
                end

                % Move the robot
                    % From the pre-set initial position to the initial
                    % position in the ROS bag
                q0 = jtraj(zeros(1,7),q(1,:),100);
                self.robot.JointMove(q0);
                
                    % Trajectory from the ROS bag
                for i = 1:8:numrows(q)
                    self.robot.JointMove(q(i,:));
                end
            end
        end
    end
        
end
            
