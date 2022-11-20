classdef LinearUR3withGripper < handle
   properties (SetAccess = private)
        % UR3 on rail model
        modelUR3;
        % A distinct UR3 for testing and solving inverse kinematics without moving on rail
        onlyUR3;
        
        % Gripper fingers
        modelFinger;
        
        % Workspace taken from LinearUR5
        workspace = [-2 2 -2 2 -0.3 2];

        % Transforms to update the gripper position:
        fing1Trans = transl(0,0,0.03) * transl(-0.012,0,0) * trotx(-pi/2) * trotz(-pi);
        fing2Trans = transl(0,0,0.03) * transl(0.012,0,0) * trotx(-pi/2) * trotz(-pi);

        % Trajectory for Gripper fingers
        fing1Open = jtraj([0 0], [-pi/6 0], 20);
        fing1Grip = jtraj([-pi/6 0], [-pi/6 pi/9], 20);

        fing2Open = jtraj([0 0], [pi/6 0], 20);
        fing2Grip = jtraj([pi/6 0], [pi/6 -pi/9], 20);

    end
    
    methods % Class for UR3 robot model
        function self = LinearUR3withGripper()
            self.GetUR3();
            self.GetGripper();
            self.PlotRobot();
        end
        
        %% get UR3 robot
        function GetUR3(self)
 
            link(1) = Link([pi 0 0 pi/2 1],'offset',0,'qlim',[-0.8 0]); % PRISMATIC Link from LinearUR5
            link(2) = Link('d',0.1519,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',pi/2);
            link(3) = Link('d',0,'a',-0.24365,'alpha',0,'qlim', deg2rad([-90 90]), 'offset',-pi/2);
            link(4) = Link('d',0,'a',-0.21325,'alpha',0,'qlim', deg2rad([-170 170]), 'offset', 0);
            link(5) = Link('d',0.11235,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]),'offset', pi);
            link(6) = Link('d',0.08535,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
            link(7) = Link('d',0.0819,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);
            
            % set limits for prismatic joint, taken from LinearUR5
            link(1).qlim = [-0.8 0];
            
            self.modelUR3 = SerialLink(link,'name','MyUR3onRail');
            self.modelUR3.base = self.modelUR3.base * trotx(pi/2)* troty(pi/2); 
            % self.modelUR3.tool = self.modelUR3.tool * transl(0,0,0.1); 
            
            self.onlyUR3 = SerialLink(link(2:7),'name','MyUR3');
            self.onlyUR3.base = trotz(-pi/2); % align with modelUR3 
            self.onlyUR3.tool = self.modelUR3.tool; 
        end
        
        function GetGripper(self)
        %% create two-finger Gripper
            %finger 1 of Gripper
            fing1(1) = Link('d',0,'a',0.06,'alpha',0,'offset',pi/3);
            fing1(2) = Link('d',0,'a',0.06,'alpha',0,'offset',pi/4);
            self.modelFinger{1} = SerialLink(fing1,'name','finger1');
            
            %finger 2 of Gripper
            fing2(1) = Link('d',0,'a',0.06,'alpha',0,'offset',2*pi/3);
            fing2(2) = Link('d',0,'a',0.06,'alpha',0,'offset',-pi/4);
            self.modelFinger{2} = SerialLink(fing2,'name','finger2');
        end
       %% Plot Robot 
        function PlotRobot(self)
            % UR3 on rail
            % q_ini = zeros(1,7);   %initial position
            plotopts = {'fps',240,'workspace',self.workspace,...
                        'noarrow','nowrist','noname','tile1color',[1 1 1],'floorlevel',-0.3};
             
            % Adding graphic for robot and gripper
             % LinearUR3
            for linkIndex = 0:self.modelUR3.n
                [faceData, vertexData, plyData{linkIndex + 1}] = plyread(['myur3link_',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>                
                self.modelUR3.faces{linkIndex + 1} = faceData;
                self.modelUR3.points{linkIndex + 1} = vertexData;
            end
             
             % Gripper Finger 1:
            for linkIndex = 0:self.modelFinger{1}.n
                [faceData, vertexData] = plyread(['finger1_',num2str(linkIndex),'.ply'],'tri');
                self.modelFinger{1}.faces{linkIndex+1} = faceData;
                self.modelFinger{1}.points{linkIndex+1} = vertexData;
            end

             % Gripper Finger 2:
            for linkIndex = 0:self.modelFinger{2}.n
                [faceData, vertexData] = plyread(['finger2_',num2str(linkIndex),'.ply'],'tri');
                self.modelFinger{2}.faces{linkIndex+1} = faceData;
                self.modelFinger{2}.points{linkIndex+1} = vertexData;
            end


            % Display robot 
            self.modelUR3.plot3d(zeros(1,self.modelUR3.n),plotopts{:});
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end  
            self.modelUR3.delay = 0;
            
            hold on
            
            % Gripper's base on UR3
            self.modelFinger{1}.base = self.modelUR3.fkine(self.modelUR3.getpos) * self.fing1Trans;
            self.modelFinger{2}.base = self.modelUR3.fkine(self.modelUR3.getpos) * self.fing2Trans;

            % Display gripper
            self.modelFinger{1}.plot3d([0 0],plotopts{:});
            self.modelFinger{2}.plot3d([0 0],plotopts{:});
            
            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.modelUR3.n
                handles = findobj('Tag', self.modelUR3.name);
                h = get(handles,'UserData');
                try 
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                                                                  , plyData{linkIndex+1}.vertex.green ...
                                                                  , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
              
        end
        
        %% gripper goes with robot
        function GripperUpdate(self)     
            self.modelFinger{1}.base = self.modelUR3.fkine(self.modelUR3.getpos) * self.fing1Trans;
            self.modelFinger{2}.base = self.modelUR3.fkine(self.modelUR3.getpos) * self.fing2Trans;

            self.modelFinger{1}.animate(self.modelFinger{1}.getpos());
            self.modelFinger{2}.animate(self.modelFinger{2}.getpos());
            drawnow()
        end
        
        %% move the arm into a given joint coordinates or along a joint trajectory
        function JointMove(self,q_traj)
            %self.PlotRobot();
            if numrows(q_traj)== 1
                    self.modelUR3.animate((q_traj));
                    hold on
                    self.GripperUpdate();
                    
                    % update the position of the UR3 arm test model so it's
                    % synchronous with the movement on rail
                    modelUR3pos = transl(self.modelUR3.base);
                    self.onlyUR3.base(1:3,4) = [modelUR3pos(1) + q_traj(1); modelUR3pos(2:3)];
            else
                for i=1:numrows(q_traj)
                    self.modelUR3.animate(q_traj(i,:));
                    hold on
                    self.GripperUpdate();
                    
                    modelUR3pos = transl(self.modelUR3.base);
                    self.onlyUR3.base(1:3,4) = [modelUR3pos(1) + q_traj(i); modelUR3pos(2:3)];
                end
            end
        end
        %% Gripper Operation
        % open the fingers
        function GripperOpen(self)
            for i = 1:numrows(self.fing1Open)
                self.modelFinger{1}.animate(self.fing1Open(i,:));
                self.modelFinger{2}.animate(self.fing2Open(i,:));
            end
        end
        
        % grip the object
        function GripperGrip(self)    
            for i = 1:numrows(self.fing1Grip)
                self.modelFinger{1}.animate(self.fing1Grip(i,:));
                self.modelFinger{2}.animate(self.fing2Grip(i,:));
            end
        end
        
        % reverse the 'GripperGrip'
        function GripperGripRev(self)
            for i = numrows(self.fing1Grip):-1:1
                self.modelFinger{1}.animate(self.fing1Grip(i,:));
                self.modelFinger{2}.animate(self.fing2Grip(i,:));
            end
        end

        % reverse the 'GripperOpen'
        function GripperOpenRev(self)    
            for i = numrows(self.fing1Open):-1:1
                self.modelFinger{1}.animate(self.fing1Open(i,:));
                self.modelFinger{2}.animate(self.fing2Open(i,:));
            end
        end
    end    
end



