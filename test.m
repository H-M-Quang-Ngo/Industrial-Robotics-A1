load('Taskspace.mat','robotTr_points');
myRobot = LinearUR3withGripper;
brickPile = BrickPile();
surf([-2,-2;2,2],[-2,2;-2,2],[0.01,0.01;0.01,0.01],...
     'CData',imread('ground.jpg'),'FaceColor','texturemap');
lighting flat
hold on
            % if the robotTr_points exists then plot it
            if exist('robotTr_points','var') == 1
                gcf
                taskSpace_h = plot3(robotTr_points(:,1),robotTr_points(:,2),robotTr_points(:,3),':cyan');
                [k, taskVolume] = convhull(robotTr_points);
                %taskSpace_h = trisurf(k,robotTr_points(:,1),robotTr_points(:,2),robotTr_points(:,3),'FaceColor','cyan');
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
            