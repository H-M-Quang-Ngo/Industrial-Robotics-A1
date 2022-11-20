classdef Brick < handle
    properties 
        brickPose
        brickVertexCount
        brickVerts
        brickMesh_h
    end
    %%
    methods
        % place the brick at pos = [x y] position
        function self = Brick(position_xy)
            if nargin == 0
                self.brickPose = transl(0,0,0.033)*rpy2tr(pi, 0 , -pi);
            else
                self.brickPose = transl(position_xy(1),position_xy(2),0.033)*rpy2tr(pi, 0 , -pi);
            end
            self.PlotBrick();
        end
        
        % update the current position of the brick by pos = [x y]
        function Update(self, T) % T is an input brickPose
            self.brickPose = T;
            updatedPoints = (T * [self.brickVerts,ones(self.brickVertexCount,1)]')';
            %self.PlotBrick();
            self.brickMesh_h.Vertices = updatedPoints(:,1:3);
            drawnow()
        end
        
        function PlotBrick(self)
                
                %delete(self.brickMesh_h);
                gcf;
                %these code below are modified from Gavin's
                %'PuttingSimulatedObjectsIntoTheEnvironment.m
                
                %Gavin (2022). Putting Simulated Objects Into The Environment 
                %(https://www.mathworks.com/matlabcentral/fileexchange/58774-putting-simulated-objects-into-the-environment), MATLAB Central File Exchange. Retrieved August 13, 2022.
                
                [f,v,data] = plyread('HalfSizedRedGreenBrick.ply','tri');

                % Get vertex count
                self.brickVertexCount = size(v,1);

                % Move center point to origin
                midPoint = sum(v)/self.brickVertexCount;
                self.brickVerts = v - repmat(midPoint,self.brickVertexCount,1);
                
                % Scale the colours to be 0-to-1 (they are originally 0-to-255
                vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
                
                updatedPoints = (self.brickPose * [self.brickVerts,ones(self.brickVertexCount,1)]')';
                
                self.brickMesh_h = trisurf(f,updatedPoints(:,1),updatedPoints(:,2), updatedPoints(:,3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            
                if isempty(findobj(get(gca,'Children'),'Type','Light'))
                    camlight
                end  
                
                
        end
    end
end