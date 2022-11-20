classdef BrickPile < handle
   
    properties (SetAccess = private)
        brick
        wallLocation
    end
    
    methods
    
        function self = BrickPile(x1,y1)
            if (nargin == 0)   
                x1 = 0.28;
                y1 = -0.25;
            end
            self.PositioningBricks(x1,y1);
            self.GetWallLocation();
        end

        function PositioningBricks(self,x1,y1)
            x_inc = 0.12; % distance between each brick in x-direction
            y_inc = 0.145; % distance between each brick in y-direction
            
            % first row
            self.brick{1} = Brick([x1 , y1]);
            self.brick{2} = Brick([x1 , y1 + y_inc]);
            self.brick{3} = Brick([x1 , y1 + 2*y_inc]);

            % second row
            x2 = x1 + x_inc;
            y2 = y1 - 0.05;

            self.brick{4} = Brick([x2 , y2]);
            self.brick{5} = Brick([x2 , y2 + y_inc]);
            self.brick{6} = Brick([x2 , y2 + 2*y_inc]);

            % third row
            x3 = x2 + x_inc;
            y3 = y1 + 0.05;

            self.brick{7} = Brick([x3 , y3]);
            self.brick{8} = Brick([x3 , y3 + y_inc]);
            self.brick{9} = Brick([x3 , y3 + 2*y_inc]); 

        end
          
        function GetWallLocation(self)
            
            self.wallLocation = NaN(4,4,9); 
            self.wallLocation(:,:,1) = transl(-0.75,0.4,0.033)*trotx(pi)*trotz(pi/2);

            % build the first row
            for i = 1:2
                    self.wallLocation(:,:,i+1) = transl(-0.13343,0,0) * self.wallLocation(:,:,i);
            end

            % build the other row based on first row
            for i = 1:6
                    self.wallLocation(:,:,i+3) = transl(0,0,0.033) * self.wallLocation(:,:,i);
            end
        end
            
    end
end