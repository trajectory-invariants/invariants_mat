function screw_transf = transform_screw(T,screw)
% Transform screw to another frame given by transformation matrix T
%
% Note: it is possible to write this transformation as a linear 6x6 matrix 
% operator on the screw
%
% Input:  screw        = screw (6x1) in frame {b}
%         T            = pose matrix of frame {b} wrt frame {a}, i.e. the transformation from {a} to {b}
% Output: screw_transf = screw (6x1) in frame {a}

screw_transf = [T(1:3,1:3)*screw(1:3) ;
                T(1:3,1:3)*screw(4:6) + cross(T(1:3,4),T(1:3,1:3)*screw(1:3)) ];

