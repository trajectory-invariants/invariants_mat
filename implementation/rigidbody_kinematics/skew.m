function [w_skew] = skew(w)
% Construct a 3x3 skew-symmetric matrix from the given 3-vector "w" so that cross(w,a) = skew(w)*a
%
% Input:  w      = 3-vector
% Output: w_skew = 3x3 skew-symmetric matrix built from elements of w

x = w(1);
y = w(2);
z = w(3);

w_skew = [0,-z,y;
          z,0,-x;
          -y,x,0];
end
