function T = extend_homogeneous_matrix(T_input)

N = size(T_input,3);
T = zeros(4,4,N);
for k = 1:N
    T(:,:,k) = [T_input(:,:,k); 0 0 0 1];
end
