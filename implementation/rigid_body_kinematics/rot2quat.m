function q_all = rot2quat(R_all)
%input: rotmatrix R(3,3) or rotmatrix R(3,3,N)
%output: xyz rpy

%Note: one rotation matrix may correspond to two quaternions: +q and -q.
%This ambiguity is normally solved by assuming the scalar part positive.

%However, for quaternions in a time series we desire continuity so we will 
%look at the previous time sample to determine if the sign must be positive 
%or negative for continuity


N = size(R_all,3);
q_all = zeros(N,4);

for i=1:N
    
    R = R_all(:,:,i);
    R = orthonormalize_rotation(R);
    
    qs = sqrt(trace(R)+1)/2.0;
    kx = R(3,2) - R(2,3);   % Oz - Ay
    ky = R(1,3) - R(3,1);   % Ax - Nz
    kz = R(2,1) - R(1,2);   % Ny - Ox

    if (R(1,1) >= R(2,2)) && (R(1,1) >= R(3,3)) 
        kx1 = R(1,1) - R(2,2) - R(3,3) + 1; % Nx - Oy - Az + 1
        ky1 = R(2,1) + R(1,2);              % Ny + Ox
        kz1 = R(3,1) + R(1,3);              % Nz + Ax
        add = (kx >= 0);
    elseif (R(2,2) >= R(3,3))
        kx1 = R(2,1) + R(1,2);          % Ny + Ox
        ky1 = R(2,2) - R(1,1) - R(3,3) + 1; % Oy - Nx - Az + 1
        kz1 = R(3,2) + R(2,3);          % Oz + Ay
        add = (ky >= 0);
    else
        kx1 = R(3,1) + R(1,3);          % Nz + Ax
        ky1 = R(3,2) + R(2,3);          % Oz + Ay
        kz1 = R(3,3) - R(1,1) - R(2,2) + 1; % Az - Nx - Oy + 1
        add = (kz >= 0);
    end

    if add
        kx = kx + kx1;
        ky = ky + ky1;
        kz = kz + kz1;
    else
        kx = kx - kx1;
        ky = ky - ky1;
        kz = kz - kz1;
    end
    nm = norm([kx ky kz]);
    if nm == 0
        q = [0 0 0 1]; %notation [vector  scalar] here 
    else
        s = sqrt(1 - qs^2) / nm;
        qv = s*[kx ky kz];

        q = [qv qs]; %notation [vector  scalar] here 

        if i>1 && norm(q - q_all(i-1,:))>0.5
            q = -q;
        end
    end
    
    q_all(i,:) = q;
    
end

