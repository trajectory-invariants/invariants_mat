FileName = 'contour.stl';
[TR,~,~,~]  = stlread(FileName);
Vertices    = TR.Points;
Faces       = TR.ConnectivityList;

figure()
scatter3(Vertices(:,1),Vertices(:,2),Vertices(:,3))
hold on
axis equal
xlabel('x')
ylabel('y')

N = size(Vertices,1);
candidates = [];
for k = 1:N
    if Vertices(k,2) < 1 && Vertices(k,3) < 3 
        candidates = [candidates;Vertices(k,:)];
    end
end
N = size(candidates,1);
for k = 1:N
    if candidates(k,1) < -150 && candidates(k,1) > -175 && candidates(k,2) > -10
        candidates(k,:) = [NaN, NaN, NaN];
    end
    if candidates(k,1) > 150 && candidates(k,1) < 175 && candidates(k,2) > -10
        candidates(k,:) = [NaN, NaN, NaN];
    end
end
candidates(isnan(candidates(:,1)),:) = [];
candidates = sortrows(candidates,1);
candidates = smoothdata(candidates,'gaussian');

N = size(candidates,1);
s = [0];
for k = 1:N-1
    s(k+1) = s(k) + norm(candidates(k+1,:)-candidates(k,:));
end
s = s/s(end);
candidate_x = interp1(s,candidates(:,1),linspace(0,1,101));
candidate_y = interp1(s,candidates(:,2),linspace(0,1,101));
candidate_z = interp1(s,candidates(:,3),linspace(0,1,101));

contour_edge = [candidate_x',candidate_y',candidate_z'];

scatter3(candidates(:,1),candidates(:,2),candidates(:,3),'r','filled')

save('contour_edge','contour_edge')