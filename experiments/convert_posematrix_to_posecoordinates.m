function pose_data = convert_posematrix_to_posecoordinates(data_input)
        pose_data = [rotm2eul(data_input(1:3,1:3,:),'zyx'),squeeze(data_input(1:3,4,:))'];
end