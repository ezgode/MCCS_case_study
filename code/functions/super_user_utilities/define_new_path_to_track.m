close all;
path_to_track.coordinates = userDefinedPath('close_path',true);
%path_to_track.coordinates = unique(path_to_track.coordinates,'rows');

path_to_track.coordinates = path_to_track.coordinates( prod(diff(path_to_track.coordinates,1),2) ~= 0 , : );
%path_to_track.coordinates = [path_to_track.coordinates; path_to_track.coordinates(1,:)];




%%
extended_path = extendPathCoordinates(path_to_track.coordinates);

path_to_track.s_coordinate = extended_path(:,1);
path_to_track.orientation = extended_path(:,4);
path_to_track.curvature = extended_path(:,5);
path_to_track.curvature = LineCurvature2D( path_to_track.coordinates );

%
%make the path start int he origin. 
path_to_track.coordinates = path_to_track.coordinates - path_to_track.coordinates(1,:);
%rotate it to make the first angle to be 0. 
rotation_angle = -path_to_track.orientation(1);
rotation_matrix = [cos(rotation_angle), -sin(rotation_angle); sin(rotation_angle), cos(rotation_angle) ];
path_to_track.coordinates = (rotation_matrix * path_to_track.coordinates')';
plot(path_to_track.coordinates(:,1),path_to_track.coordinates(:,2));



%unique_s_coordinate = unique(path_to_track.s_coordinate);

figure,plot(path_to_track.curvature)

save('/Users/ezequiel/case_study_code/code/path_4.mat','path_to_track')