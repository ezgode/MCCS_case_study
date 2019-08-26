%%==========================================================================
%   TP :            Case study: Exercse 4
%   Contact:        ezequiel.gonzalezdebada@epfl.ch
%                   matin.macktoobian@epfl.ch
%==========================================================================
    close all;clear all;clc;

    ut = utilities;
    sol = ut.load_solution_class(4);

%% load path of reference
%==========================================================================
    chosen_path = sol.select_reference_path;
    if ut.stopCondition(chosen_path, 'select_reference_path'), return; end
    
    if ~isempty(chosen_path) 
        load(chosen_path);
    else
        path_to_track = [];
    end

%% Define the position of the obstacle path of reference
%==========================================================================
    obstacle = sol.getObstaclePosition;
    if ut.stopCondition(obstacle, 'getObstaclePosition'), return; end
    
%% Define the attractive potential fields 
%==========================================================================
    attractive_field = sol.getAttractiveField;
    if ut.stopCondition(attractive_field, 'getAttractiveField'), return; end

%% Define the repulsive potential fields 
%==========================================================================
    % translate the coordinates of the obstacles
    dist_path_to_obs = sqrt(sum( (path_to_track.coordinates - repmat(obstacle,size(path_to_track.coordinates,1),1)).^2, 2));
    [min_dist, pos] = min(dist_path_to_obs);
    s_d_obstacle_coordinates = [path_to_track.s_coordinate(pos(1)), min_dist];

    %get the repulsive field
    repulsive_field = sol.getRepulsiveField(s_d_obstacle_coordinates);
    if ut.stopCondition(repulsive_field, 'getRepulsiveField'), return; end
    %plot the potential function
    [ax] = ut.representPotentialFields(path_to_track, attractive_field, repulsive_field,'s_step',1,'d_step',1);

    [~, potential_field, s, d, s_grid, d_grid] = ut.representPotentialFields(path_to_track, attractive_field, repulsive_field,'s_step',0.1,'d_step',0.1,'plot',false);

    %% Calculate the gradient 
    [fs,fd] = sol.calculateGradient(potential_field, s, d);
    if ut.stopCondition(fs, 'calculateGradient'), return; end

    %% get the result 
    [x0] = sol.getInitialState();
    if ut.stopCondition(x0, 'getInitialState'), return; end

    if ~isempty(x0)
        initial_pos_s_d = x0(1:2,1);
    else
        initial_pos_s_d  = [];
    end

    %% get the path and plot results

    [path_s_d] = ut.obtainMinimumPotentialPath(initial_pos_s_d, s, d, fs, fd,'path_points',8000,'step_size',0.01);
    path = ut.SDCoordToXY( path_to_track, path_s_d(:,1), path_s_d(:,2));

    z = interp2(s_grid, d_grid, potential_field, path_s_d(:,1), path_s_d(:,2));
    plot(path(:,1),path(:,2),'LineWidth',2,'Parent',ax(5));
    plot3(path(:,1),path(:,2),z+100,'LineWidth',2,'Parent',ax(8),'Color',[1 0 0]);
    plot3(path_s_d(:,1),path_s_d(:,2),z+100,'LineWidth',2,'Parent',ax(4),'Color',[1 0 0]);