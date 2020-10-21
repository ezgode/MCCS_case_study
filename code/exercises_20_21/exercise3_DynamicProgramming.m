%==========================================================================
%   TP :            Case study: Exercse 3
%   Contact:        ezequiel.gonzalezdebada@epfl.ch
%                   matin.macktoobian@epfl.ch
%==========================================================================
close all;clear all;clc;

%% script parameters
%==========================================================================
    sampling_time = 0.5;
    max_n_nodes_graph = 40000;

    %contraints
    max_lateral_deviation = 7.0; %[m]
    max_heading_deviation = pi; %[rad]
    time_to_start_randomizing_the_tree = 3; %[s] time from which the tree is simplified
    threshold_number_of_nodes = 100; %average number of new nodes that have to be created at each iteration. 
    
%% load the linearized discretized model
%==========================================================================
    ut = utilities;
    sol = ut.load_solution_class(3);

    % initial state
    x0 = sol.getInitialState;
    if ut.stopCondition(x0, 'getInitialState'), return; end
    
    parameters = sol.getSystemParameters;
    if ut.stopCondition(parameters, 'getSystemParameters'), return; end
    
    [allowed_u] = sol.getAllowedControlValues;
    if ut.stopCondition(allowed_u, 'getAllowedControlValues'), return; end

    %load path of reference
    chosen_path = sol.select_reference_path;
    if ut.stopCondition(chosen_path, 'select_reference_path'), return; end
    
    if ~isempty(chosen_path)
        load(chosen_path);
    else
        path_to_track = [];
    end

%% 1 - Built graph for dynamic programming application
%==========================================================================
    % LOGIC TO, IF YOU ARE RUNNING THIS WITHOUT CHANGING THE PARAMETERS OF
    % THE PATH CREATION, LOAD THE PATH INSTEAD OF GENERATING IT AGAIN.
    graph_args = {sampling_time, x0, path_to_track, allowed_u, ...
        parameters,'plot_nodes',false, 'max_n_nodes',max_n_nodes_graph,...
        'max_lateral_deviation',max_lateral_deviation,...
        'max_heading_deviation',max_heading_deviation,...
        'time_to_loose_it',time_to_start_randomizing_the_tree,...
        'threshold_number_of_nodes',threshold_number_of_nodes};
    
    f = fopen('./graph.mat');
    update_graph = f==-1;
    if f>=0 
        fclose(f);
        load('./graph.mat');
        %compare the cells
        cells_are_equal = true;
        try
        for i = 1:length(graph_args)
            if ~isstruct(graph_args{i})
                field_1 = graph_args{i};
                field_2 = saved_graph_args{i};
                cells_are_equal = cells_are_equal & ~any(field_1(:) ~= field_2(:));
            else
                fields_names = fields(graph_args{i});
                for field_id = 1:length(fields_names)
                    field_1 = getfield(graph_args{i},fields_names{field_id});
                    field_2 = getfield(saved_graph_args{i},fields_names{field_id});
                    cells_are_equal = cells_are_equal & ~any( field_1(:) ~= field_2(:)); 
                end
            end
        end
        catch
            cells_are_equal = false;
        end
        update_graph = update_graph || ~cells_are_equal;
    end
    if update_graph 
        [nodes, edges] = ut.buildDynProgGraph(graph_args{:});
        saved_graph_args = graph_args;
        save('./graph.mat','nodes','edges','saved_graph_args')
    else
        fprintf('\nUsing an existing graph\n')
    end


%% set obstacles and solve the 'optimization' using dynamic programming
%==========================================================================
    %considered obstacles
    obstacles_options = sol.getObstacle;
    if ut.stopCondition(obstacles_options, 'getObstacle'), return; end
    
    %retrieve obstacle position at this iteration
    obstacles.position = obstacles_options;

    %explore the graph forward, by calculating the costs. 
    node_cost = sol.assignCostsToNodes(nodes, edges, obstacles);
    if ut.stopCondition(node_cost, 'assignCostsToNodes'), return; end

    %plot the graph
    [ax] = ut.plotGraph(nodes, edges, path_to_track, 'plot_nodes',true, 'plot_terminal_nodes', true, 'plot_dead_ends', true, 'obstacles', obstacles);

    %Obtain the optimal path using bellman's principle of optimality.
    [optimal_node_sequence] = sol.backtrackNodeValues(nodes, edges, node_cost, sampling_time);
    if ut.stopCondition(optimal_node_sequence, 'backtrackNodeValues'), return; end

    %plot optimal trajectory
    ut.plotDynProgTraj(nodes, optimal_node_sequence, ax);
