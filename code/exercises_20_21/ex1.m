%==========================================================================
%   TP :            Case study: Exercse 1
%   Contact:        ezequiel.gonzalezdebada@epfl.ch
%==========================================================================
classdef ex1
    %Class gathering the solutions of exercise 1. 
    methods (Static)
        %
        function varargout = getSystemParameters
            % PARAMETERS = getSystemParameters() returns a 5-elements
            % column vector containing the value of the system parameters 
            % and the linearization point. Specifically it should contain 
            % (in the presented order):
            %   - k : value of curvature of the reference path.
            %   - car_length [m]: car's length.  
            %   - sigma_v : coefficient characterizing the dynamic of the
            %   actuator tracking the speed reference. 
            %   - sigma_S : coefficient characterizing the dynamic of the
            %   actuator tracking the steering wheel's reference position. 
            %   - spReg : value of the speed the vehicle should drive at. 

            varargout = {[]};               
        end
        %
        function varargout = getLinealModelArrays(parameters)
            % [A,B,C,D] = getLinealModelArrays(PARAMETERS) returns the
            % matrices A,B,C,D characterizing the continuous-time linear
            % model of the system. The input *parameters* corresponds to 
            % the output of the method *getSystemParameters*.
            
            %%- Calculate arrays A,B,C,D
            A = [];
            B = [];
            C = [];
            D = [];
            
            varargout = {A,B,C,D};
        end        
        %
        function varargout = getDiscreteLinearModel(A,B,C,D,sampling_time,method)
            % [PHI,GAM] =
            % getDiscreteLinearModel(A, B, C, D, SAMPLING_TIME, METHOD)
            % returns the PHI and GAMMA matrices characterizing the
            % discrete-time linear model of the system given the matrices
            % A,B,C,D of the continuous-time linear model and the desired 
            % SAMPLING_TIME.
            %
            % Additionally, the input METHOD is a string
            % indicating the method that should be used to calculate 
            % the matrices PHI and GAMMA. It can take values 
            % - Euler : Euler approximation as discretization method. 
            % - c2d : use the matlab command c2d. 
            Phi = [];
            Gamma = [];
            
            if strcmp(method,'Euler')
                % Calculate the discrete-time linear model using the 
                % Euler approximation.
                
                % Phi = ;
                % Gamma = ;                
                
            elseif strcmp(method,'c2d')
                %%- Build continuous representation of the system with 'ss'
                
                % Mc = ss( );
                
                %%- Calculate the discrete-time linear model of the system 
                % using the command 'c2d'
                
                % Md = c2d( ); 
                
                %%- Extract from Md, the Phi and Gamma matrices. 
                
                % Phi = ;
                % Gamma = ;
                
                %%-set up output of the function 
            end
            varargout = {Phi, Gamma};
        end                
        %
        function varargout = getWorkingTrajectory(sampling_time, simulation_time, parameters)
            % [NOMINAL_TRAJECTORY_X, NOMINAL_TRAJECTORY_U] =
            % getWorkingTrajectory(SAMPLING_TIME, SIMULTAION_TIME,
            % PARAMETERS)  
            % outputs the NOMINAL_TRAJECTORY_X and NOMINAL_TRAJECTORY_U
            % given the SAMPLING_TIME between data points, the
            % SIMULATION_TIME up to which the trajectory has to be created,
            % and the vector PARAMETERS with the value sof tha system's
            % parameters.
            %
            % The outputs NOMINAL_TRAJECTORY_X, and NOMINAL_TRAJECTORY_U must
            % be arrays [t | \bar{x}] and [t | \bar{u}] 
            % whose first column corresponds to the timespan of
            % the data point, and following columns store the information
            % of the states and inputs at the corresponding time.
            %
            % The defined output trajectories are meant to be imported in
            % Simulink with the "From Workspace" block. If any
            % additional doubt regarding how the data should be formated,
            % read the information provided in the mentioned simulink block.
            %
            % todos
            % - create time vector. 
            % - create the nominal states trajectory output
            % - create the control inputs nominal trajectory output
            
            %%- create time vector
            time_vector = [];
            
            %%-create nominal state trajectory. 
            bar_x = [];
            nominal_trajectory_x = [time_vector, bar_x ];
            
            %%-create nominal control input trajectory. 
            bar_u = [];
            nominal_trajectory_u = [time_vector, bar_us];
            
            varargout = {nominal_trajectory_x, nominal_trajectory_u};
        end
        %
        function varargout = getInitialState(nominal_trajectory_x)
            %[X0, X0TILDE] = getInitialState(NOMINAL_TRAJECTORY_X)
            % returns the initial state X0 of the system and the
            % initial state X0TILDE of the linear models given the 
            % information on the exercise handout and the
            % NOMINAL_TRAJECTORY_X.
            %
            % The outputs should be column vectors. 
            %
            % Remember that by definition \tilde{x} = x - \overline{x}.
            
            
            %%- define the value of x0 for experiment 1
            x0_experiment_1 = [];
            
            %%- define the value of x0Tilde for experiment 1
            x0Tilde_experiment_1 = [];
            
            %including the different values for different experiments as a
            %cell
            x0 = {x0_experiment_1};
            x0Tilde = {x0Tilde_experiment_1};
            
            %set outputs of the function 
            varargout = {x0,x0Tilde};
        end
        %
        function varargout = getOpenLoopInputSignal(sampling_time, simulation_time)
            %[INPUT_CONTROL_ACTIONS_OPEN_LOOP] = getOpenLoopInputSignal(SAMPLING_TIME, SIMULATION_TIME)
            % outputs an input sequence to be applied in open loop to the 
            % models. The desired SAMPLING_TIME between data points as
            % well as the SIMULTION_TIME are provided. 
            %
            % As in the case of GETWORKINGTRAJECTORY function, the outputs
            % are meant to be used in Simulink with "From Workspace"
            % importing modules. If any additional doubt regarding how the
            % data should be structured, read the information provuded by
            % the mentioned simulink block. 
            %
            % todo:
            % - Declare an appropriate timespan vector. 
            % - Create the input_control_actions_open_loop array with the
            % sequence of control inputs to be applied in open loop. 
            %
            %
            % Notice: alternatively, this function can output a cell with
            % several arrays showing different control sequences to be
            % applied. This would make the provided script to run the
            % simulink mdel as many times as different control sequences
            % are gathered within the cell. Meaning that several
            % experiments can be set at once. 
            %
            
            %%- Create a time vector.
            %time_vector = ;
            
            %%- set the control sequence to be applied in open loop for the
            %%1st experiment. 
            time_vector = [];
            u1_open_loop = [];
            u2_open_loop = [];
            uOpenLoop_experiment_1 = [time_vector, u1_open_loop, u2_open_loop];
            
            %Include different values for the different experiments in a
            %cell.
            input_control_actions_open_loop = {uOpenLoop_experiment_1};
            
            %set output of the function
            varargout = input_control_actions_open_loop;
        end
        %
    end
    
end

