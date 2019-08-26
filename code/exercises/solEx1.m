%==========================================================================
%   TP :            Case study: Exercse 1
%   Contact:        ezequiel.gonzalezdebada@epfl.ch
%==========================================================================
classdef solEx1
    %Class gathering the solutions of exercise 1.     
    methods (Static)
        function varargout = getSystemParameters
            % PARAMETERS = getSystemParameters() returns a 5-elements
            % column vector PARAMETERS containing the parameters value
            % characterizing the system and the linearization point.
            % Specifically it should contain (in the presented order):
            %   - k : vlue of the reference curvature. 
            %   - car_length : length of the car in meters.  
            %   - sigma_a : sigma_a characterizing the dynamic of the
            %   actuator tracking the inputed acceleration. 
            %   - sigma_S : sigma_s characterizing the dynamic of the
            %   actuator tracking the inputed reference speed. 
            %   - spReg : value of the speed reference. 
            parameters = zeros(5,1);
            %
            parameters(1) = 1e-10;
            %parameters(1) = 0.02;
            %parameters(1) = 0.002;
            parameters(2) = 4;       
            parameters(3) = 5;
            parameters(4) = 1;
            parameters(5) = 3;
            
            varargout = {parameters};
            
        end
        %
        function varargout = getLinealModelArrays(parameters)
            % [A,B,C,D] = getLinealModelArrays(PARAMETERS) outputs the
            % matrices A,B,C,D characterizing the continuous_time linear
            % version of the system given the set of parameters values as
            % they where defined in function 'getSystemParameters'.
            %
            A = zeros(5);
            B = zeros(5,2);
            C = zeros(5);
            D = zeros(5,2);
            %
            A(1,2) = parameters(5)*parameters(1);
            A(1,4) = 1;
            A(2,3) = parameters(5);
            A(3,2) = -parameters(5)*parameters(1)^2;
            A(3,5) = parameters(5)*(1+parameters(1)^2*parameters(2)^2)/parameters(2);
            A(4,4) = -parameters(3);
            A(5,5) = -parameters(4);
            %
            B(4,1) = parameters(3);
            B(5,2) = parameters(4);
            %
            C = eye(5);
            
            varargout = {A, B, C, D};
        end        
        %
        function varargout = getDiscreteLinearModel(A,B,C,D,sampling_time,method)
            % [PHI,GAM] =
            % getDiscreteLinearModel(A, B, C, D, SAMPLING_TIME, METHOD)
            % outputs the PHI and GAMMA matrices characterizing the
            % discrete linear model of the system given the matrices
            % A,B,C,D of the continuous time linear description of the
            % system and the desired SAMPLING_TIME.
            %
            % Additionally, the input METHOD will contain a string
            % indicating the way the matrices PHI and GAMMA are wanted to
            % be calculated. Such an input can take values 
            % - Euler : Euler approximation as discretization method. 
            % - Psi : Use the progrmmatically algorithm presented in the
            % note course which makes use of the intermediary matrix Psi.
            % - c2d : use the matlab command c2d. 
            %
            if strcmp(method,'Euler')
                
                Phi = (sampling_time*A+eye(5));
                Gamma = sampling_time*B;   
                
            elseif strcmp(method,'Psi')
                
                %Dimension of Psi
                n = size(A,1); 
                %Psi matrix initialization
                Psi = zeros(n);
                m = 3;
                for k=0:m
                    Psi = Psi+(A*sampling_time)^k/factorial(k+1);
                end
                % Calculate matrices Phi and Gam
                Phi = eye(n)+A*sampling_time*Psi;
                Gamma = Psi*sampling_time*B;
                
            elseif strcmp(method,'c2d')
                % Continuous representation of the system with 'ss'
                Mc=ss(A,B,C,D);
                % Calculation of discretized system using 'c2d'
                Md = c2d(Mc,sampling_time,'zoh'); %-?-%        
                Phi = Md.a;
                Gamma = Md.b;
            end
            
            varargout = {Phi, Gamma};
        end                
        %
        function varargout = getWorkingTrajectory(sampling_time, simulation_time, parameters)
            % [NOMINAL_TRAJECTORY_X, NOMINAL_TRAJECTORY_U] =
            % getWorkingTrajectory(SAMPLING_TIME, SIMULTAION_TIME,
            % PARAMETERS)  
            % outputs the NOMINAL_TRAJECTORY_X and NOMINAL_TRAJECTORY U
            % given the SAMPLING_TIME between data points, the
            % SIMULATION_TIME up to which the trajectory has to be created,
            % and the vector PARAMETERS with the value sof tha system's
            % parameters.
            %
            % Outputs NOMINAL_TRAJECTORY_X, and NOMINAL_TRAJECTORY_U must
            % be arrays whose first collumn correspond to the time span of
            % the data point, and successive columns store the information
            % of the states and inputs, correspondingly.
            %
            % The defined output trajectories are meant to be used in
            % Simulink with "From Workspace" importing module. If any
            % additional doubt regarding how the data should be structured,
            % read the information provuded by the mentioned simulink block.
            %
            % todo
            % - create time vector. 
            % - create the nominal states trajectory output
            % - create the control inputs nominal trajectory output
            
            time_vector = (0:sampling_time/100:simulation_time)';
            nominal_trajectory_x = ...
                [time_vector, ...
                ...%sin(parameters(5)*parameters(1)*time_vector)/parameters(1),...
                ...%-cos(parameters(5)*parameters(1)*time_vector)/parameters(1),...
                ...%parameters(5)*parameters(1)*time_vector,...
                parameters(5).*time_vector,...
                time_vector*0,...
                time_vector*0,...
                time_vector*0+parameters(5),...
                time_vector*0+atan(parameters(1)*parameters(2))];
            nominal_trajectory_u = [time_vector, nominal_trajectory_x(:,5), nominal_trajectory_x(:,6)];
            
            varargout = {nominal_trajectory_x, nominal_trajectory_u};
        end
        %
        function varargout = getInitialState(nominal_trajectory_x)
            %[X0, X0TILDE] = getInitialState(NOMINAL_TRAJECTORY_X)
            % returns the initial state X0 of the non linear system as the
            % initial state X0TILDE of the linearized model of the system,
            % given the information on the exercise handout and the
            % NOMINAL_TRAJECTORY_X.
            %
            % The outputs should be column vectors. 
            %
            % Remember that by definition \tilde{x} = x - \overline{x}.
            x0_exp1 = [0;0;0;nominal_trajectory_x(1,4);0];
            x0Tilde_exp1 = x0_exp1 - nominal_trajectory_x(1,2:end)';
            
            x0_exp2 = x0_exp1;
            x0Tilde_exp2 = x0Tilde_exp1;
            
            x0_exp3 = x0_exp1;
            %x0_exp3(3) = -pi/20;
            x0Tilde_exp3 = x0_exp3 - nominal_trajectory_x(1,2:end)';
            
        
            x0 = {x0_exp1, x0_exp2, x0_exp3};
            x0Tilde = {x0Tilde_exp1, x0Tilde_exp2, x0Tilde_exp3};
            
            varargout = {x0, x0Tilde};
        end
        %
        function varargout = getOpenLoopControlSignal(sampling_time, simulation_time)
            %[INPUT_CONTROL_ACTIONS_OPEN_LOOP] = getOpenLoopControlSignal(SAMPLING_TIME, SIMULATION_TIME)
            % outputs a sequence of control signals to be applied in open
            % loop to the system, which will be then simulated. In order to
            % do that, the desired SAMPLING_TIME between data points as
            % well as the SIMULTION_TIME is provided. 
            %
            % As int he case of GETWORKINGTRAJECTORY function, the outputs
            % are meant to be used in Simulink with "From Workspace"
            % importing module. If any additional doubt regarding how the
            % data should be structured, read the information provuded by
            % the mentioned simulink block. 
            %
            % todo:
            % - Declare en appropriate time span vector. 
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
            time_vector = (0:sampling_time/100:simulation_time)';
            uOpenLoop_experiment_1 = [time_vector, 13 + 5*sin(time_vector/10)*0, 0.1*sin(time_vector)*0];
            uOpenLoop_experiment_2 = [time_vector, 13 + 5*sin(time_vector/10)*0, 0.15*sin(pi/2 + time_vector)];
            uOpenLoop_experiment_3 = [time_vector, 13 + 5*sin(time_vector/10)*0, 0.3*sin(pi/2 + time_vector)];

            input_control_actions_open_loop = {uOpenLoop_experiment_1, uOpenLoop_experiment_2, uOpenLoop_experiment_3};
            %input_control_actions_open_loop = {uOpenLoop_experiment_2};
            varargout = {input_control_actions_open_loop};
        end
    end
    
end

