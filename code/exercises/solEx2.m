%==========================================================================
%   TP :            Case study: Exercse 2
%   Contact:        ezequiel.gonzalezdebada@epfl.ch
%==========================================================================
classdef solEx2
    %Class gathering the solutions of exercise 2. 
    methods(Static)
        %
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
            parameters(2) = 4;       
            parameters(3) = .8;
            parameters(4) = 10;
            parameters(5) = 5;
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
            A(3,5) = parameters(5)*(1+parameters(1)^2*parameters(2)^2)/(16*parameters(2));
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
            % 3.4 - Continuous representation of the system with 'ss'
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
        function varargout = getCostFunctArrays(nStates,nInputs)
            % [Q1,Q2] = getCostFunctArrays(NSTATES,NINPUTS) returns the
            % matrices Q1 and Q2 defining the cost function leading to the
            % design of a LQR control gain, given the number of states
            % NSTATES and inputs NINPUTS. 
            %
            % The information regarding the values to be given to the
            % output matrices can be found in the exercise handout. 
            
            %Default values
            Q1 = eye(nStates)*10/2;
            %Q1(1,1) = 1e-4;
            Q1(1,1) = 1e-5;
            Q1(2,2) = 50;
            %Q1(2,2) = 30;
            
            % weighting inputs in the cost function
            Q2=eye(nInputs)*1; 
            Q2 = [1 0 ; 0 0.000020];
            %Q2(2,2) = 20;
            
            varargout = {Q1,Q2};
        end
        %        
        function varargout = calculateLQRGain(Phi,Gamma,Q1,Q2, method)
            % [LQR_K,SInf,SINGULAR_VALUES] =
            % solveLQRthroughRiccati(PHI,GAMMA,Q1,Q2,'Riccati') returns the
            % LQR control gain, the matrix SINF as well as the history of
            % the singular values of the S matrix over the iterations,
            % given the PHI and GAMMA metrices describing the dicrete
            % lineal model of the systme as well as the matrices Q1 and Q2
            % defining the cost function of the control problem. The last
            % input is a string the specifies the method to be used. 
            %
            % [LQR_K,SInf] =
            % solveLQRthroughRiccati(PHI,GAMMA,Q1,Q2,'Matlab') returns the 
            % value of the LQR control gain and SINF when calculated with
            % the matlab command 'dlqr'. For more details regarding how to
            % use 'dlqr' command, please check the matlab 'help' or 'doc'
            % information.
            %
            %TODO: implement algorithm in Section 7.1.7. 
            %
            % Hint: regarding the number of iterations for the calculation
            % of the control gain following Riccati approach, you should
            % find (by trial and error) a number of iterations that is
            % sufficiently large so that the SINF matrix converges to an
            % unique value. This condition can be observed by keeping track
            % of the elements of the matrix or any other characteristics.
            % In this case, we are to use the singular values of S as it is
            % updated over the iterations.
            %
            % Note: as the number of ouputs changes depending on the
            % requested method, the output will be defined as a cell
            % containing the appropriate matrices according to the case. 
            
            if strcmp(method,'Riccati')

                S=eye(size(Phi,1)); %-?-%
                % number of interations
                N_Iter=12000; %-?-%

                % Vector of singular values (to control the convergence of method)
                singular_values=zeros(N_Iter,size(S,1)); %-!-%

                %Ricatti method, convergence
                for i=1:N_Iter
                    %Ricatti Equation 
                    S=Q1+Phi'*S*Phi-Phi'*S*Gamma*inv(Q2+Gamma'*S*Gamma)*Gamma'*S*Phi;
                    %Singular values
                    singular_values(i,:) = svd(S);
                end
                SInf=S;
                KLqrRiccati=inv(Q2+Gamma'*SInf*Gamma)*Gamma'*SInf*Phi; %-?-%
                varargout = {KLqrRiccati,SInf,singular_values};
                
            elseif strcmp(method,'Matlab')
                
                [KDlqr,SDlqr] = dlqr(Phi,Gamma,Q1,Q2);                
                varargout = {KDlqr,SDlqr};
            end
        end
        %        
        function varargout = selectReferencePath
            % OUTPUT = select_reference_path returns a string with the name
            % of the mat file containing the path to be tracked that is
            % wanted to be used. 
            options = {'circle', 'path_1', 'path_2', 'path_3'};
            output = options{3};
            
            varargout = {output};
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
                time_vector*0+16*atan(parameters(1)*parameters(2))];
            nominal_trajectory_u = [time_vector, nominal_trajectory_x(:,5), nominal_trajectory_x(:,6)];
            
            varargout = {nominal_trajectory_x, nominal_trajectory_u};
        end
        %
        function varargout = getInitialState(ntX)
            %[X0, X0TILDE, XOBS] = getInitialState(NOMINAL_TRAJECTORY_X)
            % returns the initial state X0 of the non linear system, the
            % initial state X0TILDE of the linearized model of the system,
            % as well as the initial value of the state for the observer,
            % given the,  information on the exercise handout and the
            % NOMINAL_TRAJECTORY_X.
            %
            % The outputs should be column vectors. 
            %
            % Remember that by definition \tilde{x} = x - \overline{x} and
            % that the observer is designed considering the linear model of
            % the system.
            
            x0 = [0, 0, 0, 0,0]';
            %x0 = [0, 0, 0, 3, atan(1e-10*4)]';
            
            x0Tilde = x0 - ntX(1,2:end)';
            x0Obs = zeros(5,1);
            
            varargout = {x0,x0Tilde,x0Obs};
        end        
        %        
        function varargout = getNoiseModule
            % [NOISE] = getNoiseModule
            % generates a structure with fields 
            % - mean
            % - std_deviation
            % - seed
            % defining the noise to be introduced in the corresponding
            % simulations. The name of the structure fields are
            % self-explanatory, nontheless, for further information
            % regarding their meaning, it is used in the 'noise' block in
            % the 'LQR_observer' and 'LQR_closed_loop' simulink models.
            % Check the information of such a module out for more details.
            %
            % Notice 1: Notice that every field of the structure is a
            % vector with as many elements as states.
            %
            % Notice 2: The values that are proposed in the exercise
            % handout are just a recomendation. Feel free to play with
            % these values, observe the effect they have, and reason why.
            % Take into account the range of values that the state affected
            % by each term of the noise fields can take when playing around
            % with the values.
            % 
            %
            %noiseMod = zeros(5,1);
            noise.mean = [0; 0; 0; 0; 0];
            noise.std_deviation = [1; 1; deg2rad(10); 2; 0];
            
            noise.mean = [0; 0; 0; 0; 0];
            %noise.std_deviation = [1; 1; .2; 0.1; 0.1]/5;
            noise.std_deviation = [.5, .5, .04, 1, .02];
            noise.std_deviation = [.2, .2, .04, .02, .02];
            noise.seed = randi(10,5,1);
            noise.freq = 50;
            varargout = {noise};
        end        
        %        
        function varargout = getWorkingSensor
            % working_sensor = getWorkingSensor(obj)
            % returns a column vector, whose elements (i,1) represents 
            % whether the sensor measuring the state i works correctly
            % (value 1), or not (value 0);
            %
            % It must have as many elements as number of states.
            %
            varargout = {[1;1;0;1;1]};
        end
        %        
        function varargout = updateCArray(C)
            % C = updateCArray(C)
            % takes as input the C array you originally defined in function
            % 'getLinealModelArrays' and returns a new C array to make the
            % output of the system contain only a subset of states. 
            
            %            
            varargout = {C([1 2 4 5],:)};
            %varargout = {C([1 3 4 5],:)};
        end
        %
        function varargout = getArrayDefiningObserverOutput(C)
            % [COBS,DOBS] = getArrayDefiningObserverOutput(C)
            % returns matrices COBS, and DOBS, which are needed to
            % implement the observer and have to be calculated from C. 
            %
            % The output matrices COBS and DOBS are used in the 'Observer'
            % block in the simulink model 'LQR_observer'. Go and explore
            % such a module for some hints regarding the dimensions of the
            % matrices.  
            %
            % Moreover, CObs and DObs should be constructed following the
            % approach presented in the solution of exercise 6.6.1 - 3
            %
            %
            %nObsState = size(C,1);
            nObsState = size(C,1);
            nInputs = 2;
            CObs = eye(5);
            DObs = zeros(5,nObsState + nInputs);
            %
            varargout = {CObs,DObs};
        end
        %
        function varargout = checkObservability(Phi,CMes)
            % [N_STATES_NOT_OBSERVABLE] = checkObservability(PHI,CMES)
            % checks whether the system described by Phi, 
            % CMes is observable and returns the number of states that
            % cannot be observed. 
            %
            % Remember that the number of the states that cannot be
            % observed will be given by the number of states and the rank
            % of the observability matric. 
            %
            % HINT: To calculate the observability matrix, you can use the
            % matlab command 'obsv'. 
            %
            % NOTICE: CMes is a version of the C matrix where some rows
            % have been ommited, which would be connected with the
            % assumption that the states corresponding to the ommited rows
            % are not measurable. 
            Ob = obsv(Phi,CMes);
            %
            % Compute the number of non-observable states of the system
            n_states_not_observable = length(Phi)-rank(Ob);            
            fprintf('\nObservability check: %d states cannot be observed with this configuration\n',n_states_not_observable);
            
            varargout = {n_states_not_observable};
        end
        %
        function varargout = getObserverGain(Phi,Gamma,lqr_K,c_meassurable)
            % [L, SELECTED_POLES] =
            % getObserverGain(obj,PHI,GAMMA,LQR_K,c_messurable) 
            % returns the observer matrix L as well as the poles
            % SELECTED_POLES characterizing the closed observation loop
            % given the discrete linear model defined by PHI and GAMMA, the
            % LQR control gain LQR_K and the matrix C_meassurable, that is
            % the version of the C matrix where some states are ommited. 
            %
            % 
            % Hint1: as a rule of thumb, the poles of the observer are 
            % calculated as some percentage of the poles that characterize 
            % the close loop dynamic. 
            %
            % Hint2: remember that the eigenvalues of the control closed
            % loop are given by eig(\Phi - \Gamma * lqr_K)
            %
            % Hint3: to calculate the observer gain, use the matlab command
            %'place'. Read the help information and notice that, the
            %function is meant to be used to calculate control gains, that
            %is it returns gains to impose certain poles assuming the
            %closed loop dynamics is given by (A-B*K). However, we are
            %using it to place the poles of the observer. Check and compare
            %the observer close loop, build a paralelism with the control
            %case, and introduce the needed modifications in the matrices
            %given to 'places'. 
            %
            % HINT4: Think about what happens when you transpose the closed
            % loop dynamic of the observer.
            % 
            % Notice 1: The outputs of this function can be defined as
            % cells gathering as many instances of the observer gain as
            % experiments are wanted to be run.
            %
            
            gains = [1;1;1;1;1]*.5;
            selected_poles = gains .* eig(Phi-Gamma*lqr_K);
            L = place(Phi',c_meassurable',selected_poles)';
            
            varargout = {L, selected_poles};
        end
    end

end

