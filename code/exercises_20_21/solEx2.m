%==========================================================================
%   TP :            Case study: Exercse 2
%   Contact:        ezequiel.gonzalezdebada@epfl.ch
%==========================================================================
classdef solEx2
    %Class gathering the solutions of exercise 2. 
    methods(Static)
        % LQR-related methods
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function [Q1,Q2] = getLQRCostFunctArrays()
            % [Q1,Q2] = getLQRCostFunctArrays(NSTATES,NINPUTS) returns the
            % matrices Q1 and Q2 defining the LQR's cost function. 
            
            %Default values
            Q1 = eye(nStates)*10/2;
            %Q1(1,1) = 1e-4;
            Q1(1,1) = 1e-5;
            Q1(2,2) = 50;
            %Q1(2,2) = 30;
            
            % weighting inputs in the cost function
            Q2=eye(nInputs)*1; 
            Q2 = [1 0 ; 0 0.000020];
        end
        %        
        function [LQR_gain, SInf, singular_values] = getLQRGain(Phi,Gamma,Q1,Q2)
            % [LQR_K,SInf,SINGULAR_VALUES] =
            % getLQRGain(PHI,GAMMA,Q1,Q2) returns the
            % LQR control gain, the matrix SINF as well as the history of
            % the singular values of the S matrix over the iterations. The
            % method takes as inputs the PHI and GAMMA metrices describing 
            % the dicrete lineal model of the systme and the matrices Q1
            % and Q2 defining the cost function of the control problem. 
            % The last input is a string the specifies the method to be used. 
            %
            %TODO: implement algorithm in Section 7.1.7. 
            %
            % Hint: regarding the number of iterations for the calculation
            % of the control gain following Riccati approach, you should
            % find (by trial and error) a number of iterations that is
            % sufficiently large so that the S matrix converges.
            % This condition can be observed by keeping track
            % of the elements of the matrix or any other characteristics.
            % In this case, we are to use the singular values of S as it is
            % updated over the iterations.

            S=eye(size(Phi,1)); %-?-%
            % number of interations
            N_Iter=5000; %-?-%

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
            LQR_gain=inv(Q2+Gamma'*SInf*Gamma)*Gamma'*SInf*Phi; %-?-%
        end
        
        % OBSERVER-related methods
        function Cprim = alternativeSystemsOutputEquation(C)
            % Cprim, Dprim = alternativeSystemsOutputEquation(C)
            % takes as input the C array you initially describing the 
            % system's output equation and returns the Cprime array 
            % describing the alternative output equation we will consider 
            % to implement our observer. 
            Cprim = C([1 2 4 5],:);
        end
        %
        function n_states_not_observable = checkObservability(Phi,Cprime)
            % [N_STATES_NOT_OBSERVABLE] = checkObservability(PHI,Cprime)
            % checks whether the system described by Phi, 
            % Cprime is observable and returns the number of states that
            % cannot be observed. 
            %
            % Remember that the number of the non-onbservable states 
            % is calculated from the number of states and the rank
            % of the observability matrix. 
            %
            % HINT: To calculate the observability matrix, you can use the
            % matlab command 'obsv'. 
            Ob = obsv(Phi,Cprime);
            %
            % Compute the number of non-observable states of the system
            n_states_not_observable = length(Phi)-rank(Ob);            
        end        
        %
        function [L, selected_poles] = getObserverGain(Phi, Gamma, lqr_K, Cprime)
            % [L, SELECTED_POLES] =
            % getObserverGain(obj,PHI,GAMMA,LQR_K,Cprime) 
            % returns the observer matrix L as well as the poles
            % SELECTED_POLES characterizing the observation closed loop
            % given the discrete linear model defined by PHI and GAMMA, the
            % LQR control gain LQR_K and the matrix Cprime. 
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
            % 'place'. Read the help information and note that the
            % function is meant to calculate control gains. That
            % is it returns gains to impose certain poles assuming the
            % closed loop dynamics is given by (A-B*K). However, we are
            % using it to place the poles of the observer. Check and compare
            % the observer close loop, build a paralelism with the control
            % case, and introduce the needed modifications in the matrices
            % given to 'places'. 
            %
            % HINT4: Think about what happens when you transpose the closed
            % loop dynamic of the observer.
            gains = [1;1;1;1;1]*.999;
            selected_poles = gains .* eig(Phi-Gamma*lqr_K);
            L = place(Phi',Cprime',selected_poles)';
        end        %
        
        function x0Obs = getObserverInitialState(x_bar)
            % X0OBS = getObserverInitialState(X_BAR)
            % returns the initial guess X0OBS our observer will assign to 
            % the states it has to estimate. 
            %
            % The outputs should be column vectors, and does not need to be 
            % equal to the real initial state of the system. 
            %
            % Remember that by definition \tilde{x} = x - \overline{x} and
            % that the observer is based on the linear model. 
            x0Obs = zeros(5,1);
        end
        % OBSERVER-implementation related methods
        %
        function [AObs, BObs, CObs, DObs] = getObserverImplementationArrays(Phi, Gam, L, Cprime)
            % [AObs, BObs, CObs, DObs] = getObserverImplementationArrays(Phi, Gam, L, Cprime)
            % returns matrices AObs, BObs, CObs, DObs, needed to
            % implement the observer using state-space equations 
            %
            % x(k+1) = AObs x(k) + BObs u(k)
            % y(k) = CObs x(k) + DObs u(k)
            %
            % as it is illustrated in the solution of the exercises 6.6.1 -
            % 3.
            % Note that the xs and us used above are not the state and
            % input of our system, but rather the internal state and inputs
            % of the state-space block we will use to implement the
            % observer. 
            %
            %
            %nObsState = size(Cprime,1);
            nObsState = size(Cprime,1);
            nInputs = 2;
            CObs = eye(5);
            DObs = zeros(5,nObsState + nInputs);
            %
            AObs = Phi-L*C;
            BObs = [Gam L];
        end
    end

end

