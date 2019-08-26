%==========================================================================
%   TP :            Case study: Exercse 2
%   Contact:        ezequiel.gonzalezdebada@epfl.ch
%                   matin.macktoobian@epfl.ch
%==========================================================================
clear all;close all;

%% script inputs
%==========================================================================
    animate_flag = false;
    simulation_time = 100;
    sampling_time = 0.01;
    sampling_time_nominal_trajectory = sampling_time/2;
    
    ut = utilities;
    sol = ut.load_solution_class(2);
    
%% load the linearized discretized model (inherited from previous exercise)
%==========================================================================
    parameters = sol.getSystemParameters;
    if ut.stopCondition(parameters, 'getSystemParameters'), return; end
    
    [A,B,C,D] = sol.getLinealModelArrays(parameters);
    if ut.stopCondition(A, 'getLinealModelArrays'), return; end
        
    [Phi,Gam] = sol.getDiscreteLinearModel(A,B,C,D,sampling_time,'c2d');
    if ut.stopCondition(Phi, 'getDiscreteLinearModel'), return; end

%% Optimal Controller design
%==========================================================================

    % Get Q1 and Q2 matrices
    [Q1,Q2] = sol.getCostFunctArrays(size(Phi,1),size(Gam,2));
    if ut.stopCondition(Q1, 'getCostFunctArrays'), return; end
    
    %According to Q1 and Q2, calculate LQR gain iteratively. 
    [lqrRiccati, SInf, S_vs]  = sol.calculateLQRGain(Phi,Gam,Q1,Q2,'Riccati');
    if ut.stopCondition(lqrRiccati, 'calculateLQRGain - Riccati method'), return; end
    
    %Calculate LQR gain using Matlab command. 
    [KDlqr,SDlqr] = sol.calculateLQRGain(Phi,Gam,Q1,Q2,'Matlab');
    if ut.stopCondition(KDlqr, 'calculateLQRGain - Matlab method'), return; end

    %Compare solutions
    ut.printRelativeError(KDlqr, lqrRiccati, SDlqr, SInf, 'Comparing Programmatically vs Matlab','description','[relative error K_lqr | relative error S_inf ]')
    %Draw singular values of S over the iterations. 
    ut.drawSingularValEvol(S_vs);


%% Control implementation and simulation.
%==========================================================================
    % time vector (required to generate nominal trajectories)    
    chosen_path = sol.select_reference_path;
    if ut.stopCondition(chosen_path, 'select_reference_path'), return; end
    
    if ~isempty(chosen_path)
        load(chosen_path);
    end    

    % Load nominal state and control trajectory 
    [nominal_trajectory_x, nominal_trajectory_u] = sol.getWorkingTrajectory(sampling_time_nominal_trajectory, simulation_time, parameters);
    if ut.stopCondition(nominal_trajectory_x, 'getWorkingTrajectory'), return; end
    
    %Get initial state
    [x0,x0Tild,x0Obs] = sol.getInitialState(nominal_trajectory_x);
    if ut.stopCondition(x0, 'getInitialState'), return; end
    
%%  Run simulation with clean and fully measurable states. 
%==========================================================================
    % Initial value of noise, Cmes, simulation time, time vector.
    noiseMod = ut.defaultNoiseModule;  Cprime = eye(5);
    
    %Simulate and represent results
    sim(ut.complete_simulink_model_name('LQR_closed_Loop'));
    ax_cells = ut.plotResults(time, discrete_time, global_position, control_inputs, system_state, path_to_track, parameters, 'figure_name', 'LQR - non linear - clean and fully measurable state','animate',animate_flag);
    
%% Run simulation with noisy and fully measurable states. 
%==========================================================================
    %Load your noise module
    noiseMod = sol.getNoiseModule;
    if ut.stopCondition(noiseMod, 'getNoiseModule'), return; end
    
    %Simulate and represent results
    sim(ut.complete_simulink_model_name('LQR_closed_Loop'));
    ut.plotResults(time, discrete_time, global_position, control_inputs, system_state, path_to_track, parameters, 'figure_name', 'LQR - noisy measurable states','noisy_states', noisy_system_state,'animate',animate_flag);

%% Run simulation with noisy and partially measurable states. 
%==========================================================================
    Corig   = C; Cprime  = sol.getCprime;
    if ut.stopCondition(Corig, 'getCprime'), return; end

    sim(ut.complete_simulink_model_name('LQR_closed_Loop'));
    ut.plotResults(time, discrete_time, global_position, control_inputs, system_state, path_to_track, parameters, 'figure_name', 'LQR - noisy partially measurable states','animate',animate_flag);

%% Observer design and simulation
%==========================================================================
    % obtain CMes, 
    [CMes,CObs,DObs] = sol.getCArrayConsideringMeasurableStates(C);
    if ut.stopCondition(CMes, 'getCArrayConsideringMeasurableStates'), return; end

    % Check if the system is observable with the new outputs
    unobStates = sol.checkObservability(Phi,CMes);
    if ut.stopCondition(unobStates, 'checkObservability'), return; end
    
    % Design an observer using pole placement
    [L_options,poles_options] = sol.getObserverGain(Phi,Gam,KDlqr,CMes);
    if ut.stopCondition(L_options, 'getObserverGain'), return; end

    if ~iscell(L_options)
        L_options = {L_options};
        poles_options = {poles_options};
    end
    %eNow run simulations for all set up experiments

    for exp_id = 1:length(L_options) 

        L = L_options{exp_id};
        ut.printComplexNumbers(poles_options{exp_id}, 'label', 'Observer poles')

        sim(ut.complete_simulink_model_name('LQR_observer'));
        %%
        ut.compareObservedVsRealStates(time, discrete_time, observed_x,noisy_x,x, 'figure_name', 'comparison observed states')
        ut.plotResults(time, discrete_time, global_position, u, x, path_to_track, parameters, 'figure_name', 'LQR + observer', 'animate', animate_flag);
    end