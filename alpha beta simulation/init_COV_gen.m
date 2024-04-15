function [init_cov] = init_COV_gen(var)
    % m. For cov mat of KF 2nd step
    format long e;
    dt = 0.01;
    iteration = 1e5;
    
    F = [1,dt;0,1];
    %%
    % m. for Q
    init_cov = zeros(2,2);
    
    Exp_epsilon = zeros(2,1);
    for iter = 1:iteration
        true_state = [3;1/dt]; % m. true state

        pos_2 = 1+sqrt(var)*randn;
        pos_1 = 2+sqrt(var)*randn;
        observ_state = [pos_1; (pos_1 - pos_2)/dt];
        epsilon = ...
            observ_state - true_state;
        Exp_epsilon = Exp_epsilon + epsilon;
    end
    Exp_epsilon = Exp_epsilon./iteration;
    
    for iter = 1:iteration
        true_state = [3;1/dt]; % m. true state

        pos_2 = 1+sqrt(var)*randn;
        pos_1 = 2+sqrt(var)*randn;
        observ_state = [pos_1; (pos_1 - pos_2)/dt];
        
        epsilon = ...
            true_state - F*observ_state;
        error_term = epsilon-Exp_epsilon;
        init_cov(:,:) = init_cov(:,:) + error_term*error_term';
    end
    init_cov(:,:) = init_cov(:,:)./iteration;
end