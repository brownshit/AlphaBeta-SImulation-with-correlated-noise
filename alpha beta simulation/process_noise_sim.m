function process_noise_sim()
    format long e;
    clear all;
    dt = 0.01;
    iteration = 1e5;
    noise_varience = [1e-3;1e-2;1e-1;1;1e1];
    F = [1,dt;0,1];
    %%
    % m. for Q
    Q = zeros(2,2,size(noise_varience,1));
    INSTANTANEOUS_process_noise = zeros(2,iteration,size(noise_varience,1));
    for var = 1:size(noise_varience,1)
        Exp_process_n = zeros(2,1);
        for iter = 1:iteration
            pres_state = [3;1/dt]; % m. true state

            prev_pos_2 = 1+sqrt(noise_varience(var,1))*randn;
            prev_pos_1 = 2+sqrt(noise_varience(var,1))*randn;
            prev_state = [prev_pos_1; (prev_pos_1 - prev_pos_2)/dt];
            INSTANTANEOUS_process_noise(:,iter,var) = ...
                pres_state - F*prev_state;
            Exp_process_n = Exp_process_n + INSTANTANEOUS_process_noise(:,iter,var);
        end
        Exp_process_n = Exp_process_n./iteration;
        
        for iter = 1:iteration
            pres_state = [3;1/dt]; % m. true state

            prev_pos_2 = 1+sqrt(noise_varience(var,1))*randn;
            prev_pos_1 = 2+sqrt(noise_varience(var,1))*randn;
            prev_state = [prev_pos_1; (prev_pos_1 - prev_pos_2)/dt];
            % m. overwrite process noise value.
            INSTANTANEOUS_process_noise(:,iter,var) = ...
                pres_state - F*prev_state;
            error_term = INSTANTANEOUS_process_noise(:,iter,var)-Exp_process_n;
            Q(:,:,var) = Q(:,:,var) + error_term*error_term';
        end
        % m. make Q as zero mean process noise cov. matrix
        Q(:,:,var) = Q(:,:,var)./iteration;
    end
    %%
    % m. for M
    % m. generate same size of measurement noise with process noise
    %    use INSTANTANEOUS_process_noise.
    Exp_M_correlation = zeros(2,size(noise_varience,1));
    INSTANTANEOUS_measurement_noise = zeros(1,iteration,size(noise_varience,1));
    for var = 1:size(noise_varience,1)
        INSTANTANEOUS_measurement_noise(1,:,var) = sqrt(noise_varience(var,1)).*randn(1,iteration);
        for iter = 1:iteration
            Exp_M_correlation(:,var) = Exp_M_correlation(:,var)+...
                (INSTANTANEOUS_process_noise(:,iter,var)-Exp_process_n)*...
                INSTANTANEOUS_measurement_noise(1,iter,var);
        end
        Exp_M_correlation(:,var) = Exp_M_correlation(:,var)./iteration;
    end
    %%
    % m. write as txt files.
    fileID = fopen('Q.txt', 'w');
    fprintf(fileID, '%.7f\n', Q);
    fclose(fileID);

    fileID = fopen('M.txt', 'w');
    fprintf(fileID, '%.7f\n', Exp_M_correlation);
    fclose(fileID);

    fileID = fopen('E_process.txt', 'w');
    fprintf(fileID, '%.7f\n', Exp_process_n);
    fclose(fileID);

    fileID = fopen('ProcessNoise_var0.1.txt', 'w');
    fprintf(fileID, '%.7f\n', INSTANTANEOUS_process_noise(:,:,3));
    fclose(fileID);
    
    fileID = fopen('MeasurementNoise_var0.1.txt', 'w');
    fprintf(fileID, '%.7f\n', INSTANTANEOUS_measurement_noise(:,:,3));
    fclose(fileID);
end