function MMSE_performance()
    clc; format long e; close all;
    dt = 0.01;
    noise_varience = [1e-3;1e-2;1e-1;1;1e1];

    % [This simulation code is simplified version.]
    % we only process these codes in 1 dimension circumstances.


    %m. simulation iteration number.
    Tot_iteration = 1e4;

    % m. Assume that our circum; Uniform linear motion
    % m. Position : [0;0]; init // [19,19]; end
    position_end = 20; 

    %compare Error by each varience
    KF_error = zeros(1,size(noise_varience,1));
    AB_error_uncor = zeros(1,size(noise_varience,1));
    AB_error_cor = zeros(1,size(noise_varience,1));
    %%
    % m. Recall process noise Q_k and correlation M_k

    %Get Q and M from txt files.
    Q = zeros(2,2,5);       %use this Q value on KF
    fileID = fopen('Q.txt', 'r');
    Q_raw = fscanf(fileID, '%f');
    fclose(fileID);
    for i1=1:1:size(noise_varience,1)
        for i2=1:1:2
            for i22=1:1:2
               Q(i2,i22,i1) = Q_raw((i1-1)*4+(i2-1)*2+i22,1);
            end
        end
    end
    % m. if we need to get some performance of KF, re calc of Q with
    %    specific weight, and simulate.

    M = zeros(2,1,5);
    fileID = fopen('M.txt', 'r');
    M_raw = fscanf(fileID, '%f');
    fclose(fileID);
    for i1=1:1:size(noise_varience,1)
        for i2=1:1:2
            index = i2+(i1-1)*2;
            M(i2,1,i1) = M_raw(index,1);
        end
    end

%%
    for var = 1:size(noise_varience,1)
        % m. init cov mat. for 2nd step of Kalman Filter
        P_init = init_COV_gen(var);
        R = noise_varience(var,1);  %for KF
        %%
        % m. should check gain value is not complex number.
        % m. Take Q(1,1,var) for (T^2/2)*sig_omega^2
        sig_w = (2/dt^2)*sqrt(Q(1,1,var)); sig_n = sqrt(noise_varience(var,1));
        lambda1 = (dt^2*sig_w)/sig_n; 
        lambda2 = (dt^2*sig_w)/sqrt(sig_n^2+M(1,1,var)); mu = 2*dt*M(2,1,var)/(sig_n^2+M(1,1,var));

        alpha1 = -1*(lambda1^2+8*lambda1-(4+lambda1)*sqrt(lambda1^2+8*lambda1))/8;
        beta1 = (lambda1^2+4*lambda1-lambda1*sqrt(lambda1^2+8*lambda1))/4;
        
        temp2 = lambda2^2+8*sqrt(1-mu/4)*lambda2-4*mu;
        alpha2 = -1*(temp2-(4*sqrt(1-mu/4)+lambda2)*sqrt(temp2))/8;
        beta2 = (lambda2^2+4*sqrt(1-mu/4)*lambda2-lambda2*sqrt(temp2))/4;
        
        Gain1 = [alpha1; beta1];
        Gain2 = [alpha2; beta2];
%%

        for iter = 1:Tot_iteration
            % m. storage for each iteration
            true_pos = zeros(1,position_end);

            KF_esti_state = zeros(2,position_end);       %[KF]
            KF_esti_cov = zeros(2,2,position_end);      %[KF]
            Kalman_Gain = zeros(2,position_end);

            A_B_esti_state_uncor = zeros(2,position_end);%[AB_uncor]
            A_B_esti_state_cor = zeros(2,position_end);  %[AB_cor]

            for mov_pnt = 2:position_end
                true_pos(mov_pnt) = mov_pnt-1;
                % m. generate noise to make circum
                observation = true_pos(mov_pnt) + sqrt(noise_varience(var,1))*randn;  %AWGN
                
                if mov_pnt <= 3 % m. All filtering algorithm works After step 3( Begin from step 4 ).
                    KF_esti_state(:,mov_pnt) = [observation;(observation-KF_esti_state(1,mov_pnt-1))/dt];
                    A_B_esti_state_uncor(:,mov_pnt) = [observation;(observation-A_B_esti_state_uncor(1,mov_pnt-1))/dt];
                    A_B_esti_state_cor(:,mov_pnt) = [observation;(observation-A_B_esti_state_uncor(1,mov_pnt-1))/dt];

                    if mov_pnt == 3
                        % m. initialization of KF Cov matirx
                        KF_esti_cov(:,:,mov_pnt) = P_init;
                    end
                else
                    % m. step 4 ~
                    % [KF]
                    [KF_esti_state(:,mov_pnt), KF_esti_cov(:,:,mov_pnt), Kalman_Gain(:,mov_pnt)] = ...
                        Kalman_Filter...
                        (KF_esti_state(:,mov_pnt-1), KF_esti_cov(:,:,mov_pnt-1),...
                        Q(:,:,var),noise_varience(var,1),observation);
                    
                    % [AB_uncor]
                    [A_B_esti_state_uncor(:,mov_pnt)] = ...
                        Alpha_Beta_Filter(...
                        A_B_esti_state_uncor(:,mov_pnt-1),Gain1,observation);

                    % [AB_cor]
                    [A_B_esti_state_cor(:,mov_pnt)] = ...
                        Alpha_Beta_Filter(...
                        A_B_esti_state_cor(:,mov_pnt-1),Gain2,observation);
                    
                    % m. error accumulation
                        KF_error(1,var) = KF_error(1,var) + abs(KF_esti_state(1,mov_pnt)-true_pos(1,mov_pnt));
                        AB_error_uncor(1,var) = AB_error_uncor(1,var) + abs(A_B_esti_state_uncor(1,mov_pnt)-true_pos(1,mov_pnt));
                        AB_error_cor(1,var) = AB_error_cor(1,var) + abs(A_B_esti_state_cor(1,mov_pnt)-true_pos(1,mov_pnt));
                end
            end
        end
        % m. Averaging
        KF_error(1,var) = KF_error(1,var)/(Tot_iteration*(position_end-3));
        AB_error_uncor(1,var) = AB_error_uncor(1,var)/(Tot_iteration*(position_end-3));
        AB_error_cor(1,var) = AB_error_cor(1,var)/(Tot_iteration*(position_end-3));
    end
    % m. performance of MMSE viewer.
    figure(1)
    semilogx(noise_varience',KF_error,'ko-')
    hold on
    semilogx(noise_varience',AB_error_uncor,'^-')
    semilogx(noise_varience',AB_error_cor,'^-')
    hold off
    xlabel('log_{10}(\sigma_{n}^{2})','FontAngle','italic')
    ylabel('Error','FontAngle','italic')
    legend('Kalman Filter','\alpha - \beta filter_{uncorrelated noise}','\alpha - \beta filter_{correlated noise}')
    title('MSE Error Performance','FontAngle','italic')
end