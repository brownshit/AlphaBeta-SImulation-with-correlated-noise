function [esti_pos] = ...
    Alpha_Beta_Filter(...
    PREV_esti_pos,...
    Gain,observation)
    % m. offline-tracking

    % m. mod    : 1 // uncorrelated noises
    % m. mod    : 2 //   correlated noises
    % m. Gain   : [alpha ; beta]
    dt = 0.01;
    F = [1,dt;0,1];
    H = [1,0];        

    pred_state = F*PREV_esti_pos;
    esti_pos = pred_state+Gain*(observation-H*pred_state);
end