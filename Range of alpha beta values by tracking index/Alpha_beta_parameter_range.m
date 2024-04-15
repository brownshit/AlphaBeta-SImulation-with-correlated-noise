function [alpha, beta] = Alpha_beta_parameter_range(lambda)
    syms a  %변수 생성.
    b=2*(2-a)-4*sqrt(1-a); % or a^2/(2-a)
    solutions=solve(b^2/(1-a)==lambda^2,a); %solutions of a
    %위의 결과의 근이 총 4개가 나오게 되는 이유 : 4th order equation of a

    selected_solution = [];
    for i = 1:length(solutions)
        if isAlways(0 <= solutions(i) <= 1)
            selected_solution = solutions(i);
            break; % 하나만 찾으면 for 루프 종료
        end
    end
    alpha = double(selected_solution);
    beta = double(2*(2-alpha)-4*sqrt(1-alpha)); 
end