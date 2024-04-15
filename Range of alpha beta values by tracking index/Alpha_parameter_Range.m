function [alpha] = Alpha_parameter_Range(lambda)
    syms a
    solutions=solve(4*(a^2)/(1-a)==lambda,a);
    selected_solution = [];
    for i = 1:length(solutions)
        if isAlways(0 <= solutions(i) <= 1)
            selected_solution = solutions(i);
            break; % 하나만 찾으면 for 루프 종료
        end
    end
    alpha = double(selected_solution);
end    