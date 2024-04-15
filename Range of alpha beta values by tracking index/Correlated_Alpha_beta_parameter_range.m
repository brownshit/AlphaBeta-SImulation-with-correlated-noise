function [alpha, beta] = Correlated_Alpha_beta_parameter_range(lambda, mu)
    %{
    syms a
    b=2*(2-a)-4*sqrt(1-a); % or a^2/(2-a)
    solutions=solve(b^2/(1-a)==lambda^2,a);
    selected_solution = [];

    for i = 1:length(solutions)
        if isAlways(0 <= solutions(i) <= 1)
            selected_solution = solutions(i);
            break; % 하나만 찾으면 for 루프 종료
        end
    end
    alpha = double(selected_solution);
    beta = double(2*(2-alpha)-4*sqrt(1-alpha)); 

    %}

    %uncorrelated alpha beta boundary
    alpha = -(lambda^2+8*sqrt(1-mu/4)*lambda-4*mu-...
        (4*sqrt(1-mu/4)+lambda)*sqrt(lambda^2+8*sqrt(1-mu/4)*lambda-4*mu))/8;
    beta = (lambda^2+4*sqrt(1-mu/4)*lambda-...
        lambda*sqrt(lambda^2+8*sqrt(1-mu/4)*lambda-4*mu))/4;
end