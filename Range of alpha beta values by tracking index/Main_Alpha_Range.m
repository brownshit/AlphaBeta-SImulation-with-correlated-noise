clear all
clc

lambda = logspace(-8,4,30);
mu = logspace(-8,log10(4),30);
for i = 1 : size(lambda,2)
    %[alpha(i)] = Alpha_parameter_Range(lambda(i));     %useless
    [alpha1(i),beta1(i)] = Alpha_beta_parameter_range(lambda(i));
    for j = 1:size(mu,2)
        [alpha2(i,j),beta2(i,j)] = Correlated_Alpha_beta_parameter_range(lambda(i), mu(j));
        if imag(alpha2(i,j)) ~= 0 || imag(beta2(i,j)) ~= 0
            disp([i j])
        end
    end
end
figure(1)
%semilogx(lambda,alpha,'r-')
semilogx(lambda,alpha1,'r-')
hold on
semilogx(lambda,beta1,'b-')
%semilogx(lambda,Extended_alpha,'m-o')
%semilogx(lambda,Extended_alpha1,'k-*')
xlabel('Tracking Index log_{10}(\lambda^2=T^4\sigma_w^2/\sigma_n^2)');
ylabel('Gain','FontAngle','italic');
legend('\alpha in \alpha-\beta filter', '\beta in \alpha-\beta filter','Location', 'NorthWest');
title('Optimal position tracking gain \alpha , \beta versus tracking index \Lambda');
grid on;
%saveas(gcf,'Optimal_position_tracking_gain_alpha.png')
hold off


%{
figure(2)
semilogx(lambda,mu,alpha2,'r-')
hold on
semilogx(lambda,mu,beta2,'b-')
hold off
%semilogx(lambda,Extended_beta1,'b--')
xlabel('Tracking Index log_{10}(\lambda^2=T^4\sigma_w^2/\sigma_n^2)');
ylabel('Correlated Gain','FontAngle','italic');
legend('\alpha in correlated \alpha-\beta filter', '\beta in correlated \alpha-\beta filter','Location', 'NorthWest');
title('Optimal position tracking gain \alpha , \beta versus tracking index \Lambda , \mu');
grid on;
%}
%saveas(gcf,'Optimal_position_tracking_gain_beta','epsc')
%saveas(gcf,'Optimal_position_tracking_gain_beta.png')

% Lambda와 Mu를 meshgrid를 사용하여 2D 그리드로 변환
[Lambda, Mu] = meshgrid(lambda, mu);
% Z 축 데이터 생성 (예시: Lambda와 Mu의 합)
Z = 2*(Lambda + Mu);
% 3D 표면 plot 생성 및 색상 설정
figure(2)
surf(Lambda, Mu, alpha2, alpha2); % Z 값을 색상으로 사용
% 컬러바 추가
%colorbar;
% X축과 Y축을 로그 스케일로 설정
set(gca, 'XScale', 'log', 'YScale', 'log');
% 축 레이블 추가
xlabel('\lambda');
ylabel('\mu');
zlabel('alpha');