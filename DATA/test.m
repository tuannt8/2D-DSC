clear;
close all;


%% Init
x = 0:0.1:2;
y = ones(size(x));
%y(x<=1) = 1;
%y(x>1) = -1;
%y([1, 3, 8 14]) = 0.5;

lambda = -ones(size(x));
r = 10;
dt = 0.01;
%%
phi = zeros(size(x));
phi(x<=1.5) = 0.6;

for i=1:500   

    %% Solve for c1 and c2
    xi1 = 1 - phi;
    xi2 = phi;
    dxi1 = ones(size(x))*-1;
    dxi2 = ones(size(x));
    A = [xi1*xi1' xi2*xi1'; xi1*xi2' xi2*xi2'];
    B = [xi1*y'; xi2*y'];
    
    C = A^-1*B;
    
%     dphix = diff(phi);
%     dxi1x = dxi1.* dphix;
%     dxi2x = dxi2.* dphix;

    %% Compute inten
    u = xi1*C(1) + xi2*C(2);
    du = dxi1*C(1) + dxi2*C(2);
    
    K = phi.*(phi-1);
    dK = phi*2 - 1;
     dL = 2*(u-y).*du + (lambda + r*K).*dK;
    % dL = (lambda + r*K).*dK;
    phi = phi - dL*dt;
    
    lambda = lambda + K*r*dt;
    
    
        %% Plot
    lg = cell(1,1);
    plot(x,phi, 'r');   lg(1) = {'phi'};
    hold on;
    plot(x,y); lg(2) = {'y'};
    plot(x,u, 'b--'); lg(3) = {'u'};
    plot(x,lambda); lg(4) = {'lambda'};
 %   r = r*1.01;
    legend(lg);%
    hold off;

end

    
%% Test
%plot(x,y);
hold;
plot(x,phi);