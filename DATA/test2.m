%%
% One point

u = 0.4;
g = 0.7;
lambda = 1.0;
r = 10.0;
dt = 0.1;

while 1
    xi1 = 1-u; % -1
    xi2 = u; % 1
    A = [xi1*xi1 xi2*xi1; xi1*xi2 xi2 * xi2];
    B = [xi1*g; xi2*g];
    if(det(A)) < 0.01
        c = B(1) / (A(1,1) + A(1,2));
        C = [c;c];
    else
        C = A^-1*B;
    end
    
    a = xi1*C(1) + xi2*C(2)
    da = C(2) - C(1);
    
    H = u*(u-1);
    dH = 2*u - 1;
    dEu = 2*(a-g)*da + (lambda + r*H)*dH;
    u = u - dEu*dt
    lambda = lambda + r*H*dt;
end