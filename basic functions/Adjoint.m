function adj = Adjoint(g)
% get adjoint matrix
% formula: [R    p_hat*R]
%          [0       R   ]

adj = [g(1:3,1:3), hat(g(1:3,4))*g(1:3,1:3);
        zeros(3,3), g(1:3,1:3)];
end