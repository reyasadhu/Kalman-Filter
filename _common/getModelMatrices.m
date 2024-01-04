function [A,W,H] = getModelMatrices(scanTime)

A = diag(ones(4,1));
A(1,3) = scanTime;
A(2,4) = scanTime;

W = zeros(4,2);
W(1,1) = 0.5*scanTime^2;
W(2,2) = 0.5*scanTime^2;
W(3,1) = scanTime;
W(4,2) = scanTime;

H = zeros(2,4);
H(1,1) = 1;
H(2,2) = 1;

end

