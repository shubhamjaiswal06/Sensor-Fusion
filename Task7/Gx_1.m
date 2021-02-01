function [Gx] = Gx_1(Sx,Sy,X)
Gx = 6037.5*(1/((Sx-X(1))^2+(Sy-X(2))^2)^0.5);
end