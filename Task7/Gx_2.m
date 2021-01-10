function [Gx] = Gx_2(Sx,Sy,X)
Gx = 525*tand(atand((Sy-X(2))/(Sx-X(1)))-X(3));
end