function [J_Matrix] = Fx(V,Phi)
J_Matrix = [1 0 -V*sind(Phi);0 1 V*cosd(Phi);0 0 1];
end
