function [dPsy] = dJdPsy(Px,Py,Psy,QR_code_Sx,R,Y)
Gx_C = 525*tan(atan((121.5-Py)./(QR_code_Sx-Px))-Psy);
dPsy=-2*(JPsy(Px,Py,Psy,QR_code_Sx))'*(Y - Gx_C);