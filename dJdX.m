function [dX] = dJdX(Px,Py,Psy,QR_code_Sx,R,Y)
Gx_H = 6037.5*(1./((QR_code_Sx-Px).^2+(121.5-Py).^2).^0.5);
Gx_C = 525*tan(atan((121.5-Py)./(QR_code_Sx-Px))-Psy);
Gx = [Gx_H;Gx_C];
dX=-2*(J(Px,Py,Psy,QR_code_Sx))'*(Y - Gx);

