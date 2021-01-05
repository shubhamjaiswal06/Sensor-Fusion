function [Jacobian_Psy] = JPsy(Px,Py,Psy,QR_code_Sx)
Jacobian_Psy = (-525*sec(atan((121.5-Py)./(QR_code_Sx-Px))-Psy).^2);