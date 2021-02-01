function [J] = Hx(Px,Py,Phi,QR_code_Sx,QR_code_Sy)
J = zeros(2,3);
J(1,1) = mean((6037.5*((QR_code_Sx-Px)./((QR_code_Sx-Px).^2+(QR_code_Sy-Py).^2).^1.5)));
J(1,2) = mean((6037.5*((QR_code_Sy-Py)./((QR_code_Sx-Px).^2+(QR_code_Sy-Py).^2).^1.5)));
J(2,1) = mean((525*sec(atan((QR_code_Sy-Py)./(QR_code_Sx-Px))-Phi).^2).*((QR_code_Sy-Py)./((QR_code_Sy-Py).^2 + (QR_code_Sx-Px).^2)));
J(2,2) = mean((-525*sec(atan((QR_code_Sy-Py)./(QR_code_Sx-Px))-Phi).^2).*((QR_code_Sx-Px)./((QR_code_Sy-Py).^2 + (QR_code_Sx-Px).^2)));
J(2,3) = mean(-525*sec(atan((QR_code_Sy-Py)./(QR_code_Sx-Px))-Phi).^2);

end