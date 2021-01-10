function [J] = Hx(Px,Py,Phi,QR_code_Sx,QR_code_Sy)
J = zeros(2*length(QR_code_Sx),3);
J11 = ((6037.5*((QR_code_Sx-Px)./((QR_code_Sx-Px).^2+(QR_code_Sy-Py).^2).^1.5)));
J12 = ((6037.5*((QR_code_Sy-Py)./((QR_code_Sx-Px).^2+(QR_code_Sy-Py).^2).^1.5)));
J13 = zeros(length(QR_code_Sx),1);
J21 = ((525*secd(atand((QR_code_Sy-Py)./(QR_code_Sx-Px))-Phi).^2).*((QR_code_Sy-Py)./((QR_code_Sy-Py).^2 + (QR_code_Sx-Px).^2)));
J22 = ((-525*secd(atand((QR_code_Sy-Py)./(QR_code_Sx-Px))-Phi).^2).*((QR_code_Sx-Px)./((QR_code_Sy-Py).^2 + (QR_code_Sx-Px).^2)));
J23 = (-525*secd(atand((QR_code_Sy-Py)./(QR_code_Sx-Px))-Phi).^2);

J = [J11 J12 J13; J21 J22 J23];

% J(1,1) = ((6037.5*((QR_code_Sx-Px)/((QR_code_Sx-Px)^2+(QR_code_Sy-Py)^2)^1.5)));
% J(1,2) = ((6037.5*((QR_code_Sy-Py)/((QR_code_Sx-Px)^2+(QR_code_Sy-Py)^2)^1.5)));
% J(2,1) = ((525*secd(atand((QR_code_Sy-Py)/(QR_code_Sx-Px))-Phi)^2)*((QR_code_Sy-Py)/((QR_code_Sy-Py)^2 + (QR_code_Sx-Px)^2)));
% J(2,2) = ((-525*secd(atand((QR_code_Sy-Py)/(QR_code_Sx-Px))-Phi)^2)*((QR_code_Sx-Px)/((QR_code_Sy-Py)^2 + (QR_code_Sx-Px)^2)));
% J(2,3) = (-525*secd(atand((QR_code_Sy-Py)/(QR_code_Sx-Px))-Phi)^2);

end