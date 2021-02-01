function [J] = Hx1(Px,Py,QR_code_Sx,QR_code_Sy)
J = zeros(length(QR_code_Sx),2);
J11 = ((6037.5*((QR_code_Sx-Px)./((QR_code_Sx-Px).^2+(QR_code_Sy-Py).^2).^1.5)));
J12 = ((6037.5*((QR_code_Sy-Py)./((QR_code_Sx-Px).^2+(QR_code_Sy-Py).^2).^1.5)));

J = [J11 J12];
end