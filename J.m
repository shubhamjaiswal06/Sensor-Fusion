function [JacobianMatrix] = J(Px,Py,Psy,QR_code_Sx)
% JacobianMatrix = zeros(14,3) ;

FirstCol = [(6037.5*((QR_code_Sx-Px)./((QR_code_Sx-Px).^2+(121.5-Py).^2).^1.5));
            (525*((sec(atan((121.5-Py)./(QR_code_Sx-Px))-Psy).^2).*((121.5-Py)./((QR_code_Sx-Px).^2+(121.5-Py).^2))))];
        
SecondCol = [(6037.5*((121.5-Py)./((QR_code_Sx-Px).^2+(121.5-Py).^2).^1.5));
             (-525*((sec(atan((121.5-Py)./(QR_code_Sx-Px))-Psy).^2).*((QR_code_Sx-Px)./((QR_code_Sx-Px).^2+(121.5-Py).^2))))];
         
ThirdCol = [0;0;0;0;0;0;0;(-525*sec(atan((121.5-Py)./(QR_code_Sx-Px))-Psy).^2)];

JacobianMatrix = [FirstCol SecondCol ThirdCol];

