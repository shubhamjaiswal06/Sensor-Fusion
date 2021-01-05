function [JacobianMatrix] = J(Px,Py,QR_code_Sx)


FirstCol = ((6037.5*((QR_code_Sx-Px)./((QR_code_Sx-Px).^2+(121.5-Py).^2).^1.5)));
        
SecondCol = ((6037.5*((121.5-Py)./((QR_code_Sx-Px).^2+(121.5-Py).^2).^1.5)));
    

JacobianMatrix = [FirstCol SecondCol];

