function [dPy] = dJdPy(Px,Py,H_mean,QR_code_Sx,H_var)
dPy = sum(-2*((H_mean - 6037.5*(1./((QR_code_Sx-Px).^2+(121.5-Py).^2).^0.5))).*(6037.5*((121.5-Py)./((QR_code_Sx-Px).^2+(121.5-Py).^2).^1.5)));