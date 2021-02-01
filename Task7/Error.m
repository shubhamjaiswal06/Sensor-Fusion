function [Err] = Error(j,CameraTimeStamp_SingleValue,Time_Camera,H_i,Cx_i,Sx,Sy,X)
t = CameraTimeStamp_SingleValue(j);
k = 1;
% Center_cor_y(k) = Global_cor_y(QR_No(1),Data_global);
for i = 1:length(Time_Camera)
    if t == Time_Camera(i)
         Err1(k,1) = H_i(i) - Gx_1(Sx(k),Sy(k),X);
         Err2(k,1) = Cx_i(i) - Gx_2(Sx(k),Sy(k),X);
         
         k = k+1;
    end
end
Err = [Err1;Err2];
end