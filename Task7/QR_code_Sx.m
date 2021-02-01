function [Center_cor_x] = QR_code_Sx(j,CameraTimeStamp_SingleValue,Time_Camera,QR_No,Data_global)
t = CameraTimeStamp_SingleValue(j);
k = 1;

for i = 1:length(Time_Camera)
    if t == Time_Camera(i)
        Center_cor_x(k,1) = Global_cor_x(QR_No(i),Data_global);
        k = k+1;
%           Center_cor_x = Global_cor_x(QR_No(i),Data_global);
    end
end
end