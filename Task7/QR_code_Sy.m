function [Center_cor_y] = QR_code_Sy(j,CameraTimeStamp_SingleValue,Time_Camera,QR_No,Data_global)
t = CameraTimeStamp_SingleValue(j);
k = 1;
for i = 1:length(Time_Camera)
    if t == Time_Camera(i)
        Center_cor_y(k) = Global_cor_y(QR_No(i),Data_global);
        k = k+1;
    end
end