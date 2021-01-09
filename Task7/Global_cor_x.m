function [Center_cor_x] = Global_cor_x(QR_No,Data_global)
for i = 1: length(Data_global)
    if Data_global(i,1) == QR_No
        Center_cor_x = Data_global(i,2);
    else
        
    end
end
end