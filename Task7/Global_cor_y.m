function [Center_cor_y] = Global_cor_y(QR_No,Data_global)
for i = 1: length(Data_global)
    if Data_global(i,1) == QR_No
        Center_cor_y = Data_global(i,3);
    else
        
    end
end
end