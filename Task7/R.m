function [R_mat] = R(size)
R = eye(2*size);
for i = 1:2*size
   if(i<=size)
       R(i,i) = 1;
   else
       R(i,i) = 8; 
   end
end
R_mat = R;
end