clear all;
T = readtable('motor_tracking_task6.csv');
Data_Motor = table2array(T);

T = readtable('imu_tracking_task6.csv');
Data_IMU = table2array(T);

T = readtable('camera_tracking_task6.csv');
Data_Camera = table2array(T);

T = readtable('qr_code_position_in_global_coordinate.csv');
Data_global = table2array(T);

Time_Motor = uint32(Data_Motor(:,1));
PWM_L = Data_Motor(:,2);
PWM_R = Data_Motor(:,3);

Time_IMU = uint32(Data_IMU(:,1));
Gyro_Z = Data_IMU(:,9);

Time_Camera = uint32(Data_Camera(:,1));
H_i = (Data_Camera(:,6));
Cx_i = (Data_Camera(:,3));
QR_no = (Data_Camera(:,2));


%Averaging values of single time stamp for Left and Right Motor
k=1;
l=1;
Sum_L = 0;
Sum_R = 0;
for i = 1:(length(Time_Motor)-1)
   if Time_Motor(i) == Time_Motor(i+1)
       MotorTimeStamp_SingleValue(k) = Time_Motor(i);
       Sum_L = Sum_L + PWM_L(i);
       Sum_R = Sum_R + PWM_R(i);
       l = l+1;
   elseif Time_Motor(i) ~= Time_Motor(i+1)
       Sum_L = Sum_L + PWM_L(i);
       Avg_PWM_L = Sum_L/l;
       Sum_L = 0;
       Sum_R = Sum_R + PWM_R(i);
       Avg_PWM_R = Sum_R/l;
       Sum_R = 0;
       l=1;
       Single_PWM_L(k) = Avg_PWM_L;
       Single_PWM_R(k) = Avg_PWM_R;
       k = k+1;
       MotorTimeStamp_SingleValue(k) = Time_Motor(i+1);
   else
       
   end
end
Sum_L = Sum_L + PWM_L(i+1);
Avg_PWM_L = Sum_L/l;
Single_PWM_L(k) = Avg_PWM_L;

Sum_R = Sum_R + PWM_R(i+1);
Avg_PWM_R = Sum_R/l;
Single_PWM_R(k) = Avg_PWM_R;

Single_PWM_L = [zeros(1,4) Single_PWM_L(1:end) Single_PWM_L(end)*ones(1,6)];%appending last value
Single_PWM_R = [zeros(1,4) Single_PWM_R(1:end) Single_PWM_R(end)*ones(1,6)];%appending last value


%Averaging values of single time stamp for IMU GYRO-Z Axis
k=1;
l=1;
Sum_Z = 0;
for i = 1:(length(Time_IMU)-1)
   if Time_IMU(i) == Time_IMU(i+1)
       IMUTimeStamp_SingleValue(k) = Time_IMU(i);
       Sum_Z = Sum_Z + Gyro_Z(i);
       l = l+1;
   elseif Time_IMU(i) ~= Time_IMU(i+1)
       Sum_Z = Sum_Z + Gyro_Z(i);
       Avg_Gyro_Z = Sum_Z/l;
       Sum_Z = 0;
       l=1;
       Single_Gyro_Z(k) = Avg_Gyro_Z;
       k = k+1;
       IMUTimeStamp_SingleValue(k) = Time_IMU(i+1);
   else
       
   end
end

Sum_Z = Sum_Z + Gyro_Z(i+1);
Avg_Gyro_Z = Sum_Z/l;
Single_Gyro_Z(k) = Avg_Gyro_Z;
Single_Gyro_Z = [zeros(1,10) Single_Gyro_Z(1:end) Single_Gyro_Z(end)*ones(1,6)];%appending last value

%Averaging values of single time stamp for Hi and Cxi of camera measurement
k=1;
l=1;
Sum_H_i = 0;
Sum_Cx_i = 0;
for i = 1:(length(Time_Camera)-1)
   if Time_Camera(i) == Time_Camera(i+1)
       CameraTimeStamp_SingleValue(k) = Time_Camera(i);
       Sum_H_i = Sum_H_i + H_i(i);
       Sum_Cx_i = Sum_Cx_i + Cx_i(i);
       l = l+1;
   elseif Time_Camera(i) ~= Time_Camera(i+1)
       Sum_H_i = Sum_H_i + H_i(i);
       Avg_H_i = Sum_H_i/l;
       Sum_H_i = 0;
       Sum_Cx_i = Sum_Cx_i + Cx_i(i);
       Avg_Cx_i = Sum_Cx_i/l;
       Sum_Cx_i = 0;
       l=1;
       Single_H_i(k) = Avg_H_i;
       Single_Cx_i(k) = Avg_Cx_i;
       k = k+1;
       CameraTimeStamp_SingleValue(k) = Time_Camera(i+1);
   else
       
   end
end

Sum_H_i = Sum_H_i + H_i(i+1);
Avg_H_I = Sum_H_i/l;
Single_H_i(k) = Avg_H_i;
Sum_Cx_i = Sum_Cx_i + Cx_i(i+1);
Avg_Cx_I = Sum_Cx_i/l;
Single_Cx_i(k) = Avg_Cx_i;


V = 10*(Single_PWM_L + Single_PWM_R);
V = V';
Phi_dot = Single_Gyro_Z ;

%Prediction and measurement update
Px(1,1) = 0;
Py(1,1) = 0;
Phi(1,1) = 0;
P = zeros(3,3,144);
P(1,1,1) = 10;
P(2,2,1) = 10;
P(3,3,1) = 10;
K = zeros(3,2,144);
X = zeros(3,1,144);

for i = 1:143
    %Prediction
    X(:,:,i+1) = X(:,:,i) + [(V(i)*cosd(X(3,1,i)));(V(i)*sind(X(3,1,i)));Phi_dot(i)];
    Px(i+1)=X(1,1,i+1);
    Py(i+1)=X(2,1,i+1);
    Phi(i+1)=X(3,1,i+1);
%     Px(i+1,1) = Px(i,1) + (V(i)*cosd(Phi(i)));
%     Py(i+1,1) = Py(i,1) + (V(i)*sind(Phi(i)));
%     Phi(i+1,1) = Phi(i,1) + Phi_dot(i);
    P(:,:,i+1) = Fx(V(i),X(3,1,i))*P(:,:,i)*(Fx(V(i),X(3,1,i)))';
    
%     %Measurement Update
      H_x = Hx(Px(i+1),Py(i+1),Phi(i+1),QR_code_Sx(i,CameraTimeStamp_SingleValue,Time_Camera,QR_no,Data_global),QR_code_Sy(i,CameraTimeStamp_SingleValue,Time_Camera,QR_no,Data_global));
      K(:,:,i+1) = P(:,:,i+1)*(H_x)'*inv((H_x)*P(:,:,i+1)*(H_x)');
%     
      P(:,:,i+1) = P(:,:,i+1) - K(:,:,i+1)*(H_x*P(:,:,i+1)*(H_x'))*(K(:,:,i+1))';
end
