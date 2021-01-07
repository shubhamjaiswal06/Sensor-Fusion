clear all;
T = readtable('motor_tracking_task6.csv');
Data_Motor = table2array(T);

T = readtable('imu_tracking_task6.csv');
Data_IMU = table2array(T);

Time_Motor = uint32(Data_Motor(:,1));
PWM_L = Data_Motor(:,2);
PWM_R = Data_Motor(:,3);

Time_IMU = uint32(Data_IMU(:,1));
Gyro_Z = Data_IMU(:,9);

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
Single_Gyro_Z = [zeros(1,6) Single_Gyro_Z(1:end)];

V = 10*(Single_PWM_L + Single_PWM_R);
V = V';
Phi_dot = Single_Gyro_Z - 0.1873;


Px = zeros(length(MotorTimeStamp_SingleValue),1);
Py = zeros(length(MotorTimeStamp_SingleValue),1);
Phi = zeros(length(MotorTimeStamp_SingleValue),1);

Px(1,1) = 16.5;
Py(1,1) = 49.8;
Phi(1,1) = 90;

for i = 1:(length(MotorTimeStamp_SingleValue)-1)
    delta_t = MotorTimeStamp_SingleValue(i+1) - MotorTimeStamp_SingleValue(i);
    Px(i+1,1) = Px(i,1) + (V(i)*cosd(Phi(i)));
    Py(i+1,1) = Py(i,1) + (V(i)*sind(Phi(i)));
    Phi(i+1,1) = Phi(i,1) + Phi_dot(i);
end
