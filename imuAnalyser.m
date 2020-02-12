function [transVecImu, angVecImu] = imuAnalyser(imuTime, imuACC, imuGYR, t0, t1, nearest_index0, nearest_index1)
%analyse de l'IMU entre les deux scans
transVecImu = zeros(3,1);
dcm = eye(3,3);
matRotImu_icp = [0 1 0; 1 0 0; 0 0 -1];
roll = 0;
pitch = 0;
yaw = 0;
t_imu0 = t0;
for j=nearest_index0:nearest_index1
    t_imu1 = imuTime(j);
    
    %intégration des données de l'accéléromètre
    transVecImu = transVecImu + (imuACC(j, :)*(t_imu1-t_imu0)^2)';
    transVecImu = matRotImu_icp*transVecImu;
    
    %intégration des données du gyromètre
    roll = roll + imuGYR(nearest_index0, 1)*(t_imu1-t_imu0);
    pitch = pitch + imuGYR(nearest_index0, 2)*(t_imu1-t_imu0);
    yaw = yaw + imuGYR(nearest_index0, 3)*(t_imu1-t_imu0);
    
    t_imu0= imuTime(j);
end
%intégration entre la dernière valeur de l'inertie et le scan
t_imu1 = t1;

%intégration des données de l'accéléromètre
transVecImu = transVecImu + (imuACC(j, :)*(t_imu1-t_imu0)^2)';
transVecImu = matRotImu_icp*transVecImu;

%intégration des données du gyromètre
roll = roll + imuGYR(nearest_index0, 1)*(t_imu1-t_imu0);
pitch = pitch + imuGYR(nearest_index0, 2)*(t_imu1-t_imu0);
yaw = yaw + imuGYR(nearest_index0, 3)*(t_imu1-t_imu0);
angVecImu = matRotImu_icp*[roll; pitch; yaw];
end