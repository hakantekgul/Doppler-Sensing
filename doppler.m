% MP2: Doppler Sensing for Velocity Calculation of Hand Motion %

% read input text and access points data % 
fileID = fopen('input2.txt','r');
formatSpec = '%f';
Input = fscanf(fileID,formatSpec); %Input about relevant info read
fclose(fileID);

% parse the data from input file %
client_location_x = Input(1);
client_location_y = Input(2);
client_location_z = Input(3);
Num_AP = Input(4);

AP1_coor_x = Input(5);
AP1_coor_y = Input(6);
AP1_coor_z = Input(7);

AP2_coor_x = Input(8);
AP2_coor_y = Input(9);
AP2_coor_z = Input(10);

AP3_coor_x = Input(11);
AP3_coor_y = Input(12);
AP3_coor_z = Input(13);

AP4_coor_x = Input(14);
AP4_coor_y = Input(15);
AP4_coor_z = Input(16);

% read Access Point baseband signals %

AP1 = dlmread('AP1.txt');
AP2 = dlmread('AP2.txt');
AP3 = dlmread('AP3.txt');
AP4 = dlmread('AP4.txt');

% We have 5 seconds long downsampled and downconverted samples %
sampling_freq = 1000000;

% Compute the Doppler shift %

AP1_fft = (fft(AP1));
AP2_fft = (fft(AP2));
AP3_fft = (fft(AP3));
AP4_fft = (fft(AP4));

freq = (linspace(-9000000,5000000,5000000))';

[M1,I1] = max(abs(AP1_fft));
if I1 > 4000000
    dshift1 = 5e6 - I1;
else
    dshift1 = I1;
end

[M2,I2] = max(abs(AP2_fft));
if I2 > 4000000
    dshift2 = 5e6 - I2;
else
    dshift2 = I2;
end

[M3,I3] = max(abs(AP3_fft));
if I3 > 4000000
    dshift3 = 5e6 - I3;
else
    dshift3 = I3;
end

[M4,I4] = max(abs(AP4_fft));
if I4 > 4000000
    dshift4 = 5e6 - I4;
else
    dshift4 = I4;
end

% Apply the doppler shift formula to get relative velocities %

projected_v1 = (dshift1 * (3e8)) / (2*(5.8e9));
projected_v2 = (dshift2 * (3e8)) / (2*(5.8e9));
projected_v3 = (dshift3 * (3e8)) / (2*(5.8e9));
projected_v4 = (dshift4 * (3e8)) / (2*(5.8e9));

% Use these velocities and 3D coordinates to compute the user velocity %

% find the direction vectors from each AP %
dir_v1 = [(client_location_x - AP1_coor_x),(client_location_y - AP1_coor_y), (client_location_z - AP1_coor_z)];
dir_v2 = [(client_location_x - AP2_coor_x),(client_location_y - AP2_coor_y), (client_location_z - AP2_coor_z)];
dir_v3 = [(client_location_x - AP3_coor_x),(client_location_y - AP3_coor_y), (client_location_z - AP3_coor_z)];
dir_v4 = [(client_location_x - AP4_coor_x),(client_location_y - AP4_coor_y), (client_location_z - AP4_coor_z)];


Matrix = [dir_v1
          dir_v2
          dir_v3
          dir_v4];
V = [projected_v1
    projected_v2
    projected_v3
    projected_v4];

user_velocity = linsolve(Matrix,V);

disp((user_velocity)');


