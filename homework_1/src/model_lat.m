% Lateral model of VFW 614

Alat = [
    -0.4956,  1.9243, -0.1206,    0.0;
    -0.9795,  -0.193,  0.0963, 0.1055;
      2.107, -5.5049, -3.3388,    0.0;
        0.0,     0.0,     1.0,    0.0
];

Blat = [
    -0.4515, -1.318;
        0.0, 0.0362;
     -9.498, 1.9929;
        0.0,    0.0
];

Clat = eye(size(Alat));

Dlat = zeros(size(Alat, 1), size(Blat, 2));

sys_lat = ss(Alat, Blat, Clat, Dlat);
sys_lat.InputName = {'\delta_a', '\delta_r'};
sys_lat.StateName = {'r', '\beta', 'p', '\Phi'};
sys_lat.OutputName = sys_lat.StateName;
sys_lat.InputUnit = {'rad', 'rad'};
sys_lat.OutputUnit = {'rad/s', 'rad', 'm/s', 'rad'};