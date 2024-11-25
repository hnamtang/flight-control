% Longitudinal model of VFW 614

Alon = [
    -0.9981, -2.5072, -3.518e-4,     0.0;
     0.9709, -0.9632,   -0.0025,  0.0099;
    -0.1274,  4.6526,   -0.0219, -9.7234;
        1.0,     0.0,       0.0,     0.0
];

Blon = [
     0.1335, -5.6897;
    -0.0048, -0.1038;
     2.9885, -0.6188;
        0.0,  0.0518
];

Clon = eye(size(Alon));

Dlon = zeros(size(Alon, 1), size(Blon, 2));

sysLon = ss(Alon, Blon, Clon, Dlon);
sysLon.InputName = {'\delta_t', '\delta_e'};
sysLon.StateName = {'q', '\alpha', 'V', '\Theta'};
sysLon.OutputName = sysLon.StateName;
sysLon.InputUnit = {'-', 'rad'};
sysLon.OutputUnit = {'rad/s', 'rad', 'm/s', 'rad'};