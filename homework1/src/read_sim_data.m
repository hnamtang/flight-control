function simData = read_sim_data(fileID, motionType)
%read_sim_data Read SEPHIR simulation data.
%   simData = read_sim_data(fileID)
%   read and extract relevant data from dataset.
%
%   Output is a MATLAB structure array
%   with field names being relevant flight variables
%
%   Note: currently longitudinal motion variables not tested.
%
%   Author: H. N. Tang

    % Import data
    filePath = "./sim_data/sw_00370004_TUB614_" + fileID + ".mat";
    data = load(filePath);

    % Extract data
    samplingFrequency = 50;  % 50 Hz
    time = data.data.wd_time(1):(1/samplingFrequency):data.data.wd_time(end);

    simData = struct();
    motionType = lower(motionType);
    if contains(motionType, "lon", 'IgnoreCase', true)
        simData.duration = linspace(0, time(end) - time(1), length(time));
        simData.altitude = interpolate(data.data.wd_hsl);  % [m]
        simData.flightPathAngle = interpolate(data.data.wd_gamma);
        simData.AOA = interpolate(data.data.wd_al);
        simData.AOARate = interpolate(data.data.wd_alphdot);
        simData.thrust = interpolate(data.data.wd_tnet1);
        simData.trueAirspeed = interpolate(data.data.wd_vtas);  % [m/s]
        simData.elevatorDefl = interpolate(data.data.wd_etalh);
        simData.pitchAngleEuler = interpolate(data.data.wd_theta);
        simData.pitchRateEuler = interpolate(data.data.wd_thetdot);
        simData.pitchRateBody = interpolate(data.data.wd_qb);
    elseif contains(motionType, "lat", 'IgnoreCase', true) || ...
            contains(motionType, "dir", 'IgnoreCase', true)    
        simData.duration = linspace(0, time(end) - time(1), length(time));
        simData.altitude = interpolate(data.data.wd_hsl);  % [m]
        simData.sideslip = interpolate(data.data.wd_beta);
        simData.trueAirspeed = interpolate(data.data.wd_vtas);
        simData.aileronDefl = interpolate(data.data.wd_xior);
        simData.rudderDefl = interpolate(data.data.wd_ze);
        simData.rollAngleEuler = interpolate(data.data.wd_phi);
        %simData.rollRateEuler = interpolate(data.data.wd_phidot);
        simData.rollRateBody = interpolate(data.data.wd_pb);
        simData.yawAngleEuler = interpolate(unwrap(data.data.wd_psi));  % Azimuth
        %simData.yawRateEuler = interpolate(data.data.wd_psidot);
        simData.yawRateBody = interpolate(data.data.wd_rb);
    else
        error('invalid motion type.');
    end


    function varAdjusted = interpolate(variable)
        timeIrregular = data.data.wd_time;
        varAdjusted = interp1(timeIrregular, variable, time, 'linear');
    end
    
end