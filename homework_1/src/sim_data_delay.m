function dt = sim_data_delay(dataOL, dataCL, motionType)
%   Calculate time delay between signals.
%   Output is structure array with field names being relevant variables
%
%   Author: H. N. Tang

    if contains(motionType, "lon", "IgnoreCase", true)
        dt.altitude = finddelay(dataCL.altitude, dataOL.altitude);
        dt.flightPathAngle = finddelay(dataCL.flightPathAngle, dataOL.flightPathAngle);
        dt.AOA = finddelay(dataCL.AOA, dataOL.AOA);
        dt.AOARate = finddelay(dataCL.AOARate, dataOL.AOARate);
        dt.thrust = finddelay(dataCL.thrust, dataOL.thrust);
        dt.trueAirspeed = finddelay(dataCL.trueAirspeed, dataOL.trueAirspeed);
        dt.elevatorDefl = finddelay(dataCL.elevatorDefl, dataOL.elevatorDefl);
        dt.pitchAngleEuler = finddelay(dataCL.pitchAngleEuler, dataOL.pitchAngleEuler);
        dt.pitchRateEuler = finddelay(dataCL.pitchRateEuler, dataOL.pitchRateEuler);
        dt.pitchRateBody = finddelay(dataCL.pitchRateBody, dataOL.pitchRateBody);
    elseif contains(motionType, "lat", "IgnoreCase", true) || ...
            contains(motionType, "dir", "IgnoreCase", true)
        dt.sideslip = finddelay(dataCL.sideslip, dataOL.sideslip);
        dt.altitude = finddelay(dataCL.altitude, dataOL.altitude);
        dt.trueAirspeed = finddelay(dataCL.trueAirspeed, dataOL.trueAirspeed);
        dt.aileronDefl = finddelay(dataCL.aileronDefl, dataOL.aileronDefl);
        dt.rudderDefl = finddelay(dataCL.rudderDefl, dataOL.rudderDefl);
        dt.rollAngleEuler = finddelay(dataCL.rollAngleEuler, dataOL.rollAngleEuler);
        dt.rollRateBody = finddelay(dataCL.rollRateBody, dataOL.rollRateBody);
        dt.yawAngleEuler = finddelay(dataCL.yawAngleEuler, dataOL.yawAngleEuler);
        dt.yawRateBody = finddelay(dataCL.yawRateBody, dataOL.yawRateBody);
    end

end