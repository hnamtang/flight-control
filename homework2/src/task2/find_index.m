function [index_start, index_end] = find_index(t, t_start, heading)
% Find start and end index for flight data
% Calculated based on heading
% Inputs:
%   - t: flight time
%   - t_start: start time of maneuver
%   - heading: heading data
%
% Author: H. N. Tang

    index_start = find(t >= t_start, 1);
    heading = heading(index_start:end);
    [~, index_end] = min(abs(heading(200:end) - heading(1)));  % ignore the first 200 data points
    index_end = index_end + index_start + 200;

end