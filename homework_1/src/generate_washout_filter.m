function washoutFilter = generate_washout_filter(timeConstant)
%generate_washout_filter Generate washout filter for yaw damper
%
%   Author: H. N. Tang

    washoutFilter = ss(-1/timeConstant, 1/timeConstant, -1, 1);
    washoutFilter.StateName = 'x_wo';
    washoutFilter.InputName = 'yaw rate';
    washoutFilter.OutputName = 'washed out yaw rate';

end