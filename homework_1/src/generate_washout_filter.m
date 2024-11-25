function washoutFilter = generate_washout_filter(timeConstant)
%generate_washout_filter Generate washout filter for yaw damper

    washoutFilter = tf([timeConstant, 0], [timeConstant, 1]);

end