function PIcomp = PI_compensator(zero_PI)

PIcomp = tf([(-1/zero_PI) 1], [1, 0]);
PIcomp.InputName = "uPI";
PIcomp.OutputName = "\delta \xi input";

end