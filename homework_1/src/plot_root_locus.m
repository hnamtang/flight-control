function plot_root_locus(system)
%plot_root_locus Plot root locus for each output feedback of MIMO system.
%   plot_root_locus(system_name) plots every output feedback roo locus of a
%   MIMO system. It helps control designer find the suitable output for
%   feedback controller.
%
%   If no suitable feedback is found, try changing the feedback sign.
%
%   Note: The property fields
%       - InputName
%       - OutputName
%       - StateName
%       - InputUnit
%       - OutputUnit
%   must be specified.
%
%   Author: H. N. Tang

    nOutputs = size(system, 1);
    nInputs = size(system, 2);

    figsArray = get(groot, 'Children');
    if class(figsArray) == "matlab.graphics.GraphicsPlaceholder"
        currentMaxFigIdNumber = 0;
    else
        currentMaxFigIdNumber = max([figsArray.Number]);
    end
    for iOutput = 1:nOutputs
        figure(currentMaxFigIdNumber + iOutput);
        for iInput = 1:nInputs
            subplot(nInputs, 1, iInput);
            rlocus(system(iOutput, iInput));
            legend(system.OutputName{iOutput} + " to " + system.InputName{iInput}, ...
                'Location', 'southeast', 'FontSize', 14);
        end
    end

end