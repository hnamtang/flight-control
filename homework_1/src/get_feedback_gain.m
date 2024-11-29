function K = get_feedback_gain(system, desiredDamping, modeName, kStart, kEnd)
%get_feedback_gain Compute and return feedback gain to achieve desired damping.
%   K = get_feedback_gain(system, desiredDamping, modeName, kStart, kEnd)
%   Compute and return proportional feedback to achieve desired damping.
%   If no value of K is found, function returns None with warning.
%
%   Author: H. N. Tang

kRange = kStart:0.001:kEnd;
poles = rlocus(system, kRange);

% Find the correct pole for the mode
modeNameLower = lower(modeName);
if modeNameLower == "ph" || modeNameLower == "phugoid"
    poleMode = poles(find(real(poles) == max(real(poles)), 1));
elseif modeNameLower == "sp" || modeNameLower == "short period"
    poleMode = poles(find(real(poles) == min(real(poles)), 1));
elseif modeNameLower == "dr" || modeNameLower == "dutch roll"
    poleMode = poles(imag(poles) > 0);
else
    error('invalid mode name.');
end

% Compute damping ratio
damping = -cos(angle(poleMode));

% Find feedback gain for the desired damping ratio
[~, idx] = min(abs(damping - desiredDamping));

if abs(damping(idx) - desiredDamping) <= 9e-4
    K = kRange(idx);
else
    error(['Cannot find feedback gain K. Try changing the feedback sign, ' ...
        'increasing range of K, or changing the desired damping ratio.']);

end