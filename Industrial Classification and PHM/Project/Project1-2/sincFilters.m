function filters = sincFilters(numFilters, filterLen, fs)
% ============================================================
%   Generate SincNet filterbank (band-pass filters)
%   numFilters : number of filters (e.g., 40)
%   filterLen  : filter length (e.g., 251)
%   fs         : sampling rate
%
%   Output: filters [filterLen × numFilters]
% ============================================================

% Mel-scale frequency mapping
lowFreq = 30;                      % avoid zero-frequency
highFreq = fs/2 - 100;             % Nyquist margin

melLow = hz2mel(lowFreq);
melHigh = hz2mel(highFreq);

melPoints = linspace(melLow, melHigh, numFilters+1);
hzPoints  = mel2hz(melPoints);

filters = zeros(filterLen, numFilters);

% time axis
t = -(filterLen-1)/2 : (filterLen-1)/2;
t = t(:);

for i = 1:numFilters
    f1 = hzPoints(i);
    f2 = hzPoints(i+1);

    % ideal band-pass = sinc(f2) - sinc(f1)
    h = 2*f2/fs * sinc(2*f2/fs * t) - 2*f1/fs * sinc(2*f1/fs * t);

    % apply Hamming window
    h = h .* hamming(filterLen);

    % normalize
    h = h / sum(h);

    filters(:, i) = h;
end

end

function mel = hz2mel(hz)
    mel = 2595 * log10(1 + hz/700);
end

function hz = mel2hz(mel)
    hz = 700 * (10.^(mel/2595) - 1);
end
