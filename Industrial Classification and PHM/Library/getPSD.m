function psdx = getPSD(x, L, Fs)
% ==========================================================
% getPSD.m
% Industrial AI & Automation by Y.K.Kim
% ----------------------------------------------------------
% Purpose: Compute single-sided Power Spectral Density (PSD)
% Input :  x  -> time-domain signal
%          L  -> length of signal
%          Fs -> sampling frequency
% Output:  psdx -> one-sided PSD [Power/Frequency]
% ==========================================================

% 1. FFT
Y = fft(x, L);

% 2. Two-sided spectrum (no normalization yet)
P2 = abs(Y);

% 3. Power spectral density
psdx = (1/(Fs*L)) * P2(1:L/2+1).^2;

% 4. Double energy for positive frequencies (except DC & Nyquist)
psdx(2:end-1) = 2 * psdx(2:end-1);

end
