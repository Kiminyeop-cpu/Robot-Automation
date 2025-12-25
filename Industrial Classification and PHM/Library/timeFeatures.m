function xfeature = timeFeatures(x)
% ==========================================================
% timeFeatures.m
% Industrial AI & Automation by Y.K.Kim
% ----------------------------------------------------------
% Purpose: Calculate time-domain features of signal x
% Input : x  -> input signal (vector)
% Output: xfeature -> table of features
% ==========================================================

% 기본 세팅
N = length(x);
xfeature = table;

% === 통계적 특징 ===
xfeature.mean   = mean(x);
xfeature.std    = std(x);

% === 에너지 관련 특징 ===
xfeature.rms    = sqrt(mean(x.^2));
xfeature.sra    = (mean(sqrt(abs(x))))^2;
xfeature.aav    = sum(abs(x))/N;
xfeature.energy = sum(x.^2);

% === 진폭 관련 특징 ===
xfeature.peak   = max(abs(x));
xfeature.ppv    = peak2peak(x);

% === 비율 기반 특징 ===
xfeature.if     = xfeature.peak / xfeature.aav;     % Impulse Factor
xfeature.sf     = xfeature.rms  / xfeature.aav;     % Shape Factor
xfeature.crest  = xfeature.peak / xfeature.rms;     % Crest Factor
xfeature.mf     = xfeature.peak / xfeature.sra;     % Marginal (Clearance) Factor

% === 분포 기반 특징 ===
xfeature.sk     = skewness(x);
xfeature.kt     = kurtosis(x);

end
