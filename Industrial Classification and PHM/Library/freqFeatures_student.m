function xfeature=freqFeatures_student(F,f)
% Returns frequency-domain features of FFT(x)
% input:    F, 1-d vector
% output:   xFeature, table form


% Create table variable xfeature
xfeature = table;
N=length(F);


% Frequency Center
% YOUR CODE GOES HERE
xfeature.fc=sum(f.*F)/sum(F);

% RMS frequency
% YOUR CODE GOES HERE
xfeature.rmsf=sqrt(sum((f.^2).*F)/sum(F));

% Root variance frequency
% YOUR CODE GOES HERE
xfeature.rvf=sqrt(xfeature.rmsf^2-xfeature.fc^2);

end