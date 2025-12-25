function P1=getFFT_student(x,L)
% Returns single-sided spectrum of vector X
% input:    x, 1-d vector
% output:   xFeature, table form


%%% Apply FFT of x
Y = fft(x,L);


%%% Compute the two-sided spectrum P2
% Then compute the single-sided spectrum P1 based on P2 and the even-valued signal length L.
% Zero frequency P1(0), and the Nyquist frequency P(end) do not occur twice. 
P2 = abs(Y/L);
P1 = P2(1:L/2+1);
% YOUR CODE GOES HERE
P1(2:end-1)=2*P1(2:end-1);

end
