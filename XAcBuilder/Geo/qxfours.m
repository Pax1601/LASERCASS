function R=qxfours(a,thedat)
%QXFOURS This is the Fourier Series Expansion generic function 
%============================================================================
R=a(1)+a(2)*sin(thedat)+a(3)*cos(2*thedat);
return