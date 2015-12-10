%visc is a 1x2 vector with v(1) and v(2) being the dispersed viscosity and
%continuous viscosity
function [epsilonDot,z1,z2,speed] = intTsn(vDelta,vMinax,vMajax,X,visc)
global fps
dDelta = diff(vDelta);
viscRat = visc(1)/visc(2); %viscosity ratio
alpha = (2*viscRat + 3)*(19*viscRat + 16)/40/(viscRat + 1);
%xfilt = sgolayfilt(X,5,51);
speed = diff(X)*fps;
dSpeed = diff(speed);
%dSpeed = sgolayfilt(dSpeed,5,51);
dX = diff(X);
length(dX)
length(dSpeed)
epsilonDot = dSpeed./dX(1:length(dX) - 1);
a0 = (vMajax.*vMinax.^2).^(1/3);
z1 = alpha*visc(2)*(5/(2*viscRat + 3)*epsilonDot - dDelta(1:length(epsilonDot))*fps);
z2 = (vDelta(1:length(vDelta)-1)./a0(1:length(a0) - 1));