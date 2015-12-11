%visc is a 1x2 vector with v(1) and v(2) being the dispersed viscosity and
%continuous viscosity
function [epsilonDot,z1,z2,speed] = intTsn(vDelta,vMinax,vMajax,X,visc)
global fps
global p4 p5 
dDelta = diff(vDelta);
viscRat = visc(1)/visc(2); %viscosity ratio
alpha = (2*viscRat + 3)*(19*viscRat + 16)/40/(viscRat + 1);
%xfilt = sgolayfilt(X,5,51);
if (isempty(p4) == 0)
    speed = csaps(X(1:length(diff(X))), diff(X)*fps, p4, X(1:length(diff(X))));
    else speed = diff(X)*fps;
end
dSpeed = diff(speed)*1e-6;
%dSpeed = sgolayfilt(dSpeed,5,51);
dX = diff(X)*1e-6;
if (isempty(p5) == 0)
    epsilonDot = dSpeed./dX(1:length(dX) - 1);
    epsilonDo = csaps(X(1:length(dSpeed)), epsilonDot, p5, X(1:length(dSpeed)));
    else epsilonDo = dSpeed./dX(1:length(dX) - 1);
        epsilonDot = dSpeed./dX(1:length(dX) - 1);
end
a0 = (vMajax.*vMinax.^2).^(1/3)*1e-3;
z1 = alpha*visc(2)*(5/(2*viscRat + 3)*epsilonDo - dDelta(1:length(epsilonDo))*fps);
z2 = (vDelta(1:length(vDelta)-1)./a0(1:length(a0) - 1));