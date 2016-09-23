function [Xfaif,Yfaif,Zfaif]=faircmp(bodlgtf,bodlgta,bodwidt,bodheig, ...
                                     bodaplc,offset,seg);
% This routine computes the geometric surface definition of any fairing the
% user wishes to generate. The fairing geometry is based on a hyperbolic
% paraboloid construct
%=====
% initialise all necessary parameters
ydistr(1:seg+1)=(0:seg)/seg;ydistr=ydistr';
Xfaif(:,seg+1)=zeros(seg+1,1);Yfaif(:,seg+1)=zeros(seg+1,1);
Zfaif(:,seg+1)=zeros(seg+1,1);
Xfaira(:,seg+1)=zeros(seg+1,1);Yfaira(:,seg+1)=zeros(seg+1,1);
Zfaira(:,seg+1)=zeros(seg+1,1);
%=====
% generate the forward and aft fairing coordinates
for i=0:seg-1
% forward data
   Xfaif(1:seg+1,i+1)=(seg-i)*bodlgtf*bodaplc/seg;
   semyax=(Xfaif(1,i+1)/(bodlgtf*bodaplc)*bodwidt^2)^0.5;
   Yfaif(1:seg+1,i+1)=semyax*ydistr(1:seg+1,1);
   semzax=(Xfaif(1,i+1)/(bodlgtf*bodaplc)*bodheig^2)^0.5;
   Zfaif(1:seg+1,i+1)=(semzax/semyax*((semyax^2)- ...
                        Yfaif(1:seg+1,i+1).*Yfaif(1:seg+1,i+1)).^0.5);
% aft data
   Xfaira(1:seg+1,i+1)=(seg-i)*bodlgta*bodaplc/seg;
   semyax=(Xfaira(1,i+1)/(bodlgta*bodaplc)*bodwidt^2)^0.5;
   Yfaira(1:seg+1,i+1)=semyax*ydistr(1:seg+1,1);
   semzax=(Xfaira(1,i+1)/(bodlgta*bodaplc)*bodheig^2)^0.5;
   Zfaira(1:seg+1,i+1)=(semzax/semyax*((semyax^2)- ...
                        Yfaira(1:seg+1,i+1).*Yfaira(1:seg+1,i+1)).^0.5);
end
%=====
% construct the surface matrix of the 3-D body
% add fwd and aft segments
Xfaif=[fliplr(Xfaif) fliplr(Xfaira)+Xfaif(1,1)+offset*bodaplc];
Yfaif=[fliplr(Yfaif) Yfaira];Zfaif=[fliplr(Zfaif) Zfaira];
% mirror in X-Y plane (top and bottom)
Xfaif=[Xfaif Xfaif];Yfaif=[Yfaif -Yfaif];Zfaif=[Zfaif Zfaif];
% mirror in X-Z plane (left and right)
Xfaif=[Xfaif Xfaif];Yfaif=[Yfaif Yfaif];Zfaif=[Zfaif -Zfaif];
return
%==========================================================================
%==