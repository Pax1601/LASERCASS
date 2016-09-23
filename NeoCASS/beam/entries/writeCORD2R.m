function [] = writeCORD2R(fid, CID, RID, X0, R)
%
% [] = writeCORD2R(fid, CID, RID, X0, R)
% [] = writeCORD2R(fid, CID, RID, Mat)
%
% X0 origin of the reference system
% R [3 x 3] rotation matrix
%
% Mat:  is a 3x3 matrix whose rows are coordinates
%       of points A, B, C
%
%-------------------------------------------------------------------------------
% 01-08-2016 Federico Fonte
%

if nargin == 4
	Mat = X0;
else
	Mat = RotMat_2_CORD2R(R, X0);
end

void = '        ';

fprintf(fid, '%s%s%s%s%s%s%s%s%s\n', str2char8('CORD2R', 'l'), ...
        int2char8(CID), int2char8(RID), ...
        dbl2char8(Mat(1,1)), dbl2char8(Mat(1,2)), dbl2char8(Mat(1,3)), ...
        dbl2char8(Mat(2,1)), dbl2char8(Mat(2,2)), dbl2char8(Mat(2,3)));


fprintf(fid, '%s%s%s%s\n', void, ...
        dbl2char8(Mat(3,1)), dbl2char8(Mat(3,2)), dbl2char8(Mat(3,3)));


return



function [Mat] = RotMat_2_CORD2R(R, X0)
%
% [Mat] = RotMat_2_CORD2R(R, X0)
%
% Mat:  is a 3x3 matrix whose rows are coordinates
%       of points A, B, C
%
%
%-------------------------------------------------------------------------------
% 05-03-2015
%

Mat = zeros(3,3);

X0 = reshape(X0, [3,1]);

% First Point: origin
Mat(1,:) = X0;

% Second point: on z-axis
Mat(2,:) = X0 + R(:,3);

% Third point: on xz plane
Mat(3,:) = X0 + R(:,1);

return
