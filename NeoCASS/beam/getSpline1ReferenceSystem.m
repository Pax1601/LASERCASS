function [X0, R] = getSpline1ReferenceSystem(ID1, ID2, setNodes, Coord)
%
% [X0, R] = getSpline1ReferenceSystem(ID1, ID2, setNodes, Coord)
% [X0, R] = getSpline1ReferenceSystem(Node1, Node2);
%
%
% Computes the origin and the rotation matrix (w.r.t the basic reference)
% of the reference system associated to a beam spline. The y-axis of the 
% referencesystem will be directed from the node indicated by ID1 to that
% indicated by ID2
%
% setNodes : list of nodes that define the interpolation set
% 
% ID1      : node used as the origin of the frame
% ID2      : node that defines the y-direction
%
% Coord    : coordinates of the model nodes
%
%
% Alternative calling syntax: the coordinates of the two nodes are
% explicitly provided as input.
%
% Node1 : coordinates of first node.
% Node2 : coordinates of second node.
%
%-------------------------------------------------------------------------------
% 01-08-2016 Federico Fonte
%

if nargin == 4
	Node1 = Coord(setNodes(ID1),:);
	Node2 = Coord(setNodes(ID2),:);
elseif nargin == 2
	Node1 = reshape(ID1, [1,3]);
	Node2 = reshape(ID2, [1,3]);
else
	error('Wrong number of inputs');
end


y = Node2' - Node1';
y = y/norm(y);


% Check angle between y and basic x axis
if abs(y(2)) > 0.2 % Use basic x-axis as x-axis
	x = [1;0;0];
else
	x = [0;-1;0];
end

x = x - (x'*y)*y;
x = x/norm(x);

z = cross(x, y);
R = [x, y, z];
X0 = Node1;






return
