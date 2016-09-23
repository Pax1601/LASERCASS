function comp1=qxheavy(input,limit,type)
%HEAVY A heavyside step function to switch "on" or "off"
if type<1
	comp1=0.5+0.5*tanh(110*(input-limit));
else
   comp1=round(0.5+0.5*tanh(110*(input-limit)));
end