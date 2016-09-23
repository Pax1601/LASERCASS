function acb_jmcomm(h,e)

global ACB ac defac

status=ACB.status;

if status==0 % do nothing
   return;
end

if status==1 % load variables
   cs=cell(ACB.data.keySet.toArray);
   for i=1:length(cs)
      path=char(cs(i));
      try
	 eval(['res=num2str(ac.' path ');']);
      catch
%	 disp(['jmcomm(LOAD): ''' path ''' not found in ac, using default.']);
	 try
	    eval(['res=num2str(defac.' path ');']);
	 catch
	    disp(['jmcomm(LOAD): ''' path ''' not found in defac.']);
	    res='err';
	 end
      end
      ACB.data.put(path,java.lang.String(res));
   end
   ACB.status=0;
   return
end

if status==2 % save variables
   dt=ACB.data;
   cs=cell(dt.keySet.toArray);
   for i=1:length(cs)
      path=char(cs(i));
      eval(['ac.' path '=' char(dt.get(path)) ';']);
   end
   ACB.status=0;
   dt=[];
   clear dt cs
   return
end

if status==3 % execute command
   cmd=char(ACB.data.get('cmd'));
   if isempty(cmd)
      disp('jmcomm(EXEC): cannot retrieve command');
      return
   end
   try
      eval(cmd);
   catch
      disp(['jmcomm(EXEC): error when executing ''' cmd '''']);
   end
   ACB.status=0;
   return
end
