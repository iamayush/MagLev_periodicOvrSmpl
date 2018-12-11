function GE420_serialwrite(name,floatval,comport)
%GE420_SERIALWRITE
%   GE420_serialwrite(NAME,FLOATVAL,COMPORT) will modify a variable on the DSP 
%   where NAME is the name of the variable on the DSP, and FLOATVAL is the 
%   value you want to change that variable to.
%   comport is a string with the COM port connected to the "USB Serial Port".
%	It is usually the largest number of COM port in your system.  If you do not
%	know your comport, run the command "list_serailports" to see all the
%	available comports.
%
%	For example, to change the float variable "myfloat" on the DSP to a
%	value to 5.5 you would type:
%
%		GE420_serialwrite('myfloat',5.5,'COM12');
%


s = instrfind;
if length(s) > 0
    fclose(s);
end
clear s;

filename = dir('../Lab9_oversample_NONburstProject/debug/*.map');

map = parseMap(strcat('../Lab9_oversample_NONburstProject/debug/',filename.name))

memloc = 0;
arrsize = size(map);
found = 1;

for i=1:arrsize(1)
    if strfind(char(map(i,1)),name)
        memloc = char(map(i,2));
        found = 0;
    end
end
if (found == 1)
    exception = MException('MATLAB:VarNotFound','Variable name not found.');
    throw(exception);
end

if (testifcom(comport) == 0)
    display('Choose from the following COM ports');
    list_serialports()
    exception = MException('MATLAB:ComportNotFound','COM Port not found.');
    throw(exception);
end

hex_str = '2A0B33'; % header
hex_str = strcat(hex_str,memloc);
char_str = char(sscanf(hex_str,'%2X').');
s = serial(comport);
set(s,'BaudRate',115200);
s.InputBufferSize = 5000;
fopen(s);
fwrite(s,char_str);
fwrite(s,floatval,'float32');
fclose(s)
delete(s)
clear s
