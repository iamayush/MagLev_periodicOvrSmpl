function [out3] = GE420_serialread(name,varsize,comport)
%GE420_SERIALREAD
%   N = GE420_serialread(NAME,VARSIZE,COMPORT) will return data from the DSP declared
%	in either the ".my_arrs" or ".my_vars" data sections where NAME refers to
%	the name of the variable or array on the DSP, and varsize refers to the
%	size (number of 32-bit words) in the variable or array.
%   comport is a string with the COM port connected to the "USB Serial Port".
%	It is usually the largest number of COM port in your system.  If you do not
%	know your comport, run the command "list_serailports" to see all the
%	available comports.
%
%	For example, to read the float variable "myfloat" from the DSP and store
%	it into a Matlab single variable called "mydata" you would type:
%
%		mydata = GE420_serialread('myfloat',1,'COM12');
%
%	To read the first 1000 entries in the float array "myfloatarray" 
%	and store it into a Matlab single variable array called "myarray" 
%	you would type:	
%
%		myarray = GE420_serialread('myfloatarray',1000,'COM12');
%


if (varsize > 1000)
    display('Varsize must be 1000 or less');
    exception = MException('MATLAB:VarsizeMAX','Varsize must be 1000 or smaller');
    throw(exception);
end

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


hexsize = dec2hex(varsize,4);
hex_str = '2A0932'; % header
hex_str = strcat(hex_str,memloc,hexsize);
char_str = char(sscanf(hex_str,'%2X').');

s = serial(comport);
set(s,'BaudRate',115200);
s.InputBufferSize = 15000;
fopen(s);
fwrite(s,char_str);

count = 0;
while 1
    inchar = fread(s,1);
    if inchar == 42
        inchar = fread(s,1);
        inchar = fread(s,1);
        if inchar == 51
            out3 = fread(s,varsize,'float32');
            break;
        end
    end
    count = count + 1;
    if count > 100
        break;
    end
end

fclose(s)
delete(s)
clear s
