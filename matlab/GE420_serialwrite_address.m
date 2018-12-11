function GE420_serialwrite_address(address,floatval)
%GE420_SERIALWRITE_ADDRESS
%   GE420_serialwrite_address(ADDRESS,FLOATVAL) will modify a variable at 
%   the memory location ADDRESS in decimal, not hex, on the DSP. FLOATVAL 
%   is the value you want to change that variable to.
%
%	For example, to change the float variable at address "0001f5b0" on the 
%   DSP to a value to 5.5 you would type:
%
%		GE420_serialwrite_address(128432,5.5);
%


s = instrfind;
if length(s) > 0
    fclose(s);
end
clear s;

filename = dir('../Lab9_oversample_NONburst/debug/*.map');

map = parseMap(strcat('../Lab9_oversample_NONburst/debug/',filename.name))

memloc = dec2hex(address,8);

hex_str = '2A0B33'; % header
hex_str = strcat(hex_str,memloc);
char_str = char(sscanf(hex_str,'%2X').');
s = serial('COM1');
set(s,'BaudRate',115200);
s.InputBufferSize = 5000;
fopen(s);
fwrite(s,char_str);
fwrite(s,floatval,'float32');
fclose(s)
delete(s)
clear s


