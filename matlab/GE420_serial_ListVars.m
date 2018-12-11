function GE420_serial_ListVars()
%GE420_serial_ListVars
%   GE420_serial_ListVars() simple lists the possible variables Matlab has
%	access to inside your DSP application.  Helpful in finding the name of
%	the variable you would like to write to or read from.
%
%		GE420_serial_ListVars();
%
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
