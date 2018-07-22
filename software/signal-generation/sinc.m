function out = sinc(in)
if in == 0
    out = 1;
else
    out = sin(in)/in;
end