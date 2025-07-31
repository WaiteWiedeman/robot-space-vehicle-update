function out = clip(x,lowlim,uplim)
if x > uplim
    out = uplim;
elseif x < lowlim
    out = lowlim;
else
    out = x;
end
