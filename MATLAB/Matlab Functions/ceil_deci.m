function y = ceil_deci(val, decimal_multiple)
    y = decimal_multiple * ceil(val / decimal_multiple);
end