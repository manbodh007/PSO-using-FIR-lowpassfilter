function z = findError(currentFilterPosition, desiredFilter_h)
    [h,w] = freqz(currentFilterPosition,1,250);
    h1 = abs(h);
    h2 = abs(desiredFilter_h);
   %%disp(h1);
   %%disp(h2);
    d1 =(h1-h2).^2;
     
    result = sum(d1);
    z = result.^0.5;
    
end