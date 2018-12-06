function [feq] = obj(x,T,g,Cr,y1,m,eff_d)
     feq=.001*(m/1)*(((x)'*x)/2+Cr*(g)'*x);
end