function [c,ceq] = constr(x,v1,T,p1_init,p2_init,a2_init)
    p1(1)=p1_init; %Initial Position of vehicle 1 
    p2(1)=p2_init; %Initial Position of vehicle 2
    acc(1)=a2_init; %Initial acceleration of vehicle 2
   for i=2:length(x)
    p2(i) =  p2(i-1)+x(i,1)*T;
    p1(i) =  p1(i-1)+v1(i)*T;
    acc(i)=(x(i)-x(i-1))/T;
   end
    D = p1 - p2; % relative position 
    %Constraints:Relative Position and acceleration of vehicle 2  
    c(:,1) = [20 - D'; D' - 70; -1.0-acc'; acc'-1.0];
    ceq =0 ;
end
    