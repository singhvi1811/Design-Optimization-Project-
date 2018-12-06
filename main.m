clear all;
close all;
clc;
tic
%% Constants.
m = 30000;       % Mass of vehicles.
eff_e = 0.5;    % Engine efficiency.
eff_d = 0.8;    % Driveline efficiency.
Cr = 0.01;      % Coefficient of Rolling resistance.
T = 1;          % Discrete time intervals.
ix=5;           %Final drive ratio
i0=10;          %Gear ratio
rw=0.5;          %radius of wheel
y1=i0*ix/rw;

%% Drive Cycle
dr_cyc = xlsread('DriveCycles_Scaled',6,'A4:B600');
v1 = 0.44704*dr_cyc(:,2); %mph to m/s
t = dr_cyc(:,1); %time vector
s = length(t); %total time
TI= s;         % Prediction time zone.

%% 
g = 9.81*ones(s,1);       % Gravitational constant.
v2 = zeros(s,1);          % Optimize velocity vector  
a2 = zeros(s,1);          % acceleration of vehicle 2  
p1 = zeros(s,1);          % Position vector of vehicle 1  
p2 = zeros(s,1);          % Position vector of vehicle 2  
vfinal = zeros(s,1);      % Final optimized velocity vector 
p1(1) = 50;               % Initial position of vehicle 1  
p2(1) = 0;                % Initial position of vehicle 2  
E1total=0;                % Total Energy for vehicle 1 
E2total=0;                % Total energy for vehicle 2  
%% Optimization section.
A = [];         
b = [];
Aeq = [];
beq = [];
ub=[];
opts=optimoptions('fmincon','Algorithm','sqp');

%The following loops optimizes the energy of vehicle 2 using velocity as
%the design variable
j=1;
 while j<s
    if s-j<TI
         TI=s-j;
    end
    x0 = v2(j:j+TI);
    lb=0.00001*ones(TI+1,1);
    [v2_new,E,exitflag]=fmincon(@(x) obj(x,T,g(j:j+TI),Cr,y1,m,eff_d),x0,A,b,Aeq,beq,lb,ub,@(x) constr(x,v1(j:j+TI),T,p1(j),p2(j),a2(j)),opts);
     vfinal(j:j+TI,1)=v2_new; %Velocity vector
     E2total=E+E2total;       %Energy Update  
     for i=j+1:j+TI
         p2(i) =  p2(i-1)+vfinal(i)*T; % Position Update for vehicle 2
         p1(i) =  p1(i-1)+v1(i)*T; % POstion Update for Vehicle 1
         a2(i) =  (vfinal(i) - vfinal(i-1))/T; % Acceleration Update for Vehicle 2
     end
     j=j+TI;
 end
    for i=2:s
        a1(i) = (v1(i) - v1(i-1))/T;       %Accleration of truck 1
    end   

%% Energy function for vehicle 1.
 E1total=.001*(m/1)*(((v1)'*v1)/2+Cr*(g)'*v1);
%% Energy saved (%)
Esaved=100*((E1total-E2total)/E1total);
%% Plots.
figure(1)
plot(t,v1,'Linewidth',2);
hold on;
plot(t,vfinal,'Linewidth',2);
title('Velocity');
legend("Vehicle 1","Vehicle 2")
xlabel('Time(s)')
ylabel('Velocity(m/s)')

figure(2)
plot(t,p1);
hold on;
plot(t,p2);
title('position');
legend("Vehicle 1 ","Vehicle 2")
xlabel('Time(s)')
ylabel('Absolute position(m)')

figure(3)
plot(t,a1);
hold on;
plot(t,a2);
title('Acceleration');
legend("Vehicle 1"," Vehicle 2")
xlabel('Time(s)')
ylabel('Acceleration(m/s2')

figure(4)
plot(t,p1-p2,'Linewidth',2);
title('Relative Position(Limits: 20m to 70m) ');
xlabel('Time(s)')
ylabel('Relative position(m)')

figure(5)
Efinal=[E1total,E2total];
bar(Efinal);
legend("Energy Vehicle 1 and Vehicle 2");
title('Energy Consumption (Kw)');

toc

