%% From XFLR5
T0 = readtable('T1-16_0 m_s-VLM2 VTOL.txt');
T1 = readtable('vtol_data_aero.txt');
alpha=T1.Var1;
beta=T1.Var2;
CL=T1.Var3;
CD=T1.Var6;
CY=T1.Var7;
Cl=T1.Var8;
Cm=T1.Var9;
Cn=T1.Var10;


%Initial condition 
V0=16; 
alpha0= 0;
gamma0=0;
on=1;
%% Mass Property of  Micro Drone 
% ( A V tailed (Elevator+Rudder) Fixed wing configuration)
g=9.8;
Wspan= 2.5;              % In m
xyProjSpan= 2.5;         % In m 
S_wing=  0.704;          % In m2
Diahedral_w=0;
Diahedral_t=34;          % In degree
%S_t=0.04145;             %  Tail area In m2

S_t= S_wing*cosd(Diahedral_t); % horizontal tail area in m2
S_v= S_wing*sind(Diahedral_t);  % Vertical  tail area in m2
S_e=S_t/4;               %  Elevator area m2
S_r=S_v/4;               %  Rudder area  m2
tou=0.5;                 % flap effectiveness based on area ratio (Se/St)
Wing_load=  9.95;        %In  kg/m2
VH_t= 0.484;             % Tail Volume Ratio
C_rootchrod= 0.35;       %  wing chrod In m (measured)
MAC     =0.29;           %  wing mac  In m   (given in Xflr)
TipTwist=  3.5;           % In Degree          
AR      =  8.884;
Ratio_Tapr=0.171;
Sweep_roottip= 9.841;     % In Degree   
XNP= 0.197;              % In m  (xflr)
m_plane= 7 ;             % Plane mass In kg 
rho=1.225;              % In kg/m3
Viscocity=1.16114e-05;   % In  m2/s
S_ref= 0.704;          %  SurefACE AREA In m2
L_ref= 1.65;              % Refrence length In m
% ___CG- Position In meters_____ 
Xcg= 0.159;           %measured  in m from wings LE 
Ycg=-1.093e-17;
Zcg=0.011;
% ___Inertia - Body Axis - CoG Origin____
     Ibxx=     0.38691; % In kg.m²
     Ibyy=     0.24870;   % In kg.m²
     Ibzz=     0.62541;   % In kg.m²
     Ibxz=  0.00134;   % In kg.m²
     J=[ Ibxx 0  -Ibxz; 0 Ibyy  0; -Ibxz 0 Ibzz];
%% Motor parameters front two only
R_p=0.3556/2;  % propeller dia front 14 inches=0.3556 m
S_p=pi*(R_p)^2; % motor swept area
%% Loading Propulsion data
% data0=load('front_motor_propulsion.mat');
% % voltage=data0.aaa(:,1);
% % current=data0.aaa(:,2);
% % Thrust=(data0.aaa(:,3))*0.00980665;   % Now in  newton
% % RPM=data0.aaa(:,6);% 

data1=load('front_motor_extracted.mat');
voltage=data1.bb(:,1);
current=data1.bb(:,2);
Thrust=(data1.bb(:,3))*0.00980665;   % Now in  newton
RPM=(data1.bb(:,6))*0.1047;% now convertrd into rpm to radian per sec

K_T= 7.0343e-05;  % from omega2 vs T grph we get Kt or km in N/squre(rad/sec)
C_p=K_T/(0.5*rho*S_p*(R_p)^2);
K_Q=C_p/(2*pi);
% C_p= 0.12;      % motor aero coefficient
%K_motor=8.565;   % rad/sec
throttle=(1:16)'*(6.2500);
K_motor=942.3726;
q_motor=283;     % rad/se

%% static stability and dynamic derivative  calculation 

 R2D= 180/pi;    % 1 rad equvalent in degree 
 D2R=1/R2D;
 
 % 
 % CD_a=0.0652/D2R;
 CD_a=0.0075846;
 CD_0=0.028225;
 CL_a=0.0904/D2R; %% converted from per degree to per radian for calculation
 CL_0=0.669072;
 CLmax=1.38688;    %  at alpha=8 deg no elevator deflection case  
CL_a_w=CL_a;      % Lift curve slope of wing plot()
CL_a_t=0.65*CL_a;   % Lift curve slope of  horizontal tail is 0.65 times overall CL_a
CL_a_v=0.55*CL_a;   %  Lift curve slope of vertical tail 
 Cm_a=-0.0144155/D2R;

 Cm_0=0.051262; %-0.002156
 Cm_0_w=0;      % moment coeff of wing at alpha =0
 Cm_0_b=0;      % moment coeff of  body at alpha =0
 Cm_wb=Cm_0_w+Cm_0_b;                  %wing  tail moment coeff at alpha=0
 Cm_a_b=0;            % moment slope of body 
 c_w= 0.35;          % Wing chord
 cmac_w= 0.29;        %% from xfrlr     -->> calculated c_w/4 ===Wing MAC 
 c_t=0.25;            % tail chrod  ************from xflr ct/cr ratio
 cmac_t=c_t/4;        % tail MAC    *************

L_w_t_mac=1000/1000;                 % wing to tail mac distance 
L_cg     =150/1000;                  % Cg of aircraft from wing leading edge 
Lmac_w_cg = L_cg-cmac_w/4;                % Wing MAC to A/C cg distance 
lt        =L_w_t_mac-Lmac_w_cg;       % A/C cg to tail MAC distance 
zeta=0.9;                             % tail to wing efficiency ratio
VH= 0.484 ;        

% tail volume ratio
S_tail=  VH*S_wing*cmac_w/lt;         % tail area 
AR    = Wspan/cmac_w;                    %
eps_0= (2*CL_0/(pi*AR));       % downwash angle in /rad 
del_eps_by_a=(2*CL_a_w/(pi*AR)); 
i_w   = 0;                            % wing incident angle in Deg 
i_t   =0;                             % tail incident angle 
Cm0_t = Cm_wb+zeta*VH*CL_a_t*(eps_0+i_w-i_t)*D2R;  % moment coeff of tail at alpha=0 in /Red
L_cg_NP= (((Cm_a-Cm_a_b+zeta*VH*CL_a_t*(1-del_eps_by_a))*cmac_w)/CL_a_w);
X_NP=L_cg+L_cg_NP;
% for lateral dynamics constants
Y1=0.48;                   % spanwise dist. from ceterline to airelon inboard edge
Y2=1;                   % spanwise dist. from ceterline to airelon outboard edge
zeta_emperical=Y1/(Wspan/2);
K_emperical=-0.12;        % calculated from graph given in nelsion pg-122
c_tip=0.25;               % tip chrod length 
c_root=0.35;              % root chrod length 
tap_ratio=c_tip/c_root;   % wing tapered ratio
lv=lt;                             % cg to vertical tail distance in m 
 Vv=S_v*lv/(S_wing*Wspan);          % vertical tail volume ratio
 zeta_v=0.95;                        % vertcal tail eficiency factor 
z_v=0.08;                         % distance from cp of vrtical tail to fuselage ceter line in m
z_w=0.06;                         % wing quarter chod to fueslage centerline || to x axis
dfusl=0.15;                       % fueslage depth 
one_plus_dsigma_by_beta=0.724+3.06*((S_v/S_wing)/(1+cosd(Sweep_roottip)))+0.4*(z_w/dfusl)+0.009*AR;

%% Trim condition constatns Initialization 
u0=16;   % Trim velocity in  m/sec
a=331.5;  % speed of sound in air
M=u0/a;
sim_pace=0;
guidance_mode_on=1;
 manual_control_on=0;
max_airelon_defl=30*pi/180;
max_rudder_defl= 50*pi/180;
max_elevtr_defl= 30*pi/180;
% constants 
Qd=0.5*rho*(u0)^2;
QS_m=Qd*S_wing/m_plane;
QSb=Qd*S_wing*Wspan;
QSb2u0=Qd*S_wing*(Wspan)^2/(2*u0);
QS_mu0=QS_m/u0;
c_2u0= cmac_w/(2*u0);
QSc_Iy= Qd*S_wing*cmac_w/Ibyy;
cons01=c_2u0*QSc_Iy;
% % Lift & drag coefficient, Induced Drag solope 
 e=0.8;   % Oswald efficiency 
 Kinduce=1/(pi*e*AR);
% LF=1; % load factor n=1 at cruise
% W=m_plane*g;
% CL=LF*W/(Qd*S_wing);           % Lift coefficient
% CD=CD_0+Kinduce*(CL)^2;        % Drag coefficient 
% F_lift=Qd*S_wing*CL;           % Lift at cruise condition 
% F_drag=Qd*S_wing*CD;           % Drag at cruise condition 
% 
% % Rate of decent
% ROD=(CD)*sqrt(2*W/(rho*S_wing))/(CL)^3/2;
% Y-Force Derivative 
Cy_beta=-(S_v/S_wing)*CL_a_v*one_plus_dsigma_by_beta;

Cy_p=CLmax*((AR+cosd(Sweep_roottip))/(AR+4*cosd(Sweep_roottip)))*tand(Sweep_roottip);

Cy_beta_t=-CL_a_v*zeta_v*(S_v/S_wing);
Cy_r=-2*(lv/Wspan)*Cy_beta_t;

Cy_dela=0;
Cy_delr=(S_v/S_wing)*tou*CL_a_v;

% Yawing moment derivative 
Cn_beta_wf=-1*0;             % it should be negative value due to fueslage and wing 
Cn_beta=Cn_beta_wf+Vv*CL_a_v*one_plus_dsigma_by_beta;
Cn_p=-CLmax/8;
Cn_r=-2*zeta_v*Vv*(lv/Wspan)*CL_a_v;

cons02=0.5*((Y2)^2-(Y1)^2);
cons03=((Y2)^3-(Y1)^3)/3;
cons04=(tap_ratio-1)/(0.5*Wspan);

Cl_dela=((2*CL_a_w*tou*c_root)/(S_wing*Wspan))*(cons02+cons03*cons04); 
Cn_dela=2*K_emperical*CL_0*Cl_dela;
Cn_delr=-Vv*zeta_v*tou*CL_a_v;
% Rolling moment derivative 
del_Cl_beta=0.0002;         % In /radian 
Cl_beta= del_Cl_beta;%-0.01;
Cl_p=-CL_a*(1+3*tap_ratio)/(12*(1+tap_ratio));
Cl_r=0.25*CLmax-((2*z_v*lv*Cy_beta_t)/(Wspan)^2);
Cl_delr=(S_v*z_v*tou*CL_a_w)/(S_wing*Wspan);

%% Stebility derivative calculation -Lateral  
Y_beta=QS_m*Cy_beta;
Y_p= QSb*Cy_p/(2*m_plane*u0);
Y_r=QSb*Cy_r/(2*m_plane*u0);
Y_dela=QS_m*Cy_dela;
Y_delr=QS_m*Cy_delr;
N_dela=QSb*Cn_dela/Ibzz;
N_delr=QSb*Cn_delr/Ibzz;
N_beta=QSb*Cn_beta/Ibzz;
N_p=QSb2u0*Cn_p/Ibxx;
N_r=QSb2u0*Cn_r/Ibxx;
L_dela=QSb*Cl_dela/Ibxx;
L_delr=QSb*Cl_delr/Ibxx;
L_beta=QSb*Cl_beta/Ibxx;
L_p=QSb2u0*Cl_p/Ibxx;
L_r=QSb2u0*Cl_r/Ibxx;

%% state space system of lateral dynamics 
A_lat=[Y_beta/u0  Y_p/u0  -(1-(Y_r)/(u0))  g/u0 ; L_beta  L_p  L_r  0  ;N_beta  N_p N_r 0  ; 0 1 0 0 ];
B_lat=[0 Y_delr/u0 ; L_dela   L_delr; N_dela N_delr ; 0 0];
C_lat=eye(4);
D_lat=zeros(4,2);
Q_lat=[B_lat A_lat*B_lat  (A_lat)^2*B_lat (A_lat)^3*B_lat];

AA_lat=[-0.254 0 -1 0.182 ; -16.02 -8.4 2.19 0 ; 4.48 -0.35 -0.76 0 ; 0 1 0 0];
%% Steability derivative calculation - Longitudnal
% stebility coefficient for matrix A
CT_u=-CD_0;
CD_u=0;
Cx_u=CT_u-(2*CD_0+CD_u);
Xu=Cx_u*QS_mu0;

Cx_w= -(CD_a-CL_0);
Xw=Cx_w*QS_mu0;

CL_u=((M)^2/(1-(M)^2))*CL_0;
Cz_u=-(CL_u+2*CL_0);
Zu=Cz_u*QS_mu0;

Cz_w= -(CL_a+CD_0);
Zw=Cz_w*QS_mu0;

Za=u0*Zw;

Cz_alphadot=-2*VH*zeta*CL_a_t*del_eps_by_a; 
Zwdot=Cz_alphadot*c_2u0*QS_mu0;

Zalphadot=u0*Zwdot;

Cz_q=-2*CL_a_t*zeta*VH;
Zq=Cz_q*c_2u0*QS_m;

Cz_de=-CL_a_t*tou*zeta*(S_t/S_wing); % elevator 
Zde  =Cz_de*QS_m;

Cm_u=0;
Mu=0;

Mw=Cm_a*QSc_Iy/u0;
Malpha=u0*Mw;

Cm_alphadot=-2*CL_a_t*zeta*VH*del_eps_by_a*(lt/cmac_w);
Mwdot= Cm_alphadot*c_2u0*QSc_Iy/u0;
Malphadot=u0*Mwdot;

Cm_q=-2*CL_a_t*zeta*VH*(lt/cmac_w);
Mq=Cm_q*cons01;

Cm_de=-zeta*VH*CL_a_t*tou;
Mde=Cm_de*QSc_Iy;

% stebility coefficient for matrix B
Cx_de=-0.1;
Xde=Cx_de*Qd*S_wing;
Xd=Xde/m_plane;

Xd_T=0.1*m_plane*g;
XdT=Xd_T/m_plane;

ZdT=0;

Md=Mde+(Mwdot*Zde);
M_dT=0;
MdT=(M_dT)/(Ibyy)+Mwdot*ZdT;

k0=-u0;

A_long=[Xu Xw   0  -g  0; Zu Zw  u0 0  0 ; Mu+Mwdot*Zu  Mw+Mwdot*Zw  Mq+Mwdot*u0  0  0; 0  0  1  0  0 ; 0 0 0 k0  0];
B_long=[ Xd XdT ; Zde   ZdT; Md  MdT; 0 0 ;0 0];

Qc_long= [ B_long   A_long*B_long (A_long)^2*B_long  (A_long)^3*B_long     (A_long)^4*B_long  ];


C_long=eye(5);
D_long=[ 0 0; 0 0; 0 0; 0 0; 0 0];
 



% XYZ=out.pe;
% north_pos=XYZ(:,1);
% east_pos=XYZ(:,2);
% up_pos=XYZ(:,3);

figure(5)
h5=plot3(north_pos,east_pos,-up_pos);
grid on 
xlabel('North[m]') 
ylabel('East[m]') 
zlabel('Altitude[m]')  
title('NED Position of MAV')

%% longitudnal satatic steability calculation 
%% PLot
figure(1)
h1=plot(alpha,CL);
legend('Lift coefficient','LineWidth=2');
grid on

figure(2)
h2=plot(alpha,CD);
legend('Drag coefficient','LineWidth=2');

grid on
figure(3)
h3=plot(alpha,Cm);
legend('Pitching moment  coefficient','LineWidth=2');
grid on
figure(4)
h4=plot(alpha,Cm);
legend('Aero moment  coefficient','LineWidth=2');
grid on

model = fixedwing;

s = state(model);
s(4) = 5; % 5 m/s
u = control(model);
u.RollAngle = pi/3;
u.AirSpeed = 15;
e = environment(model);

sdot = derivative(model,s,u,e);
simOut = ode45(@(~,x)derivative(model,x,u,e), [0 50], s);
size(simOut.y)
figure(5)
h5=plot(simOut.y(7,:));

downsample = 1:30:size(simOut.y,2);
translations = simOut.y(1:3,downsample)'; % xyz-position
rotations = eul2quat([simOut.y(5,downsample)',simOut.y(6,downsample)',simOut.y(7,downsample)']); % ZYX Euler
plotTransforms(translations,rotations,...
    'MeshFilePath','fixedwing.stl','InertialZDirection',"down")
hold on
plot3(simOut.y(1,:),-simOut.y(2,:),simOut.y(3,:),'--b') % full path
xlim([-180.0 40.0])
ylim([-140.0 50])
zlim([-0.5 1])
view([-450 90])
hold off

% openExample('uav/MotionPlanningWithRRTForAFixedWingUAVExample')

% Set RNG seed for repeatable result
rng(1,"twister");

mapData = load("uavMapCityBlock.mat","omap");
omap = mapData.omap;
% Consider unknown spaces to be unoccupied
omap.FreeThreshold = omap.OccupiedThreshold;

startPose = [12  22  50    pi/2];
goalPose = [150 180  50   pi/2]; 
figure("Name","StartAndGoal")
hMap = show(omap);
hold on
scatter3(hMap,startPose(1),startPose(2),startPose(3),30,"red","filled")
scatter3(hMap,goalPose(1),goalPose(2),goalPose(3),30,"green","filled")
hold off
view([-31 63])

ss = ExampleHelperUAVStateSpace("MaxRollAngle",pi/6,...
                                "AirSpeed",15,...
                                "FlightPathAngleLimit",[-0.1 0.1],...
                                "Bounds",[-20 220; -20 220; 10 100; -pi pi]);
threshold = [(goalPose-0.5)' (goalPose+0.5)'; -pi pi];
setWorkspaceGoalRegion(ss,goalPose,threshold)
sv = validatorOccupancyMap3D(ss,"Map",omap);
sv.ValidationDistance = 0.1;

planner = plannerRRT(ss,sv);
planner.MaxConnectionDistance =150;
planner.GoalBias = 0.10;  
planner.MaxIterations = 400;
planner.GoalReachedFcn = @(~,x,y)(norm(x(1:3)-y(1:3)) < 10);

[pthObj,solnInfo] = plan(planner,startPose,goalPose);

if (solnInfo.IsPathFound)
    figure("Name","OriginalPath")
    % Visualize the 3-D map
    show(omap)
    hold on
    scatter3(startPose(1),startPose(2),startPose(3),30,"red","filled")
    scatter3(goalPose(1),goalPose(2),goalPose(3),30,"green","filled")
    
    interpolatedPathObj = copy(pthObj);
    interpolate(interpolatedPathObj,1000)
    
    % Plot the interpolated path based on UAV Dubins connections
    hReference = plot3(interpolatedPathObj.States(:,1), ...
        interpolatedPathObj.States(:,2), ...
        interpolatedPathObj.States(:,3), ...
        "LineWidth",2,"Color","g");
    
    % Plot simulated UAV trajectory based on fixed-wing guidance model
    % Compute total time of flight and add a buffer
    timeToReachGoal = 1.05*pathLength(pthObj)/ss.AirSpeed;
    waypoints = interpolatedPathObj.States;
    [xENU,yENU,zENU] = exampleHelperSimulateUAV(waypoints,ss.AirSpeed,timeToReachGoal);
    hSimulated = plot3(xENU,yENU,zENU,"LineWidth",2,"Color","r");
    legend([hReference,hSimulated],"Reference","Simulated","Location","best")
    hold off
    view([-31 63])
end


syms L  W  b d  w1 w2 w3 w4 u1 u2 u3 u4 

matrix=[ b b b b ; -W*b W*b W*b -W*b;W*b W*b -L*b  -L*b ; d  d -d -d ];
om_sq=inv(matrix).*[u1 u2 u3 u4];