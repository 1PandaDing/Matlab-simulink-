function [sys,x0,str,ts] = chapter5_2_2(t,x,u,flag)

% Q_40*40,R_5*5,a_6*6,b_6*1,d_k 6*1,A_7*7,B_7*1,C_2*7
% Nx=6;%״̬���ĸ�����Nu=1;%�������ĸ�����Ny=2;%������ĸ�����Np =20;%Ԥ��ʱ��Nc=5;%����ʱ��
% ״̬��=[y_dot,x_dot,phi,phi_dot,Y,X]��������Ϊǰ��ƫ��delta_f

switch flag,
 case 0
  [sys,x0,str,ts] = mdlInitializeSizes; % Initialization
  
 case 2
  sys = mdlUpdates(t,x,u); % Update discrete states
  
 case 3
  sys = mdlOutputs(t,x,u); % Calculate outputs
 
%  case 4
%   sys = mdlGetTimeOfNextVarHit(t,x,u); % Get next sample time 

 case {1,4,9} % Unused flags
  sys = [];
  
 otherwise
  error(['unhandled flag = ',num2str(flag)]); % Error handling
end
% End of dsfunc.

%==============================================================
% Initialization
%==============================================================

function [sys,x0,str,ts] = mdlInitializeSizes

% Call simsizes for a sizes structure, fill it in, and convert it 
% to a sizes array.

sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 6;
sizes.NumOutputs     = 1;
sizes.NumInputs      = 8;
sizes.DirFeedthrough = 1; % Matrix D is non-empty.
sizes.NumSampleTimes = 1;
sys = simsizes(sizes); 
x0 =[0.001;0.0001;0.0001;0.00001;0.00001;0.00001];    
global U;%UΪ���ǵĿ�����
U=[0];%

% Initialize the discrete states.
str = [];             % Set str to an empty matrix.
ts  = [0.05 0];       % sample time: [period, offset]������ʱ��Ӱ�����
  
global VehiclePara; 
    VehiclePara.m   = 1723;   %mΪ��������,Kg; Sprung mass = 1370
    VehiclePara.g   = 9.8;
    VehiclePara.Lf  = 1.232;  % 1.05
    VehiclePara.Lr  = 1.468;  % 1.55
    VehiclePara.L   = 2.6;  %VehiclePara.Lf + VehiclePara.Lr;
    VehiclePara.Iz  = 4175;   %IΪ������Z���ת���������������в���  
    VehiclePara.Ccf  = 66900;
    VehiclePara.Ccr  = 62700;
    VehiclePara.Clf  = 66900;
    VehiclePara.Clr  = 62700;
    
global MPCParameters; 
%     MPCParameters.Np  = 11;% predictive horizon Assume Np=Nc
%     MPCParameters.Nc  = 5; %  Tsplit
    MPCParameters.Ts  = 0.05; % the sample time of near term  
%     MPCParameters.Nx  = 6; %the number of state variables
%     MPCParameters.Ny  = 2; %the number of output variables      
%     MPCParameters.Nu  = 1; %the number of control inputs
%End of mdlInitializeSizes


% Update the discrete states
function sys = mdlUpdates(t,x,u)
  
sys = x;
%End of mdlUpdate.

%==============================================================
% Calculate outputs
%==============================================================
function sys = mdlOutputs(t,x,u)
    global VehiclePara;
    global MPCParameters; 
    global U; %����UΪȫ����
    tic
    Nx=6;%״̬���ĸ���
    Nu=1;%�������ĸ���
    Ny=2;%������ĸ���
    Np=20;%Ԥ��ʱ��
    Nc=5;%����ʱ��
    Row=1000;%�ɳ�����Ȩ��
    fprintf('Update start, t=%6.3f\n',t)
   

    y_dot=u(1)/3.6; %�����ٶȻ�Ϊm/s
    x_dot=u(2)/3.6+0.0001;%CarSim�������km/h��ת��Ϊm/s
    phi=u(3)*3.141592654/180; %CarSim�����Ϊ�Ƕȣ��Ƕ�ת��Ϊ���ȣ�ʹ�û���
    phi_dot=u(4)*3.141592654/180;%���ٶ�
    Y=u(5);%��λΪm������λ��
    X=u(6);%��λΪ�ף�����λ��
    Sf= u(7);%��ǰ�ֻ�����
    Sr= u(8);%��ǰ�ֻ�����
    
% %% ������������
%syms sfΪǰ�ֻ����ʣ�srΪ���ֻ�����
    Sf= u(7); Sr= u(8);
%syms lf%ǰ�־��복�����ĵľ��룬lrΪ���־��복�����ĵľ���
    Lf=1.232;Lr=1.468;
%syms C_cfǰ�����Ժ����ƫ�նȣ� C_cr�������Ժ����ƫ�ն� ��C_lf ǰ�������ƫ�նȣ� C_lr ���������ƫ�ն�
    Ccf=66900;Ccr=62700;Clf=66900;Clr=62700;
%syms m g I;%mΪ����������gΪ�������ٶȣ�IΪ������Z���ת���������������в���
    m=1723;g=9.8;I=4175;
%    

%% �ο��켣����
    X_predict=zeros(Np,1);%���ڱ���Ԥ��ʱ���ڵ�����λ����Ϣ�����Ǽ��������켣�Ļ���
    phi_ref=zeros(Np,1);%���ڱ���Ԥ��ʱ���ڵĲο���ڽ���Ϣ
    Y_ref=zeros(Np,1);%���ڱ���Ԥ��ʱ���ڵĲο�����λ����Ϣ
    load Reftra2
    % %  �����ο��켣ͼ��
%     W=6;
%     d=120;
%     X=0:0.05:d;
%     Y=W*(10*(X/d).^3-15*(X/d).^4+6*(X/d).^5);
%     figure(1);
%     plot(X, Y,'r--','LineWidth',2);
%     hold on;
%     plot(y(:,6),y(:,5));
   
    %  ���¼���״̬������״̬�������������һ�� 
    kesi=zeros(Nx+Nu,1);%״̬��6����������1��
    kesi(1)=y_dot;%
    kesi(2)=x_dot;
    kesi(3)=phi; 
    kesi(4)=phi_dot;
    kesi(5)=Y;
    kesi(6)=X; 
    kesi(7)=U(1); %����ǿ�����ǰ��ƫ�ǣ����ڹ켣���ٹ�����ͨ����ǰ��ƫ�ǽ��п��ƣ������ٶȱ��ֲ���
    delta_f=U(1);%ǰ��ת��
    fprintf('Update start, u(1)=%4.2f\n',U(1))

    T=0.05;%���沽��
    T_all=20;%�ܵķ���ʱ�䣬��Ҫ�����Ƿ�ֹ���������켣Խ��
     
    %Ȩ�ؾ������� 
    Q_cell=cell(Np,Np);%�ܵ�Ԫ��QΪ20��20�еģ�40*40
    for i=1:1:Np
        for j=1:1:Np
            if i==j
                  Q_cell{i,j}=[3000 0;0 5000;]; %�����ǽ�����Խ��ߵĵط�����Ϊ�þ�������Ϊֹȫ��Ϊ0
            else 
                Q_cell{i,j}=zeros(Ny,Ny);    %20��20�еľ�����ÿһ��С��λ���϶���2*2��С����           
            end
        end 
    end 
    R=50000*eye(Nu*Nc);

 %���ö���ѧģ�ͣ��þ����복������������أ�ͨ���Զ���ѧ��������ſ˱Ⱦ���õ���aΪ6*6��bΪ6*1
    [a, b] = func_Model_linearization_Jacobian(kesi,Sf,Sr,MPCParameters,VehiclePara);
    
    d_k=zeros(Nx,1);%����ƫ��,6*1
    state_k1=zeros(Nx,1);%Ԥ����һʱ��״̬�������ڼ���ƫ�6*1
    %���¼�Ϊ������ɢ������ģ��Ԥ����һʱ��״̬��
    %ע�⣬Ϊ����ǰ�����ı��ʽ��a,b�����������a,b�����ͻ����ǰ�����ı��ʽ��Ϊlf��lr��������Щ��ʽ����P35��
    %������������ʱ��ģ��Ԥ��·�����ٿ���
    state_k1(1,1)=y_dot+T*(-x_dot*phi_dot+2*(Ccf*(delta_f-(y_dot+Lf*phi_dot)/x_dot)+Ccr*(Lr*phi_dot-y_dot)/x_dot)/m);
    state_k1(2,1)=x_dot+T*(y_dot*phi_dot+2*(Clf*Sf+Clr*Sr+Ccf*delta_f*(delta_f-(y_dot+phi_dot*Lf)/x_dot))/m);
    state_k1(3,1)=phi+T*phi_dot;
    state_k1(4,1)=phi_dot+T*((2*Lf*Ccf*(delta_f-(y_dot+Lf*phi_dot)/x_dot)-2*Lr*Ccr*(Lr*phi_dot-y_dot)/x_dot)/I);
    state_k1(5,1)=Y+T*(x_dot*sin(phi)+y_dot*cos(phi));
    state_k1(6,1)=X+T*(x_dot*cos(phi)-y_dot*sin(phi)); %����Ϊֹ����һʱ�̵�״̬���Ѿ��Ƶ����
    d_k=state_k1-a*kesi(1:6,1)-b*kesi(7,1);
    d_piao_k=zeros(Nx+Nu,1);%ƫ�����d_k��������ʽ���ּ��˿���������һ�У�ά��7*1�ο�falcone(B,4c)
    d_piao_k(1:6,1)=d_k; %��d_k��������ʽ���и�ֵ��ǰ6��Ϊ���ǵ�ƫ��
    d_piao_k(7,1)=0;
    
    A_cell=cell(2,2);
    B_cell=cell(2,1);
    A_cell{1,1}=a;%��һ�е�һ��6*6
    A_cell{1,2}=b;%��һ�еڶ���Ϊ6*1
    A_cell{2,1}=zeros(Nu,Nx);%�ڶ��е�һ��Ϊ1*6
    A_cell{2,2}=eye(Nu);%����еڶ���Ϊ1*1
    B_cell{1,1}=b;%B�����һ�е�һ��Ϊb,6*1
    B_cell{2,1}=eye(Nu);%�ڶ��е�һ��Ϊ1*1��λ��
    A=cell2mat(A_cell);%A����ά��7*7
    B=cell2mat(B_cell);%B����ά��7*1
    C=[0 0 1 0 0 0 0;0 0 0 0 1 0 0;];%����ֻ���״̬���ռ�ĵ�����������ڽǣ��͵������������ƫ������

    PSI_cell=cell(Np,1);%������̵ĵ�һ��ϵ������ά�� 20*1
    THETA_cell=cell(Np,Nc);%������̵ĵڶ���ϵ������ά�� 20*5
    GAMMA_cell=cell(Np,Np);%ά�� 20*20
    PHI_cell=cell(Np,1);%ά�� 20*1

    for p=1:1:Np
        PHI_cell{p,1}=d_piao_k;%  7��1�е�ƫ���������˵�������Ҫʵʱ���µģ�����Ϊ�˼�㣬������һ�ν���
        for q=1:1:Np
            if q<=p  %�����Ǿ���
                GAMMA_cell{p,q}=C*A^(p-q); %�þ�����C��A�������
            else 
                GAMMA_cell{p,q}=zeros(Ny,Nx+Nu); %ÿһ��Ԫ���Ĵ�СΪ2�������������*7��״̬�ռ����ĸ�����
            end 
        end
    end  %���ոþ���Ϊ40*140
 
    for j=1:1:Np %j��1��20
     PSI_cell{j,1}=C*A^j; %�����������ʽ��21���е�һ���ϵ������2*7*7*7=2*7
        for k=1:1:Nc %k��1��5�������������ʽ��21���еڶ����ϵ������
            if k<=j  %�����ǵ�������ֵ
                THETA_cell{j,k}=C*A^(j-k)*B;  
            else %����������ȫ��Ϊ0
                THETA_cell{j,k}=zeros(Ny,Nu); %ÿһ��СԪ������2*1С����
            end
        end
    end
    
    PSI=cell2mat(PSI_cell);%size(PSI)=[Ny*Np Nx+Nu]����20��1�У�ÿһ��СԪ������2*7��С��������Ϊ40*7
    THETA=cell2mat(THETA_cell);%size(THETA)=[Ny*Np Nu*Nc]��40*5
    
    GAMMA=cell2mat(GAMMA_cell);%��д��GAMMA��ά��40*140
    PHI=cell2mat(PHI_cell);%���ڸ�Ԫ������ÿһ��λ�ö���7*1�ľ���һ��20�У�����һ����140*1
    %144���Ѿ����QԪ�����飬���ڽ���ת���ɾ�����ʽ
    Q=cell2mat(Q_cell); %�ܵ�Ԫ��QΪ20��20�еģ�ÿ��λ�ö���2*2��С����Q����Ϊ40*40
    %������ι滮��H����
    H_cell=cell(2,2);
    H_cell{1,1}=2*(THETA'*Q*THETA+R);%��һ�е�һ�е�λ��Ϊ5*5�ľ���
    H_cell{1,2}=zeros(Nu*Nc,1);%��һ�еڶ��е�λ��Ϊ5*1
    H_cell{2,1}=zeros(1,Nu*Nc);%�ڶ��е�һ�е�λ��Ϊ1*5
    H_cell{2,2}=2*Row;%�ڶ��еڶ���Ϊһ��ֵ���ɳ�����
    H=cell2mat(H_cell);%�������֮��HԪ������ת��Ϊ��������ΪHΪ6*6
    H=(H+H')/2;
    error_1=zeros(Ny*Np,1);%40*7*7*1=40*1
    Yita_ref_cell=cell(Np,1);%�ο���Ԫ������Ϊ20*1
    for p=1:1:Np
        X_DOT=x_dot*cos(phi)-y_dot*sin(phi);%��������ϵ�������ٶ�
        X_predict(p,1)=X+X_DOT*p*T;%���ȼ����δ��X��λ�ã�
        %�ο�·����Ϣ�����ο�������λ�ã�����λ�ã���ڽ�
        last=size(path5,1);
        if X_predict(p,1)>=path5(last,1)
            Y_ref(p,1)=path5(last,2);
            phi_ref(p,1)=path5(last,3);
            Yita_ref_cell{p,1}=[phi_ref(p,1);Y_ref(p,1)];
        else
            x_ref=path5(:,1); %���õ�����λ��
            y_ref=path5(:,2); %���õĺ���λ��
            phi_ref=path5(:,3);
            Y_ref(p,1)=interp1(x_ref,y_ref,X_predict(p,1));
            phi_ref(p,1)=interp1(x_ref,phi_ref,X_predict(p,1));
            Yita_ref_cell{p,1}=[phi_ref(p,1);Y_ref(p,1)];
        end
    end
    Yita_ref=cell2mat(Yita_ref_cell);%�����ǵõ������Ԫ��ת��Ϊ����
    error_1=Yita_ref-PSI*kesi-GAMMA*PHI; 
     g_cell=cell(1,2);
     g_cell{1,1}=2*error_1'*Q*THETA;
     g_cell{1,2}=0;
     g=-cell2mat(g_cell);
     f=g';

 %% ����ΪԼ����������
 %������Լ��
    A_t=zeros(Nc,Nc);
    for p=1:1:Nc
        for q=1:1:Nc
            if q<=p %�����Ǿ�������Խ���
                A_t(p,q)=1;
            else 
                A_t(p,q)=0;
            end
        end 
    end 
    A_I=kron(A_t,eye(Nu));%������ڿ˻�
    Ut=kron(ones(Nc,1),U(1));

    umin=-0.3744;%ά������Ʊ����ĸ�����ͬ��ǰ��ƫ�ǵ���Լ��
    umax=0.3744;%ǰ��ƫ�ǵ���Լ��
    Umin=kron(ones(Nc,1),umin);
    Umax=kron(ones(Nc,1),umax);
    
    %�����Լ��
    ycmax=[1;4];  %��ڽǺ�����λ�Ƶ�Լ��
    ycmin=[-1;-1];
    Ycmax=kron(ones(Np,1),ycmax);
    Ycmin=kron(ones(Np,1),ycmin);
    
%     %���ι滮A����
%     A_cons_cell={A_I zeros(Nu*Nc,1);-A_I zeros(Nu*Nc,1);THETA zeros(Ny*Np,1);-THETA zeros(Ny*Np,1)};
%     A_cons=cell2mat(A_cons_cell);%����ⷽ�̣�״̬������ʽԼ���������ת��Ϊ����ֵ��ȡֵ��Χ
%     
%     %���ι滮��b����
%     b_cons_cell={Umax-Ut;-Umin+Ut;Ycmax-PSI*kesi-GAMMA*PHI;-Ycmin+PSI*kesi+GAMMA*PHI};
%     b_cons=cell2mat(b_cons_cell);%����ⷽ�̣�״̬������ʽԼ����ȡֵ

    %���ι滮A����
    A_cons_cell={A_I zeros(Nu*Nc,1);-A_I zeros(Nu*Nc,1);};
    A_cons=cell2mat(A_cons_cell);%����ⷽ�̣�״̬������ʽԼ���������ת��Ϊ����ֵ��ȡֵ��Χ

    %���ι滮��b����
    b_cons_cell={Umax-Ut;-Umin+Ut;};
    b_cons=cell2mat(b_cons_cell);%����ⷽ�̣�״̬������ʽԼ����ȡֵ
    
    %��������Լ��
    M=10; 
    delta_umin=-0.248;%ǰ��ƫ�Ǳ仯������Լ��
    delta_umax=0.248;%ǰ��ƫ�Ǳ仯������Լ��
    delta_Umin=kron(ones(Nc,1),delta_umin);
    delta_Umax=kron(ones(Nc,1),delta_umax);
    lb=[delta_Umin;0];%����ⷽ�̣�״̬���½磬��������ʱ���ڿ����������ɳ�����
    ub=[delta_Umax;M];%����ⷽ�̣�״̬���Ͻ磬��������ʱ���ڿ����������ɳ�����
    
    %% ��ʼ������
    %% ��ʼ������
   options = optimset('Display','off', ...
        'TolFun', 1e-8, ...
        'MaxIter', 2000, ...
        'Algorithm', 'interior-point-convex', ...
        'FinDiffType', 'forward', ...
        'RelLineSrchBnd', [], ...
        'RelLineSrchBndDuration', 1, ...
        'TolConSQP', 1e-8);
    warning off all  % close the warnings during computation
%     x_start=zeros(Nc+1,1);%����һ����ʼ��
    [X,fval,exitflag]=quadprog(H,f,A_cons,b_cons,[],[],lb,ub,[],options);
    fprintf('exitflag=%d\n',exitflag);
    fprintf('H=%4.2f\n',H(1,1));
    fprintf('f=%4.2f\n',f(1,1));
    %% �������
    if isempty(X) 
    U(1)=0;
    else
    fprintf("X(1)=%4.2f\n",X(1));
    u_piao=X(1);%�õ���������
    U(1)=kesi(7,1)+u_piao;%��ǰʱ�̵Ŀ�����Ϊ��һ��ʱ�̿���+��������
    sys= U;
    toc
    end
