function [sys,x0,str,ts] = chapter5_2_2(t,x,u,flag)

% Q_40*40,R_5*5,a_6*6,b_6*1,d_k 6*1,A_7*7,B_7*1,C_2*7
% Nx=6;%状态量的个数；Nu=1;%控制量的个数；Ny=2;%输出量的个数；Np =20;%预测时域；Nc=5;%控制时域
% 状态量=[y_dot,x_dot,phi,phi_dot,Y,X]，控制量为前轮偏角delta_f

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
global U;%U为我们的控制量
U=[0];%

% Initialize the discrete states.
str = [];             % Set str to an empty matrix.
ts  = [0.05 0];       % sample time: [period, offset]，采样时间影响最大。
  
global VehiclePara; 
    VehiclePara.m   = 1723;   %m为车辆质量,Kg; Sprung mass = 1370
    VehiclePara.g   = 9.8;
    VehiclePara.Lf  = 1.232;  % 1.05
    VehiclePara.Lr  = 1.468;  % 1.55
    VehiclePara.L   = 2.6;  %VehiclePara.Lf + VehiclePara.Lr;
    VehiclePara.Iz  = 4175;   %I为车辆绕Z轴的转动惯量，车辆固有参数  
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
    global U; %设置U为全局量
    tic
    Nx=6;%状态量的个数
    Nu=1;%控制量的个数
    Ny=2;%输出量的个数
    Np=20;%预测时域
    Nc=5;%控制时域
    Row=1000;%松弛因子权重
    fprintf('Update start, t=%6.3f\n',t)
   

    y_dot=u(1)/3.6; %横向速度化为m/s
    x_dot=u(2)/3.6+0.0001;%CarSim输出的是km/h，转换为m/s
    phi=u(3)*3.141592654/180; %CarSim输出的为角度，角度转换为弧度，使用弧度
    phi_dot=u(4)*3.141592654/180;%角速度
    Y=u(5);%单位为m，横向位置
    X=u(6);%单位为米，纵向位置
    Sf= u(7);%左前轮滑移率
    Sr= u(8);%右前轮滑移率
    
% %% 车辆参数输入
%syms sf为前轮滑移率，sr为后轮滑移率
    Sf= u(7); Sr= u(8);
%syms lf%前轮距离车辆质心的距离，lr为后轮距离车辆质心的距离
    Lf=1.232;Lr=1.468;
%syms C_cf前轮线性横向侧偏刚度； C_cr后轮线性横向侧偏刚度 ；C_lf 前轮纵向侧偏刚度； C_lr 后轮纵向侧偏刚度
    Ccf=66900;Ccr=62700;Clf=66900;Clr=62700;
%syms m g I;%m为车辆质量，g为重力加速度，I为车辆绕Z轴的转动惯量，车辆固有参数
    m=1723;g=9.8;I=4175;
%    

%% 参考轨迹生成
    X_predict=zeros(Np,1);%用于保存预测时域内的纵向位置信息，这是计算期望轨迹的基础
    phi_ref=zeros(Np,1);%用于保存预测时域内的参考横摆角信息
    Y_ref=zeros(Np,1);%用于保存预测时域内的参考横向位置信息
    load Reftra2
    % %  画出参考轨迹图像
%     W=6;
%     d=120;
%     X=0:0.05:d;
%     Y=W*(10*(X/d).^3-15*(X/d).^4+6*(X/d).^5);
%     figure(1);
%     plot(X, Y,'r--','LineWidth',2);
%     hold on;
%     plot(y(:,6),y(:,5));
   
    %  以下计算状态量，即状态量与控制量合在一起 
    kesi=zeros(Nx+Nu,1);%状态量6个，控制量1个
    kesi(1)=y_dot;%
    kesi(2)=x_dot;
    kesi(3)=phi; 
    kesi(4)=phi_dot;
    kesi(5)=Y;
    kesi(6)=X; 
    kesi(7)=U(1); %这个是控制量前轮偏角，即在轨迹跟踪过程中通过对前轮偏角进行控制，纵向速度保持不变
    delta_f=U(1);%前轮转角
    fprintf('Update start, u(1)=%4.2f\n',U(1))

    T=0.05;%仿真步长
    T_all=20;%总的仿真时间，主要功能是防止计算期望轨迹越界
     
    %权重矩阵设置 
    Q_cell=cell(Np,Np);%总的元胞Q为20行20列的，40*40
    for i=1:1:Np
        for j=1:1:Np
            if i==j
                  Q_cell{i,j}=[3000 0;0 5000;]; %作用是将方阵对角线的地方设置为该矩阵，其余为止全部为0
            else 
                Q_cell{i,j}=zeros(Ny,Ny);    %20行20列的矩阵中每一个小的位置上都是2*2的小矩阵           
            end
        end 
    end 
    R=50000*eye(Nu*Nc);

 %采用动力学模型，该矩阵与车辆参数密切相关，通过对动力学方程求解雅克比矩阵得到，a为6*6，b为6*1
    [a, b] = func_Model_linearization_Jacobian(kesi,Sf,Sr,MPCParameters,VehiclePara);
    
    d_k=zeros(Nx,1);%计算偏差,6*1
    state_k1=zeros(Nx,1);%预测下一时刻状态量，用于计算偏差，6*1
    %以下即为根据离散非线性模型预测下一时刻状态量
    %注意，为避免前后轴距的表达式（a,b）与控制器的a,b矩阵冲突，将前后轴距的表达式改为lf和lr，下面这些公式见（P35）
    %自主车辆线性时变模型预测路径跟踪控制
    state_k1(1,1)=y_dot+T*(-x_dot*phi_dot+2*(Ccf*(delta_f-(y_dot+Lf*phi_dot)/x_dot)+Ccr*(Lr*phi_dot-y_dot)/x_dot)/m);
    state_k1(2,1)=x_dot+T*(y_dot*phi_dot+2*(Clf*Sf+Clr*Sr+Ccf*delta_f*(delta_f-(y_dot+phi_dot*Lf)/x_dot))/m);
    state_k1(3,1)=phi+T*phi_dot;
    state_k1(4,1)=phi_dot+T*((2*Lf*Ccf*(delta_f-(y_dot+Lf*phi_dot)/x_dot)-2*Lr*Ccr*(Lr*phi_dot-y_dot)/x_dot)/I);
    state_k1(5,1)=Y+T*(x_dot*sin(phi)+y_dot*cos(phi));
    state_k1(6,1)=X+T*(x_dot*cos(phi)-y_dot*sin(phi)); %到此为止，下一时刻的状态量已经推导完毕
    d_k=state_k1-a*kesi(1:6,1)-b*kesi(7,1);
    d_piao_k=zeros(Nx+Nu,1);%偏差矩阵d_k的增广形式，又加了控制量的那一行，维度7*1参考falcone(B,4c)
    d_piao_k(1:6,1)=d_k; %给d_k的增广形式进行赋值，前6行为我们的偏差
    d_piao_k(7,1)=0;
    
    A_cell=cell(2,2);
    B_cell=cell(2,1);
    A_cell{1,1}=a;%第一行第一列6*6
    A_cell{1,2}=b;%第一行第二列为6*1
    A_cell{2,1}=zeros(Nu,Nx);%第二行第一列为1*6
    A_cell{2,2}=eye(Nu);%最二行第二列为1*1
    B_cell{1,1}=b;%B矩阵第一行第一列为b,6*1
    B_cell{2,1}=eye(Nu);%第二行第一列为1*1单位阵
    A=cell2mat(A_cell);%A矩阵维度7*7
    B=cell2mat(B_cell);%B矩阵维度7*1
    C=[0 0 1 0 0 0 0;0 0 0 0 1 0 0;];%我们只输出状态看空间的第三个量（横摆角）和第五个量（横向偏移量）

    PSI_cell=cell(Np,1);%输出方程的第一个系数矩阵，维度 20*1
    THETA_cell=cell(Np,Nc);%输出方程的第二个系数矩阵，维度 20*5
    GAMMA_cell=cell(Np,Np);%维度 20*20
    PHI_cell=cell(Np,1);%维度 20*1

    for p=1:1:Np
        PHI_cell{p,1}=d_piao_k;%  7行1列的偏差，理论上来说，这个是要实时更新的，但是为了简便，这里又一次近似
        for q=1:1:Np
            if q<=p  %下三角矩阵
                GAMMA_cell{p,q}=C*A^(p-q); %该矩阵由C和A构造而成
            else 
                GAMMA_cell{p,q}=zeros(Ny,Nx+Nu); %每一个元胞的大小为2（输出两个数）*7（状态空间量的个数）
            end 
        end
    end  %最终该矩阵为40*140
 
    for j=1:1:Np %j从1到20
     PSI_cell{j,1}=C*A^j; %构造输出方程式（21）中第一项的系数矩阵，2*7*7*7=2*7
        for k=1:1:Nc %k从1到5，构造输出方程式（21）中第二项的系数矩阵
            if k<=j  %下三角的区域有值
                THETA_cell{j,k}=C*A^(j-k)*B;  
            else %上三角区域全部为0
                THETA_cell{j,k}=zeros(Ny,Nu); %每一个小元胞都是2*1小矩阵
            end
        end
    end
    
    PSI=cell2mat(PSI_cell);%size(PSI)=[Ny*Np Nx+Nu]，共20行1列，每一个小元胞都是2*7的小矩阵，所以为40*7
    THETA=cell2mat(THETA_cell);%size(THETA)=[Ny*Np Nu*Nc]，40*5
    
    GAMMA=cell2mat(GAMMA_cell);%大写的GAMMA；维度40*140
    PHI=cell2mat(PHI_cell);%由于该元胞数组每一个位置都是7*1的矩阵，一共20行，所以一共是140*1
    %144行已经求得Q元胞数组，现在将其转换成矩阵形式
    Q=cell2mat(Q_cell); %总的元胞Q为20行20列的，每个位置都是2*2的小矩阵，Q最终为40*40
    %构造二次规划的H矩阵
    H_cell=cell(2,2);
    H_cell{1,1}=2*(THETA'*Q*THETA+R);%第一行第一列的位置为5*5的矩阵
    H_cell{1,2}=zeros(Nu*Nc,1);%第一行第二列的位置为5*1
    H_cell{2,1}=zeros(1,Nu*Nc);%第二行第一列的位置为1*5
    H_cell{2,2}=2*Row;%第二行第二列为一个值：松弛因子
    H=cell2mat(H_cell);%构造完毕之后将H元胞数组转化为矩阵，最终为H为6*6
    H=(H+H')/2;
    error_1=zeros(Ny*Np,1);%40*7*7*1=40*1
    Yita_ref_cell=cell(Np,1);%参考的元胞数组为20*1
    for p=1:1:Np
        X_DOT=x_dot*cos(phi)-y_dot*sin(phi);%惯性坐标系下纵向速度
        X_predict(p,1)=X+X_DOT*p*T;%首先计算出未来X的位置，
        %参考路径信息包括参考的纵向位置，横向位置，横摆角
        last=size(path5,1);
        if X_predict(p,1)>=path5(last,1)
            Y_ref(p,1)=path5(last,2);
            phi_ref(p,1)=path5(last,3);
            Yita_ref_cell{p,1}=[phi_ref(p,1);Y_ref(p,1)];
        else
            x_ref=path5(:,1); %设置的纵向位移
            y_ref=path5(:,2); %设置的横向位移
            phi_ref=path5(:,3);
            Y_ref(p,1)=interp1(x_ref,y_ref,X_predict(p,1));
            phi_ref(p,1)=interp1(x_ref,phi_ref,X_predict(p,1));
            Yita_ref_cell{p,1}=[phi_ref(p,1);Y_ref(p,1)];
        end
    end
    Yita_ref=cell2mat(Yita_ref_cell);%将我们得到的输出元胞转换为矩阵
    error_1=Yita_ref-PSI*kesi-GAMMA*PHI; 
     g_cell=cell(1,2);
     g_cell{1,1}=2*error_1'*Q*THETA;
     g_cell{1,2}=0;
     g=-cell2mat(g_cell);
     f=g';

 %% 以下为约束生成区域
 %控制量约束
    A_t=zeros(Nc,Nc);
    for p=1:1:Nc
        for q=1:1:Nc
            if q<=p %下三角矩阵包含对角线
                A_t(p,q)=1;
            else 
                A_t(p,q)=0;
            end
        end 
    end 
    A_I=kron(A_t,eye(Nu));%求克罗内克积
    Ut=kron(ones(Nc,1),U(1));

    umin=-0.3744;%维数与控制变量的个数相同，前轮偏角的上约束
    umax=0.3744;%前轮偏角的下约束
    Umin=kron(ones(Nc,1),umin);
    Umax=kron(ones(Nc,1),umax);
    
    %输出量约束
    ycmax=[1;4];  %横摆角和纵向位移的约束
    ycmin=[-1;-1];
    Ycmax=kron(ones(Np,1),ycmax);
    Ycmin=kron(ones(Np,1),ycmin);
    
%     %二次规划A矩阵
%     A_cons_cell={A_I zeros(Nu*Nc,1);-A_I zeros(Nu*Nc,1);THETA zeros(Ny*Np,1);-THETA zeros(Ny*Np,1)};
%     A_cons=cell2mat(A_cons_cell);%（求解方程）状态量不等式约束增益矩阵，转换为绝对值的取值范围
%     
%     %二次规划的b矩阵
%     b_cons_cell={Umax-Ut;-Umin+Ut;Ycmax-PSI*kesi-GAMMA*PHI;-Ycmin+PSI*kesi+GAMMA*PHI};
%     b_cons=cell2mat(b_cons_cell);%（求解方程）状态量不等式约束的取值

    %二次规划A矩阵
    A_cons_cell={A_I zeros(Nu*Nc,1);-A_I zeros(Nu*Nc,1);};
    A_cons=cell2mat(A_cons_cell);%（求解方程）状态量不等式约束增益矩阵，转换为绝对值的取值范围

    %二次规划的b矩阵
    b_cons_cell={Umax-Ut;-Umin+Ut;};
    b_cons=cell2mat(b_cons_cell);%（求解方程）状态量不等式约束的取值
    
    %控制增量约束
    M=10; 
    delta_umin=-0.248;%前轮偏角变化量的下约束
    delta_umax=0.248;%前轮偏角变化量的上约束
    delta_Umin=kron(ones(Nc,1),delta_umin);
    delta_Umax=kron(ones(Nc,1),delta_umax);
    lb=[delta_Umin;0];%（求解方程）状态量下界，包含控制时域内控制增量和松弛因子
    ub=[delta_Umax;M];%（求解方程）状态量上界，包含控制时域内控制增量和松弛因子
    
    %% 开始求解过程
    %% 开始求解过程
   options = optimset('Display','off', ...
        'TolFun', 1e-8, ...
        'MaxIter', 2000, ...
        'Algorithm', 'interior-point-convex', ...
        'FinDiffType', 'forward', ...
        'RelLineSrchBnd', [], ...
        'RelLineSrchBndDuration', 1, ...
        'TolConSQP', 1e-8);
    warning off all  % close the warnings during computation
%     x_start=zeros(Nc+1,1);%加入一个起始点
    [X,fval,exitflag]=quadprog(H,f,A_cons,b_cons,[],[],lb,ub,[],options);
    fprintf('exitflag=%d\n',exitflag);
    fprintf('H=%4.2f\n',H(1,1));
    fprintf('f=%4.2f\n',f(1,1));
    %% 计算输出
    if isempty(X) 
    U(1)=0;
    else
    fprintf("X(1)=%4.2f\n",X(1));
    u_piao=X(1);%得到控制增量
    U(1)=kesi(7,1)+u_piao;%当前时刻的控制量为上一刻时刻控制+控制增量
    sys= U;
    toc
    end
