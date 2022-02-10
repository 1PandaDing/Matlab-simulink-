function    [a, b] = func_Model_linearization_Jacobian(kesi, Sf, Sr, MPCParameters, VehiclePara)
%***************************************************************%
% ���ݼ򻯶���ѧģ��(����С�Ƕȼ�����)�����ſ˱Ⱦ���
% ��������ſ˱Ⱦ������복�����в����������
    %----------������������ -----------%
    syms y_dot x_dot phi phi_dot Y X;%����״̬��
    syms delta_f  %ǰ��ƫ��,������
    
    Ts = MPCParameters.Ts;
    Lf  = VehiclePara.Lf;
    Lr  = VehiclePara.Lr;
    m  = VehiclePara.m;
    Iz = VehiclePara.Iz;
    Ccf = VehiclePara.Ccf;
    Ccr = VehiclePara.Ccr;
    Clf = VehiclePara.Clf;
    Clr = VehiclePara.Clr;

    %----��������ѧģ��-------------%
%     dy_dot = -x_dot*phi_dot + 2*(Ccf*((y_dot+Lf*phi_dot)/x_dot - delta_f) + Ccr*(y_dot - Lr*phi_dot)/x_dot)/m;
%     dx_dot = y_dot*phi_dot + 2*(Clf*Sf + Clr*Sr + Ccf*((y_dot + phi_dot*Lf)/x_dot - delta_f)*delta_f)/m;
%     dphi_dot = (2*Lf*Ccf*((y_dot+Lf*phi_dot)/x_dot - delta_f) - 2*Lr*Ccr*(y_dot - Lr*phi_dot)/x_dot)/Iz;
%     Y_dot = x_dot*sin(phi) + y_dot*cos(phi);
%     X_dot = x_dot*cos(phi) - y_dot*sin(phi);
    
    % ��������ѧģ��
    dy_dot=-x_dot*phi_dot+2*(Ccf*(delta_f-(y_dot+Lf*phi_dot)/x_dot)+Ccr*(Lr*phi_dot-y_dot)/x_dot)/m;
    dx_dot=y_dot*phi_dot+2*(Clf*Sf+Clr*Sr-Ccf*delta_f*(delta_f-(y_dot+phi_dot*Lf)/x_dot))/m;
    %dphi_dot=dphi_dot;
    dphi_dot=(2*Lf*Ccf*(delta_f-(y_dot+Lf*phi_dot)/x_dot)-2*Lr*Ccr*(Lr*phi_dot-y_dot)/x_dot)/Iz;
    Y_dot=x_dot*sin(phi)+y_dot*cos(phi);
    X_dot=x_dot*cos(phi)-y_dot*sin(phi);

    %----�ſ˱Ⱦ������-------------%
    Dynamics_func = [dy_dot; dx_dot; phi_dot; dphi_dot; Y_dot; X_dot];%����ѧģ��
    state_vector = [y_dot,x_dot,phi,phi_dot,Y,X];%ϵͳ״̬��
    control_input = delta_f;
    A_t = jacobian(Dynamics_func, state_vector);  %����A(t)-����
    B_t = jacobian(Dynamics_func, control_input); %����B(t)-����

    %----����������ת��Ϊ��ɢ����-------------%
    % ����Forward Euler Method�����㷨  A = Iz+Ts*A(t),B = Ts*B(t)
    I_6 = eye(6);
    Ad_temp = I_6 + Ts * A_t;
    Bd_temp = Ts * B_t;
    
    %----��ȡ����״̬����-------------%
    y_dot   = kesi(1);
    x_dot   = kesi(2);
    phi     = kesi(3);
    phi_dot = kesi(4);
    Y       = kesi(5);
    X       = kesi(6);
    delta_f = kesi(7);
    
    a = eval(Ad_temp);
    b = eval(Bd_temp);
end % end of func.