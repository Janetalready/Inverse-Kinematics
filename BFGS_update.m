function [iter, iter_time, dis_error] = BFGS_update(DoF, target, a, prec, step)
%initialize parameter
theta = sym('theta',[1 DoF]);
d = zeros(1, DoF);
alpha = zeros(1, DoF);
delta_theta = zeros(1, DoF);
DHmatrices = cell(DoF,1);
T = eye(4);
positions = cell(DoF,1);
joints_x = sym('joints_x',[1 DoF]);
joints_y = sym('joints_y',[1 DoF]);
theta_sub = zeros(1, DoF);
b = 0;
dis_errors = [];
iters = [];
iter = 0;
%DHmatrix
for i=1:DoF
        DHmatrices{i} = DHmatrix(theta(i),d(i),a(i),alpha(i));
        T = T*DHmatrix(theta(i),d(i),a(i),alpha(i));
        positions{i} = transpose(T(1:2,4));
        joints_x(i) = positions{i}(1);
        joints_y(i) = positions{i}(2);
end
%Jacobian
Js = cell(2,1);
J = jacobian(T(1:2,4), theta);
%end_effectors
end_effectors = cell(2,1);
%q
qs = cell(2,1);
%flag
flag_e = 1;
flag_j = 1;
flag_q = 1;
%intialize 
joints_x_sub = subs(joints_x, theta, theta_sub);
joints_y_sub = subs(joints_y, theta, theta_sub);
end_effectors{flag_e,1} = [joints_x_sub(DoF) joints_y_sub(DoF)];
end_effectors{flag_e+1,1} = [joints_x_sub(DoF) joints_y_sub(DoF)];
flag_e = flag_e + 1;

J_sub = subs(J, theta, theta_sub);
Js{flag_j,1} = J_sub;
Js{flag_j+1,1} = J_sub;
flag_j = flag_j + 1;

qs{flag_q,1} = theta_sub;
qs{flag_q+1,1} = theta_sub;
flag_q = flag_q + 1;
%update
converge_begin = cputime;
figure
while b==0
   iter_begin = cputime;
   if iter == 0
        H = eye(DoF);
    else
        s = qs{2,1}-qs{1,1};
        disp([qs{2,1},qs{1,1}]);
        s = reshape(s, [DoF, 1]);
        dev2 = round(double(end_effectors{2,1}),prec)-target;
        dev2 = reshape(dev2, [2, 1]);
        dev1 = round(double(end_effectors{1,1}),prec)-target;
        dev1 = reshape(dev1, [2, 1]);
        g = round(double(Js{2,1}),10)'*dev2 - round(double(Js{1,1}),10)'*dev1;
        if(g'*s<=0)
            t = 1;
        else
            t = g'*s;
        end
        H = H - H*(s*s')*H/(s'*H*s)+g*g'/t;
    end
    % update delta_theta  
    delta_theta = -H*(Js{2,1}'*(end_effectors{2,1}-target)');
    delta_theta = reshape(delta_theta, [1, DoF]);
    delta_theta = round(double(delta_theta),prec);
    theta_sub = theta_sub + radtodeg(step*delta_theta);
    
    joints_x_sub = subs(joints_x, theta, theta_sub);
    joints_y_sub = subs(joints_y, theta, theta_sub);
    end_effectors{flag_e-1,1} = end_effectors{flag_e,1};
    end_effectors{flag_e,1} = [joints_x_sub(DoF) joints_y_sub(DoF)];
    
    qs{flag_q-1,1} = qs{flag_q,1};
    qs{flag_q,1} = theta_sub;
    
    J_sub = subs(J, theta, theta_sub);
    Js{flag_j-1,1} = Js{flag_j,1};
    Js{flag_j,1} = J_sub;
    
%     disp(theta_sub)
    
    %step1: forward kinematicsa
    
    T_sub = subs(T, theta, theta_sub);
%     disp('end effector pos:');disp(T_sub);
    plot([0 joints_x_sub],[0 joints_y_sub],'-o','LineWidth',4);
    axis([-31,31,-31,31]);
    grid on;
    
    % calculate_error
    delta_p = target - T_sub(1:2,4)';
    dis_error = norm(delta_p);
    dis_errors = [dis_errors dis_error];
    iters = [iters iter];
    disp('dis_error');
    disp(round(double(dis_error),3));
    text(joints_x_sub(DoF),joints_y_sub(DoF),['  (', num2str(round(double(joints_x_sub(DoF)),3)), ...
        ', ', num2str(round(double(joints_y_sub(DoF)),3)), ')']);
    text(-25,-17,'Orinerror:','Color','red','FontSize',12)
    text(-25,-20,num2str(round(double(dis_error),3)),'Color','red','FontSize',12)
    iter_end = cputime;
    iter_time = iter_end - iter_begin;
    if (dis_error <= 0.2) || iter>120
        b=1;
        converge_end = cputime;
        time = converge_end - converge_begin;
%         figure
%         plot(iters, dis_errors, '-o','LineWidth',4);
    end
    iter = iter+1;
    pause(0.01);
end

end