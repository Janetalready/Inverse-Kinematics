function [iter, iter_time, dis_error] = ccd_update(DoF, target, a, prec)
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
%update
converge_begin = cputime;
figure
while b==0
    iter_begin = cputime;
    joints_x_sub = subs(joints_x, theta, theta_sub);
    joints_y_sub = subs(joints_y, theta, theta_sub);
    
    for i = 1:DoF
        joints_x_sub = subs(joints_x, theta, theta_sub);
        joints_y_sub = subs(joints_y, theta, theta_sub);
        plot([0 joints_x_sub],[0 joints_y_sub],'-o','LineWidth',4);
        axis([-31,31,-31,31]);
        grid on;
        end_effector = [joints_x_sub(DoF) joints_y_sub(DoF)];
        joint_id = DoF-i;
        if joint_id == 0
            joint_x = 0;
            joint_y = 0;
        else
            joint_x = joints_x_sub(joint_id);
            joint_y = joints_y_sub(joint_id);
        end        
        u = end_effector - [joint_x joint_y];
        u_hat = u/norm(u);
        v = target - [joint_x joint_y];
        v_hat = v/norm(v);
        u_hat = [u_hat 0];
        v_hat = [v_hat 0];
        axis_ = cross(u_hat, v_hat);
        angle = atan2(norm(axis_),dot(u_hat,v_hat));
        angle = radtodeg(round(double(angle),prec));
        disp(angle)
        theta_sub(joint_id+1) = theta_sub(joint_id+1) + angle;
    end    
    delta_p = target - end_effector;
    dis_error = norm(delta_p);
    dis_errors = [dis_errors dis_error];
    iters = [iters iter];
    disp('dis_error');
    disp(round(double(dis_error),3));
    text(joints_x_sub(DoF),joints_y_sub(DoF),['  (', num2str(round(double(joints_x_sub(DoF)),3),3), ...
        ', ', num2str(round(double(joints_y_sub(DoF)),3),3), ')']);
    text(-25,-17,'Orinerror:','Color','red','FontSize',12)
    text(-25,-20,num2str(round(double(dis_error)),3),'Color','red','FontSize',12)
    iter_end = cputime;
    iter_time = iter_end - iter_begin;
    if dis_error < 1 || iter>120
        b=1;
        converge_end = cputime;
        time = converge_end - converge_begin;
%         figure
%         plot(iters, dis_errors, '-o','LineWidth',4);
    end
    iter = iter+1;
    pause(0.01);
    
end