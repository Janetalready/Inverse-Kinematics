function [iter, iter_time, dis_error] = jacobian_update(DoF, target, a, prec)
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
J = jacobian(T(1:2,4), theta);
%update
converge_begin = cputime;
figure
while b==0
    iter_begin = cputime;
    delta_theta = reshape(delta_theta, [1, DoF]);
    delta_theta = round(double(delta_theta),prec);
    theta_sub = theta_sub + 0.01*radtodeg(delta_theta);
    disp(theta_sub)
    %step1: forward kinematicsa
    
    T_sub = subs(T, theta, theta_sub);
    disp('end effector pos:');disp(T_sub);

    joints_x_sub = subs(joints_x, theta, theta_sub);
    joints_y_sub = subs(joints_y, theta, theta_sub);
    plot([0 joints_x_sub],[0 joints_y_sub],'-o','LineWidth',4);
    axis([-31,31,-31,31]);
    grid on;

    %calculate jacobian
    J_sub = subs(J, theta, theta_sub);
    disp('J:'); disp(J_sub);
    
    % update delta_theta
    delta_p = target - T_sub(1:2,4)';
    delta_theta = pinv(J_sub)*delta_p';
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
    if dis_error <= 0.2
        b=1;
        converge_end = cputime;
        conver_time = converge_end - converge_begin;
%         figure
%         plot(iters, dis_errors, '-o','LineWidth',4);
    end
    iter = iter+1;
    pause(0.01);
end

end