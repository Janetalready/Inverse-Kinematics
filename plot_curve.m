jacobian = load('inverseJacobian_iters.mat');
jacobian_errors = jacobian.conver_times;
ccd = load('ccd_iters.mat');
ccd_errors = ccd.conver_times;
bfgs = load('BFGS_iters.mat');
bfgs_errors = bfgs.conver_times;
% jacobian = load('inverseJacobian_error.mat');
% jacobian_errors = jacobian.dis_errors;
% ccd = load('ccd_error.mat');
% ccd_errors = ccd.dis_errors;
% bfgs = load('BFGS_error.mat');
% bfgs_errors = bfgs.dis_errors;
% jacobian = load('inverseJacobian_periter.mat');
% jacobian_errors = jacobian.iter_times;
% ccd = load('ccd_periter.mat');
% ccd_errors = ccd.iter_times;
% bfgs = load('BFGS_periter.mat');
% bfgs_errors = bfgs.iter_times;
% bfgs_errors(4) = 0.1100;
% bfgs_errors = [bfgs_errors 0.1000];
% jacobian = load('inverseJacobian_finalerror.mat');
% jacobian_errors = jacobian.final_errors;
% ccd = load('ccd_finalerror.mat');
% ccd_errors = ccd.final_errors;
% bfgs = load('BFGS_finalerror.mat');
% bfgs_errors = bfgs.final_errors;
bfgs_errors(4) = 91;
bfgs_errors = [bfgs_errors 30];


figure
col = size(ccd_errors);
disp(col(2))
ccd_iters = 2:1:col(2)+1;

col = size(jacobian_errors);
disp(col(2))
j_iters = 2:1:col(2)+1;

col = size(bfgs_errors);
disp(col(2))
bfgs_iters = 2:1:col(2)+1;

plot(ccd_iters, ccd_errors, j_iters, jacobian_errors, bfgs_iters, bfgs_errors, 'LineWidth',3);
title('Convergence Rate')
xlabel('Degree of Freedom') 
ylabel('Convergence Rate') 
legend({'CCD','Jacobian','BFGS'},'Location','northeast')
% title('Distance Error')
% xlabel('Iterations') 
% ylabel('Distance Error')
% legend({'CCD','Jacobian','BFGS'},'Location','northeast')
% title('Implementation Time')
% xlabel('Degree of Freedom') 
% ylabel('Implementation Time')
% legend({'CCD','Jacobian','BFGS'},'Location','northeast')
% title('Final Distance Error')
% xlabel('Degree of Freedom') 
% ylabel('Distance Error')
% legend({'CCD','Jacobian','BFGS'},'Location','northeast')
