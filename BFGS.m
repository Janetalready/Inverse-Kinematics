close all; clear;
%Input parameter
origin = [0,0];
DoFs = [2, 3, 4, 5, 6];
a = {[10 10],[10 10 5], [10 10 5 4], [10 10 5 4 3],[10 10 5 4 3 2]};
steps = [4,4,4,8,8];
target = {[5 10]};
precs = [10 10 10 8 8];
conver_times = [];
iter_times = [];
iters = [];
final_errors = [];
for i =1:length(DoFs)
    [conver_time, iter_time, dis_error] = BFGS_update(DoFs(i), target{1}, a{i}, precs(i), steps(i));
    conver_times = [conver_times conver_time];
    iter_times = [iter_times iter_time];
    iters = [iters i];
    final_errors = [final_errors, dis_error];
end

plot(iters, conver_times,'-o','LineWidth',4);
figure
plot(iters, iter_times,'-o','LineWidth',4);