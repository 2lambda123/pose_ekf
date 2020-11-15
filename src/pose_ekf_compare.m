% visualize.m the output of the trajectory_example file.
% this file runs with Matlab or octave
close all;
data=load('./build/sensor_simulator.dat');
data_ekf = load('./build/pose_ekf_result.dat');


time = data(:, 1);
pos = data(:, 2:4);
vel = data(:, 5:7);
quat = data(:, 8:11);
acc_bias = data(:, 12:14);
gyro_bias = data(:, 15:17);

time_ekf      = data_ekf(:, 1);
pos_ekf       = data_ekf(:, 2:4);
vel_ekf       = data_ekf(:, 5:7);
quat_ekf      = data_ekf(:, 8:11);
acc_bias_ekf  = data_ekf(:, 12:14);
gyro_bias_ekf = data_ekf(:, 15:17);
delta_pos_ekf = data_ekf(:, 18:20);
delta_vel_ekf = data_ekf(:, 21:23);



fig2 = figure(2);
for i = 1 : 3
    subplot(3,1,i);
    plot(time, pos(:, i));
    hold on;
    plot(time_ekf, pos_ekf(:, i));
    hold off;
    legend('pos', 'pos ekf')
    xlabel('time [s]');
    ylabel('position[m]');
    title("position")
end
saveas(fig2, 'ekf_pos', 'png')

fig3 = figure(3);
for i = 1 : 3
    subplot(3,1,i);
    plot(time, vel(:, i));
    hold on;
    plot(time_ekf, vel_ekf(:, i));
    hold off;
    legend('vel', 'vel ekf')
    xlabel('time [s]');
    ylabel('velocity[m/s]');
    title("velocity")
end
saveas(fig3, 'ekf_vel', 'png')

fig4 = figure(4);
for i = 1 : 4
    subplot(4,1,i);
    plot(time, quat(:, i));
    hold on;
    plot(time_ekf, quat_ekf(:, i));
    hold off;
    legend('pos', 'quat ekf')
    xlabel('time [s]');
    ylabel('quat');
    title("quat")
end
saveas(fig4, 'ekf_quat','png')

fig5 = figure(5);
for i = 1 : 3
    subplot(3,1,i);
    plot(time, acc_bias(:, i));
    hold on;
    plot(time_ekf, acc_bias_ekf(:, i));
    hold off;
    legend('acc bias', 'acc bias ekf')
    xlabel('time [s]');
    ylabel('acc bias[m/s^2]');
    title("acc bias")
end
saveas(fig5, 'ekf_acc_bias','png')

fig6 = figure(6);
for i = 1 : 3
    subplot(3,1,i);
    plot(time, gyro_bias(:, i));
    hold on;
    plot(time_ekf, gyro_bias_ekf(:, i));
    hold off;
    legend('gyro bias', 'gyro bias ekf')
    xlabel('time [s]');
    ylabel('gyro bias[m/s^2]');
    title("gyro bias")
end
saveas(fig6, 'ekf_gyro_bias','png')

fig7 = figure(7);
for i = 1 : 3
    subplot(3,1,i);
    plot(time_ekf, delta_pos_ekf(:, i));
    hold on;
    plot(time_ekf, delta_vel_ekf(:, i));
    hold off;
    legend('delta pos ekf', 'delta vel ekf')
    xlabel('time [s]');
    ylabel('delta');
    title("delta")
end
saveas(fig7, 'ekf_delta','png')

