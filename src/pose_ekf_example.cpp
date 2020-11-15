#include "pose_ekf.h"
#include "sensor_config.h"
#include <iostream>

int main(int argc, char **argv)
{

    FILE *pf = fopen("../build/sensor_measurement.dat", "r");
    FILE *pf_out = fopen("pose_ekf_result.dat", "w+");

    if (pf == NULL || pf_out == NULL)
    {
        cout << "failed to open file" << endl;
    }

    pose_ekf pose;
    double imu_freq = IMU_FREQ;
    pose.set_imu_freq(imu_freq);

    Vector3d acc_raw, mag, omega_raw;
    Vector3d pos, vel, vel_prev;
    double timestamp;
    double dt = 1.0 / imu_freq;
    int data_cnt = 0;

    while (!feof(pf))
    {
        double data[16];
        for (int j = 0; j < 16; j++)
        {
            fscanf(pf, "%lf,", &data[j]);
        }
        fscanf(pf, "\n");

        timestamp = data[0];
        for (int k = 0; k < 3; k++)
        {
            pos(k) = data[1 + k];
            vel(k) = data[4 + k];
            omega_raw(k) = data[7 + k];
            acc_raw(k) = data[10 + k];
            mag(k) = data[13 + k];
        }
        cout << "time: " << timestamp << endl;
        cout << "acc: " << acc_raw.transpose() << endl;
        // cout << "omega: " << omega.transpose() << endl;
        cout << "mag: " << mag.transpose() << endl;
        //cout << "pos: " << pos.transpose() << endl;
        //cout << "vel: " << vel.transpose() << endl;
        Vector3d linear_acc = Vector3d::Zero();
        if (data_cnt > 0)
        {
            linear_acc = (vel - vel_prev) / dt;
            cout << "linear_acc: " << linear_acc.transpose() << endl;
        }
        vel_prev = vel;
        data_cnt++;

        if (pose.is_atti_init_done == false)
        {
            pose.atti_init(acc_raw, mag);
            continue;
        }
        if (pose.is_init_done == false)
        {
            pose.pose_init(pos, vel);
            fprintf(pf_out, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,\n",
                    timestamp, pose.pos(0), pose.pos(1), pose.pos(2),
                    pose.vel(0), pose.vel(1), pose.vel(2),
                    pose.q.w(), pose.q.x(), pose.q.y(), pose.q.z(),
                    pose.acc_bias(0), pose.acc_bias(1), pose.acc_bias(2),
                    pose.gyro_bias(0), pose.gyro_bias(1), pose.gyro_bias(2),
                    pose.delta_pos(0), pose.delta_pos(1), pose.delta_pos(2),
                    pose.delta_vel(0), pose.delta_vel(1), pose.delta_vel(2));

            continue;
        }

        pose.predict(omega_raw, acc_raw,  dt);
        //pose.update_acc(acc_raw);
        //pose.update_linear_acc(linear_acc, acc_raw);
        pose.update_magnetic(mag);
        //pose.update_gps(pos, vel);
        pose.update_gps_vel(vel);
        pose.update_gps_pos(pos);

        fprintf(pf_out, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,\n",
                timestamp, pose.pos(0), pose.pos(1), pose.pos(2),
                pose.vel(0), pose.vel(1), pose.vel(2),
                pose.q.w(), pose.q.x(), pose.q.y(), pose.q.z(),
                pose.acc_bias(0), pose.acc_bias(1), pose.acc_bias(2),
                pose.gyro_bias(0), pose.gyro_bias(1), pose.gyro_bias(2),
                pose.delta_pos(0), pose.delta_pos(1), pose.delta_pos(2),
                pose.delta_vel(0), pose.delta_vel(1), pose.delta_vel(2));
        pose.print_state();
    }
    fclose(pf);
    fclose(pf_out);

    return 0;
}