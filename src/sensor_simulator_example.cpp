#include <iostream>
#include <Eigen/Eigen>
#include "sensor_simulator.h"
#include "noise_simulator.h"
#include "sensor_config.h"

using namespace std;
using namespace Eigen;

int main(int argc, char **argv)
{
    double freq = IMU_FREQ;
    double total_time = 500;
    Vector3d pos, vel, linear_acc;
    Vector3d omega;
    Quaterniond q;

    sensor_simulator simulator;
    map<int, Vector3d> landmarks;
    map<int, Vector2d> features;
    map<int, Vector3d> feature3ds;//feature pos in cam frame, for stereo camera

    //Vector3d gyro_bias(0.02, 0.01, -0.02);
    //Vector3d acc_bias(-0.01, 0.01, 0.02);
    Vector3d acc_bias(0, 0, 0);
    Vector3d gyro_bias(0, 0, 0);
    Vector3d pos_noise, vel_noise, acc_noise, gyro_noise, mag_noise;

    noise_simulator acc_noise_simulator(ACC_NOISE_DENSITY, ACC_RANDOM_WALK, ACC_CONST_BIAS_MAX, freq);
    noise_simulator gyro_noise_simulator(GYRO_NOISE_DENSITY, GYRO_RANDOM_WALK, GYRO_CONST_BIAS_MAX, freq);
    noise_simulator pos_noise_simulator(GPS_POS_NOISE_DENSITY, GPS_POS_RANDOM_WALK, 0, freq / IMU_CNT_PER_GPS);
    noise_simulator vel_noise_simulator(GPS_VEL_NOISE_DENSITY, GPS_VEL_RANDOM_WALK, 0, freq) / IMU_CNT_PER_GPS;
    noise_simulator mag_noise_simulator(MAG_NOISE_DENSITY, MAG_RANDOM_WALK, 0, freq / IMU_CNT_PER_MAG);

    // use the trajectory
    double dt = 1.0 / freq;
    FILE *pf = fopen("./sensor_simulator.dat", "w+");
    Vector3d gravity(0, 0, -9.8);
    Vector3d magnetic(1, 0, 0.2); //[1, 0, x]
    FILE *pf_m = fopen("./sensor_measurement.dat", "w+");
    //fprintf(pf,"t,posx,posy,posz,rotx,roty,rotz,qw, qx, qy, qz, velx,vely,velz,omegax,omegay,omegaz,accx,accy,accz,\n");
    //get current state from path, pose, velocity, acceleration, including translation and rotation
    for (int j = 0; j < freq * total_time; j++)
    {
        double t = j / freq;
        simulator.generate_state(t);
        simulator.generate_measurement(t);

        if (j % (IMU_FREQ / IMAGE_FREQ)  == 0)
        {
            simulator.generate_landmark(landmarks, features, feature3ds);
            simulator.visualize_feature(features);
        }


        vel = simulator.get_vel();
        pos = simulator.get_pos();
        q = simulator.get_rot();
        omega = simulator.get_angular_rate();
        linear_acc = simulator.get_linear_acc();

        pos_noise_simulator.generate_noise();
        vel_noise_simulator.generate_noise();
        acc_noise_simulator.generate_noise();
        gyro_noise_simulator.generate_noise();
        mag_noise_simulator.generate_noise();

        pos_noise = pos_noise_simulator.get_noise();
        vel_noise = vel_noise_simulator.get_noise();
        acc_noise = acc_noise_simulator.get_noise();
        gyro_noise = gyro_noise_simulator.get_noise();
        mag_noise = mag_noise_simulator.get_noise();

        acc_bias = acc_noise_simulator.get_bias();
        gyro_bias = gyro_noise_simulator.get_bias();

        fprintf(pf, "%lf,", t);
        for (int i = 0; i < 3; i++)
        {
            fprintf(pf, "%lf,", pos(i));
        }
        for (int i = 0; i < 3; i++)
        {
            fprintf(pf, "%lf,", vel(i));
        }
        double quat[4];
        quat[0] = q.w();
        quat[1] = q.x();
        quat[2] = q.y();
        quat[3] = q.z();

        for (int i = 0; i < 4; i++)
        {
            fprintf(pf, "%lf,", quat[i]);
        }
        for (int i = 0; i < 3; i++)
        {
            fprintf(pf, "%lf,", acc_bias(i));
        }
        for (int i = 0; i < 3; i++)
        {
            fprintf(pf, "%lf,", gyro_bias(i));
        }
        fprintf(pf, "\n");

        //record measurements
        {
            //add noise with noise simulator
            {
                pos += pos_noise;
                vel += vel_noise;
            }
            fprintf(pf_m, "%lf,", t);
            for (int i = 0; i < 3; i++)
            {
                fprintf(pf_m, "%lf,", pos(i));
            }
            for (int i = 0; i < 3; i++)
            {
                fprintf(pf_m, "%lf,", vel(i));
            }
            //imu data, acc and omega
            Matrix3d R = q.toRotationMatrix().transpose(); //ned to body
            Vector3d acc_raw = R * (gravity + linear_acc);
            Vector3d omega_raw = omega;
            Vector3d mag = R * magnetic;
            //add noise with noise simulator
            {
                acc_raw += acc_noise;
                omega_raw += gyro_noise;
                mag += mag_noise;

                acc_bias = acc_noise;
                gyro_bias = gyro_noise;
            }
            for (int i = 0; i < 3; i++)
            {
                fprintf(pf_m, "%lf,", omega_raw(i));
            }
            for (int i = 0; i < 3; i++)
            {
                fprintf(pf_m, "%lf,", acc_raw(i));
            }
            //compass data

            for (int i = 0; i < 3; i++)
            {
                fprintf(pf_m, "%lf,", mag(i));
            }
            fprintf(pf_m, "\n");
        }
    }
    fclose(pf);
    fclose(pf_m);
    return 0;
}