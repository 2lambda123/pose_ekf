#include "noise_simulator.h"
#include "sensor_config.h"
#include <iostream>
#include <Eigen/Eigen>


using namespace std;
using namespace Eigen;

int main(int argc, char** argv)
{
    noise_simulator gyro_nosie_simulator(GYRO_NOISE_DENSITY, GYRO_RANDOM_WALK, GYRO_CONST_BIAS_MAX, IMU_FREQ);
    noise_simulator acc_noise_simulator(ACC_NOISE_DENSITY, ACC_RANDOM_WALK, ACC_CONST_BIAS_MAX, IMU_FREQ);

    double freq = IMU_FREQ;
    double total_time = 3600 * 5;//5 hour
    Vector3d gyro_noise;
    Vector3d acc_noise;
    FILE *pf = fopen("./sensor_noise.dat", "w+");
    for (int i = 0; i < freq * total_time; i++)
    {
        double t = i / freq;
        gyro_nosie_simulator.generate_noise();
        acc_noise_simulator.generate_noise();

        gyro_noise = gyro_nosie_simulator.get_noise();
        acc_noise = acc_noise_simulator.get_noise();
        fprintf(pf, "%lf,", t);
        for (int j = 0; j < 3; j++)
        {
            fprintf(pf, "%lf,", acc_noise(j));
        }
        for (int j = 0; j < 3; j++)
        {
            fprintf(pf, "%lf,", gyro_noise(j));
        }
        fprintf(pf, "\n");
        fflush(pf);
    }
    fclose(pf);
    return 0;
}
