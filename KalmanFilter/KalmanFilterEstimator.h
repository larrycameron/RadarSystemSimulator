#ifndef KALMANFILTERESTIMATOR_H
#define KALMANFILTERESTIMATOR_H

#include <cstddef>
#include <random>
#include <string>

class KalmanFilterEstimator
{
private:
    static const std::size_t Sample_Size = 100;
    static const std::size_t States = 9;
    static const std::size_t Positions = 3;
   
    double DeltaTime = 0.1;
   
    // PRF Variables
    double PRF_A = 1000.0;
    double PRF_B = 1500.0;
    double Radar_Wavelength = 0.03;
   
    // Function parameter storage variables
    double Wrapped_Vel_A{};
    double Wrapped_Vel_B{};
               
    double MeasurementMatrix_H[Positions][States]{};
    double Raw_Sensor_Measurement_Vector[Positions]{};
    double Adaptive_Noise_Matrix[Positions][Positions]{};
   
    // Raw radar input history (2D: 100 * 3 measurements)
    double Position_Radar_Matrix[Sample_Size][Positions]{};
   
    // Transformed grid history (2D: 100 steps * 3 coordinates)
    // FIXED: Removed the double bracket typo [[3] and replaced with proper [Positions]
    double Position_Grid_Matrix[Sample_Size][Positions]{};
   
    // Kalman filter matrices 9 X 9
    double State_Transition_Matrix[States][States]{};
    double P_Covariance_Matrix[States][States]{};
   
    // Current State Vectors (1D: 9 Elements)
    double State_Vector[States]{};
    double predicted_state[States]{};
   
     std::mt19937 random_engine{12345}; // Seeded with a fixed number for repeatable tests

    // Radar Sensor Error Margin (Standard Deviations)
    double Noise_Sigma_Range = 15.0;     // Range measurements bounce by +/- 15 meters
    double Noise_Sigma_Azimuth = 0.2;   // Azimuth bounces by +/- 0.2 degrees
    double Noise_Sigma_Elevation = 0.2; // Elevation bounces by +/- 0.2 degrees
   
    double Measurement_Noise_Matrix_R[Positions][Positions]{};
    double Innovation_Covariance_Matrix_S[Positions][Positions]{};
    double Kalman_Gain_Matrix_K[States][Positions]{};

    double Process_Noise_Matrix_Q[States][States]{};

public:
    KalmanFilterEstimator();
    void TransformAndLogCoordinate(std::size_t sample, double range, double azimuth, double elevation);
    double ResolveVelocityAmbiguity(double wrapped_vel_a, double wrapped_vel_b);
    void KalmanFilterPredictor();
    void RunTrackingSimulation();
    void GenerateNoisyMeasurement(double true_range, double true_azimuth, double true_elevation, double& noisy_range, double& noisy_azimuth, double& noisy_elevation);
    bool IsWithinValidationGate(double meas_x, double meas_y, double meas_z);
    void ExecuteRadarTrackingLoop();
    void CalculateInnovationCovariance();
    void CalculateKalmanGain();
    void PredictCovariance();
    void UpdateStateWithMeasurement( double meas_x, double meas_y,  double meas_z);
    void UpdateCovariance();
    void PrintState() const;
    void PrintKalmanGainData() const;
    void ProcessMeasurement( double measured_x, double measured_y, double measured_z);

    double GetEstimatedX() const;
    double GetEstimatedY() const;
    double GetEstimatedZ() const;

    double GetEstimatedVelocityX() const;
    double GetEstimatedVelocityY() const;
    double GetEstimatedVelocityZ() const;

    void SetInitialState(double x, double y, double z, double vx, double vy, double vz, double ax, double ay, double az);
    
}; 

#endif