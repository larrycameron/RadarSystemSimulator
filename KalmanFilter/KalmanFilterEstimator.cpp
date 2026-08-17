#include "KalmanFilterEstimator.h"

#include <iostream>
#include <cmath>
#include <iomanip>


    KalmanFilterEstimator::KalmanFilterEstimator()
    {
        for (std::size_t i = 0; i < States; ++i)
        {
            State_Transition_Matrix[i][i] = 1.0;    
        }
           
        // Velocity Multipliers to Positions (d = v * dt)
        State_Transition_Matrix[0][3] = DeltaTime;
        State_Transition_Matrix[1][4] = DeltaTime;
        State_Transition_Matrix[2][5] = DeltaTime;
   
        // Acceleration multipliers to Positions (d = 0.5 * a * dt^2)
        double half_dt_sp = 0.5 * std::pow(DeltaTime, 2);
        State_Transition_Matrix[0][6] = half_dt_sp;
        State_Transition_Matrix[1][7] = half_dt_sp;
        State_Transition_Matrix[2][8] = half_dt_sp;
   
        // Apply Acceleration Multipliers to Velocities (v = a * dt)
        State_Transition_Matrix[3][6] = DeltaTime;
        State_Transition_Matrix[4][7] = DeltaTime;
        State_Transition_Matrix[5][8] = DeltaTime;
   
        // Initializes Measurement Matrix H to map states 0,1,2 to positions
        MeasurementMatrix_H[0][0] = 1.0;
        MeasurementMatrix_H[1][1] = 1.0;
        MeasurementMatrix_H[2][2] = 1.0;
       
         // Measurement Noise Matrix R
        Measurement_Noise_Matrix_R[0][0] = 25.0;
        Measurement_Noise_Matrix_R[1][1] = 25.0;
        Measurement_Noise_Matrix_R[2][2] = 25.0;

        for (std::size_t i = 0; i < States; ++i)
        {
            P_Covariance_Matrix[i][i] = 100.0;
        }
        
        // Initialize process noise matrix Q
        for (std::size_t i = 0; i < States; ++i)
        {
            Process_Noise_Matrix_Q[i][i] = 1.0;
        }

        // Give the aircraft an initial velocity of 10 m/s in X-axis
        State_Vector[3] = 10.0;
       
        // Quiet down warnings for fields declared but not actively used in predictions yet
        (void)MeasurementMatrix_H;
        (void)Raw_Sensor_Measurement_Vector;
        (void)Adaptive_Noise_Matrix;
        (void)Position_Radar_Matrix;
        (void)Position_Grid_Matrix;
        (void)P_Covariance_Matrix;
    }
    void KalmanFilterEstimator::TransformAndLogCoordinate(std::size_t sample, double range, double azimuth, double elevation)
{
    if (sample >= Sample_Size)
    {
        return;
    }

    const double PI = 3.14159265358979323846;

    double Azimuth_Radians = azimuth * PI / 180.0;
    double Elevation_Radians = elevation * PI / 180.0;

    // Store raw radar measurements
    Position_Radar_Matrix[sample][0] = range;
    Position_Radar_Matrix[sample][1] = azimuth;
    Position_Radar_Matrix[sample][2] = elevation;

    // Convert spherical radar coordinates to Cartesian coordinates
    double X = range * std::cos(Elevation_Radians) * std::cos(Azimuth_Radians);

    double Y = range * std::cos(Elevation_Radians) * std::sin(Azimuth_Radians);

    double Z = range * std::sin(Elevation_Radians);

    // Store transformed coordinates
    Position_Grid_Matrix[sample][0] = X;
    Position_Grid_Matrix[sample][1] = Y;
    Position_Grid_Matrix[sample][2] = Z;

    std::cout << "Spherical coordinates transformed to Cartesian coordinates: " << Position_Grid_Matrix[sample][0] << " :X " << Position_Grid_Matrix[sample][1] << " :Y " << Position_Grid_Matrix[sample][2] << " :Z" << std::endl;
}    

    double KalmanFilterEstimator::ResolveVelocityAmbiguity(double wrapped_vel_a, double wrapped_vel_b)
    {
        Wrapped_Vel_A = wrapped_vel_a;
        Wrapped_Vel_B = wrapped_vel_b;
                   
        double V_Max_A = (Radar_Wavelength * PRF_A) / 4.0;
        double V_Max_B = (Radar_Wavelength * PRF_B) / 4.0;
       
        for (int m = -10; m <= 10; ++m)
        {
            double Candidate_True_Speed = Wrapped_Vel_A + (2.0 * m * V_Max_A);
            double Expected_Wrap_B = std::fmod(Candidate_True_Speed, 2.0 * V_Max_B);
       
            if (std::abs(Expected_Wrap_B - Wrapped_Vel_B) < 0.1)
            {
                return Candidate_True_Speed;  
            }
        }    
        return Wrapped_Vel_A;    
    }

    void KalmanFilterEstimator::KalmanFilterPredictor()
{
    std::cout << std::left << std::setw(8) << "Step" << std::setw(18) << "True X (m)"<< std::setw(18) << "Noisy Radar X (m)" << std::endl;
    std::cout << std::string(50, '-') << std::endl;

    double true_x = 1000.0; // Starting position
    double constant_azimuth = 45.0;
    double constant_elevation = 30.0;

    for (std::size_t sample = 0; sample < Sample_Size; ++sample) // Run 10 steps to watch the noise
    {  
        // 1. Move the aircraft forward perfectly in the background
        true_x += 10.0 * DeltaTime;

        // Reverse-engineer what the perfect Range would be for this X position
        const double PI = 3.14159265358979323846;
        double true_range = true_x / (std::cos(constant_elevation * PI / 180.0) * std::sin(constant_azimuth * PI / 180.0));

        // 2. Simulate the radar hardware capturing the target with built-in noise
        double radar_range, radar_azimuth, radar_elevation;
        GenerateNoisyMeasurement(true_range, constant_azimuth, constant_elevation,radar_range, radar_azimuth, radar_elevation);

        // 3. Transform the messy radar dots into Cartesian coordinates
        TransformAndLogCoordinate(sample, radar_range, radar_azimuth, radar_elevation);

        // Fetch the newly transformed messy X coordinate out of your grid matrix history
        double noisy_x = Position_Grid_Matrix[sample][0];

        // Output results to watch the values bounce around the true path
        std::cout << std::left << std::setw(8) << (sample + 1)<< std::setw(18) << std::fixed << std::setprecision(2) << true_x << std::setw(18) << noisy_x << "\n";
    }
}
    void KalmanFilterEstimator::RunTrackingSimulation()
{
    // Local tracking simulation variables
    double true_x_position = 0.0;
    double true_x_velocity = 10.0; // Starts at 10 m/s

    for (std::size_t sample = 0; sample < Sample_Size; ++sample)
    {  
        // 1. SIMULATE AN AIRCRAFT ACCELERATION SPIKE
        if (sample == 50)
        {
            true_x_velocity = 1005.0;
            State_Vector[3] = true_x_velocity;
        
        } // Jet suddenly goes supersonic!State_Vector[3] = true_x_velocity;

        // Update the true background position using basic kinematics
        true_x_position += true_x_velocity * DeltaTime;

        // 2. SIMULATE WRAPPED DOPPLER RADAR READINGS (Sensor Side)
        double V_Max_A = (Radar_Wavelength * PRF_A) / 4.0;
        double V_Max_B = (Radar_Wavelength * PRF_B) / 4.0;
       
        // Truncate the massive true speed into wrapped sensor outputs
        double wrapped_sensor_val_A = std::fmod(true_x_velocity, 2.0 * V_Max_A);
        double wrapped_sensor_val_B = std::fmod(true_x_velocity, 2.0 * V_Max_B);

        // 3. RUN AMBIGUITY RESOLUTION SWITCH
        double resolved_velocity = ResolveVelocityAmbiguity(wrapped_sensor_val_A, wrapped_sensor_val_B);

        // 4. RUN YOUR EXISTING MATRICES PREDICTION
        for(std::size_t i = 0; i < States; ++i) predicted_state[i] = 0.0;
        
            for (std::size_t i = 0; i < States; ++i) 
            {
                for(std::size_t j = 0; j < States; ++j) 
                                {
                    predicted_state[i] += State_Transition_Matrix[i][j] * State_Vector[j];  
                }
            }

        // Apply state updates
        for (std::size_t i = 0; i < States; ++i) State_Vector[i] = predicted_state[i];
       
        // Force the resolved unaliased speed directly back into the filter state
        State_Vector[3] = resolved_velocity;

        // Print output to verify if it handles the jump at step 200 safely
        if (sample >= 45 && sample <= 55) 
        {
            std::cout << "Step: " << sample << " | True Vel: " << true_x_velocity << " | Sensor A: " << wrapped_sensor_val_A << " | Resolved: " << State_Vector[3] << "\n";
        }
    }  
}

// Adds random radar measurement noise to a perfect position
    void KalmanFilterEstimator::GenerateNoisyMeasurement(double true_range, double true_azimuth, double true_elevation, double& noisy_range, double& noisy_azimuth, double& noisy_elevation)
    {
        // Set up normal distributions centered at 0 with our specific standard deviations
        std::normal_distribution<double> dist_range(0.0, Noise_Sigma_Range);
        std::normal_distribution<double> dist_azimuth(0.0, Noise_Sigma_Azimuth);
        std::normal_distribution<double> dist_elevation(0.0, Noise_Sigma_Elevation);

        // Inject the random bounce into the true values
        noisy_range = true_range + dist_range(random_engine);
        noisy_azimuth = true_azimuth + dist_azimuth(random_engine);
        noisy_elevation = true_elevation + dist_elevation(random_engine);
    }
  bool KalmanFilterEstimator::IsWithinValidationGate(double meas_x, double meas_y, double meas_z)
    {
        // 1. Calculate the Innovation Vector (Difference between actual and predicted)
        // Maps directly to tracking states: State_Vector[0]=X, [1]=Y, [2]=Z
        double innovation_x = meas_x - State_Vector[0];
        double innovation_y = meas_y - State_Vector[1];
        double innovation_z = meas_z - State_Vector[2];

        // 2. Setup the Innovation Covariance diagonal variances (S Matrix)
        // Combines system uncertainty (P_Covariance_Matrix) with sensor hardware variance
        // Using a baseline variance of 25.0 meters squared for this phase
        double S_x = Innovation_Covariance_Matrix_S[0][0];
        double S_y = Innovation_Covariance_Matrix_S[1][1];
        double S_z = Innovation_Covariance_Matrix_S[2][2];

        // 3. Compute the squared Mahalanobis Distance (algebraic matrix expansion)
        double mahalanobis_distance_sq = (innovation_x * innovation_x / S_x) +  (innovation_y * innovation_y / S_y) + (innovation_z * innovation_z / S_z);

        // 4. Set the Chi-Squared Gate Threshold
        // For a 3D tracking space (X, Y, Z), a threshold of 9.21 establishes
        // a 99% confidence interval bubble.
        double gate_threshold = 11.345;

        // If the distance is less than the threshold, it is a valid target point!
        return (mahalanobis_distance_sq <= gate_threshold);
    }    

void KalmanFilterEstimator::ExecuteRadarTrackingLoop()
{
    std::cout << std::left << std::setw(8) << "Step" << std::setw(15) << "Model Pred X" << std::setw(15) << "Radar Meas X"<< std::setw(12) << "Gate Status" << std::endl;std::cout << std::string(52, '-') << std::endl;

    for (std::size_t sample = 0; sample < Sample_Size; ++sample)
    {   
        for(std::size_t i = 0; i < States; ++i) 
        {
            predicted_state[i] = 0.0;
        }
            for (std::size_t i = 0; i < States; ++i) 
            {
            
                for(std::size_t j = 0; j < States; ++j) 
                
                {
                predicted_state[i] += State_Transition_Matrix[i][j] * State_Vector[j];  
                }
            }
            
            for (std::size_t i = 0; i < States; ++i) 
            {
                State_Vector[i] = predicted_state[i];
            }
            
            PredictCovariance();

            CalculateInnovationCovariance();

            CalculateKalmanGain();

            std::cout<< "Kx: " << std::fixed << std::setprecision(4) << Kalman_Gain_Matrix_K[0][0] << std::endl;

            // 2. Mock incoming transformed sensor coordinates
            double radar_meas_x = State_Vector[0];
            double radar_meas_y = State_Vector[1];
            double radar_meas_z = State_Vector[2];

        // Inject an artificial clutter anomaly at step 5 to test the gate logic
            if (sample == 4) 
            {
                radar_meas_x += 50.0; // Sudden extreme jump (clutter/noise)
            } 
            else 
            {
                radar_meas_x += 2.1;  // Standard noisy deviation
            }

        // 3. Evaluate the point against the validation bubble
        bool passed_gate = IsWithinValidationGate(radar_meas_x, radar_meas_y, radar_meas_z);

        std::string status = passed_gate ? "ACCEPTED" : "REJECTED (Noise)";

        std::cout << std::left << std::setw(8) << (sample + 1)<< std::setw(15) << std::fixed << std::setprecision(2) << State_Vector[0]<< std::setw(15) << radar_meas_x << std::setw(12) << status << "\n";

        // 4. Update track vector only if it passes the security check
        if (passed_gate)
        {
            UpdateStateWithMeasurement(radar_meas_x, radar_meas_y, radar_meas_z);

            UpdateCovariance();
        }
    }
}

    void KalmanFilterEstimator::CalculateInnovationCovariance()
    {
        Innovation_Covariance_Matrix_S[0][0] = P_Covariance_Matrix[0][0] + Measurement_Noise_Matrix_R[0][0];

        Innovation_Covariance_Matrix_S[1][1] = P_Covariance_Matrix[1][1] + Measurement_Noise_Matrix_R[1][1];

        Innovation_Covariance_Matrix_S[2][2] = P_Covariance_Matrix[2][2] + Measurement_Noise_Matrix_R[2][2];
    }

    void KalmanFilterEstimator::CalculateKalmanGain()
    {
    // K = P * H^T * S^-1
    //
    // Since H directly selects X, Y, Z:
    // column 0 of K uses column 0 of P
    // column 1 of K uses column 1 of P
    // column 2 of K uses column 2 of P

        for (std::size_t state = 0; state < States; ++state)
        {
            for (std::size_t measurement = 0; measurement < Positions; ++measurement)
            {
                double S = Innovation_Covariance_Matrix_S [measurement][measurement];

                if (S != 0.0)
                {
                    Kalman_Gain_Matrix_K[state][measurement] = P_Covariance_Matrix[state][measurement]/ S;
                }
            }
        }
}

    void KalmanFilterEstimator::PredictCovariance()
    {
        double Temporary_Matrix[States][States]{};
        double Predicted_P[States][States]{};

        // Step 1: Temporary_Matrix = F * P
    for (std::size_t i = 0; i < States; ++i)
    {
        for (std::size_t j = 0; j < States; ++j)
        {
            for (std::size_t k = 0; k < States; ++k)
            {
                Temporary_Matrix[i][j] += State_Transition_Matrix[i][k] * P_Covariance_Matrix[k][j];
            }
        }
    }

    // Step 2: Predicted_P = Temporary_Matrix * F^T
    for (std::size_t i = 0; i < States; ++i)
    {
        for (std::size_t j = 0; j < States; ++j)
        {
            for (std::size_t k = 0; k < States; ++k)
            {
                Predicted_P[i][j] += Temporary_Matrix[i][k]* State_Transition_Matrix[j][k];
            }
        }
    }

    
    for (std::size_t i = 0; i < States; ++i)
    {
        for (std::size_t j = 0; j < States; ++j)
        {
            P_Covariance_Matrix[i][j] = Predicted_P[i][j] + Process_Noise_Matrix_Q[i][j];
        }
    }
}

void KalmanFilterEstimator::ProcessMeasurement(double measured_x, double measured_y, double measured_z)
{
    // 1. Predict the next state
    for (std::size_t i = 0; i < States; ++i)
    {
        predicted_state[i] = 0.0;
    }

    for (std::size_t i = 0; i < States; ++i)
    {
        for (std::size_t j = 0; j < States; ++j)
        {
            predicted_state[i] += State_Transition_Matrix[i][j] * State_Vector[j];
        }
    }

    for (std::size_t i = 0; i < States; ++i)
    {
        State_Vector[i] = predicted_state[i];
    }

    // 2. Predict uncertainty
    PredictCovariance();

    // 3. Calculate innovation covariance
    CalculateInnovationCovariance();

    // 4. Calculate Kalman gain
    CalculateKalmanGain();

    // 5. Validate the incoming measurement
    bool passed_gate = IsWithinValidationGate(measured_x, measured_y, measured_z);

    // 6. Only correct the track if measurement passes gate
    if (passed_gate)
    {
        UpdateStateWithMeasurement(measured_x, measured_y, measured_z);
        UpdateCovariance();
    }
}
    void KalmanFilterEstimator::UpdateStateWithMeasurement( double meas_x, double meas_y,  double meas_z)
    {
        double Innovation[Positions]{};

        Innovation[0] = meas_x - State_Vector[0];

        Innovation[1] = meas_y - State_Vector[1];

        Innovation[2] = meas_z - State_Vector[2];


        // x = x_predicted + K * innovation

        for (std::size_t state = 0; state < States;++state)
        {
            double Correction = 0.0;

            for (std::size_t measurement = 0; measurement < Positions; ++measurement)
            {
                Correction += Kalman_Gain_Matrix_K[state][measurement] * Innovation[measurement];
            }

            State_Vector[state] += Correction;
        }
    }

    void KalmanFilterEstimator::UpdateCovariance()
    {
        double Identity_Minus_KH[States][States]{};
        double Updated_P[States][States]{};

        // Start with Identity Matrix
        for (std::size_t i = 0; i < States; ++i)
        {
            Identity_Minus_KH[i][i] = 1.0;
        }

        // Calculate I - K*H // // H is 3x9 and maps measurements to  // X, Y, Z.

        for (std::size_t i = 0; i < States; ++i)
        {
                for (std::size_t j = 0; j < States; ++j)
                {
                    double KH = 0.0;

                    for (std::size_t k = 0; k < Positions; ++k)
                    {
                        KH += Kalman_Gain_Matrix_K[i][k] * MeasurementMatrix_H[k][j];
                    }

                    Identity_Minus_KH[i][j] -= KH;
                }
        }

    // Updated_P = (I - KH) * P

    for (std::size_t i = 0; i < States; ++i)
    {
        for (std::size_t j = 0; j < States; ++j)
        {
            for (std::size_t k = 0; k < States; ++k)
            {
                Updated_P[i][j] += Identity_Minus_KH[i][k] * P_Covariance_Matrix[k][j];
            }
        }
    }
    // Store updated covariance

    for (std::size_t i = 0; i < States; ++i)
    {
        for (std::size_t j = 0; j < States; ++j)
        {
            P_Covariance_Matrix[i][j] = Updated_P[i][j];
        }
    }
}
    void KalmanFilterEstimator::PrintState() const
    {
        std::cout << "Final X Position: " << State_Vector[0] << " meters\n";
        std::cout << "Final X Velocity: " << State_Vector[3] << " m/s\n";
    }

    double KalmanFilterEstimator::GetEstimatedX() const
    {
        return State_Vector[0];
    }

    double KalmanFilterEstimator::GetEstimatedY() const
    {
        return State_Vector[1];
    }

    double KalmanFilterEstimator::GetEstimatedZ() const
    {
        return State_Vector[2];
    }

    double KalmanFilterEstimator::GetEstimatedVelocityX() const
    {
        return State_Vector[3];
    }

    double KalmanFilterEstimator::GetEstimatedVelocityY() const
    {
        return State_Vector[4];
    }

    double KalmanFilterEstimator::GetEstimatedVelocityZ() const
    {
        return State_Vector[5];
    }

    void KalmanFilterEstimator::SetInitialState(double x, double y, double z, double vx, double vy, double vz, double ax, double ay, double az)
    {
        State_Vector[0] = x;
        State_Vector[1] = y;
        State_Vector[2] = z;

        State_Vector[3] = vx;
        State_Vector[4] = vy;
        State_Vector[5] = vz;

        State_Vector[6] = ax;
        State_Vector[7] = ay;
        State_Vector[8] = az;
    }

    void KalmanFilterEstimator::PrintKalmanGainData() const
    {
        std::cout << "\nInnovation Covariance S:" << std::endl;

        for (std::size_t i = 0; i < Positions; ++i)
        {
            std::cout << Innovation_Covariance_Matrix_S[i][i] << std::endl;
        }

        std::cout << "\nKalman Gain K:" << std::endl;

        for (std::size_t i = 0; i < Positions; ++i)
        {
            std::cout << Kalman_Gain_Matrix_K[i][i] << std::endl;
        }
    }



