#include "AircraftNumberGenerator.h"
#include <iostream>

    void Aircraft_Number_Generator::GenerateAircraftValues()
    {
        std::uniform_real_distribution<double> X_Position_Distribution(-50000.0, 50000.0);
        std::uniform_real_distribution<double> Y_Position_Distribution(-50000.0, 50000.0);
        std::uniform_real_distribution<double> Z_Position_Distribution(1000.0, 12000.0);

        std::uniform_real_distribution<double> X_Velocity_Distribution(-250.0, 250.0);
        std::uniform_real_distribution<double> Y_Velocity_Distribution(-250.0, 250.0);
        std::uniform_real_distribution<double> Z_Velocity_Distribution(-15.0, 15.0);

        std::uniform_real_distribution<double> X_Acceleration_Distribution(-5.0, 5.0);
        std::uniform_real_distribution<double> Y_Acceleration_Distribution(-5.0, 5.0);
        std::uniform_real_distribution<double> Z_Acceleration_Distribution(-2.0, 2.0);

        X_Position = X_Position_Distribution(engine);
        Y_Position = Y_Position_Distribution(engine);
        Z_Position = Z_Position_Distribution(engine);

        X_Velocity = X_Velocity_Distribution(engine);
        Y_Velocity = Y_Velocity_Distribution(engine);
        Z_Velocity = Z_Velocity_Distribution(engine);

        X_Acceleration = X_Acceleration_Distribution(engine);
        Y_Acceleration = Y_Acceleration_Distribution(engine);
        Z_Acceleration = Z_Acceleration_Distribution(engine);
    
    }  

    void Aircraft_Number_Generator::PrintAircraftValues() const
    {
            
        std::cout << "\n==================== Simulated Aircraft Values ====================" << std::endl;

        std::cout << "\nPosition:" << std::endl;
        std::cout << "X: " << X_Position << " meters" << std::endl;
        std::cout << "Y: " << Y_Position << " meters" << std::endl;
        std::cout << "Z: " << Z_Position << " meters" << std::endl;

        std::cout << "\nVelocity:" << std::endl;
        std::cout << "Vx: " << X_Velocity << " m/s" << std::endl;
        std::cout << "Vy: " << Y_Velocity << " m/s" << std::endl;
        std::cout << "Vz: " << Z_Velocity << " m/s" << std::endl;

        std::cout << "\nAcceleration:" << std::endl;
        std::cout << "Ax: " << X_Acceleration << " m/s^2" << std::endl;
        std::cout << "Ay: " << Y_Acceleration << " m/s^2" << std::endl;
        std::cout << "Az: " << Z_Acceleration << " m/s^2" << std::endl;

        std::cout << "\n===================================================================" << std::endl;
    }

    // ==================== GET POSITION ====================
    double Aircraft_Number_Generator::GetXPosition() const
    {
        return X_Position;
    }

    double Aircraft_Number_Generator::GetYPosition() const
    {
        return Y_Position;
    }

    double Aircraft_Number_Generator::GetZPosition() const
    {
        return Z_Position;
    }

    // ==================== GET VELOCITY ====================

    double Aircraft_Number_Generator::GetXVelocity() const
    {
        return X_Velocity;
    }

    double Aircraft_Number_Generator::GetYVelocity() const
    {
        return Y_Velocity;
    }

    double Aircraft_Number_Generator::GetZVelocity() const
    {
        return Z_Velocity;
    }

    // ==================== GET ACCELERATION ====================

    double Aircraft_Number_Generator::GetXAcceleration() const
    {
        return X_Acceleration;
    }

    double Aircraft_Number_Generator::GetYAcceleration() const
    {
        return Y_Acceleration;
    }

    double Aircraft_Number_Generator::GetZAcceleration() const
    {
        return Z_Acceleration;
    }

