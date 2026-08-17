#include <cmath>
#include <iostream>
#include "RadarTrackingSystem.h"


void RadarTrackingSystem::GetUpdateTargetPositions(double X_Velocity, double Y_Velocity, double Z_Velocity, double X_Acceleration_Input,double Y_Acceleration_Input,double Z_Acceleration_Input,double deltaTime)
{
    X_Position_Velocity = X_Velocity;
    Y_Position_Velocity = Y_Velocity;
    Z_Position_Velocity = Z_Velocity;

    X_Acceleration = X_Acceleration_Input;
    Y_Acceleration = Y_Acceleration_Input;
    Z_Acceleration = Z_Acceleration_Input;

    // Constant-acceleration position equations
    Final_X_Position = Initial_X_Position + X_Position_Velocity * deltaTime + 0.5 * X_Acceleration * deltaTime * deltaTime;

    Final_Y_Position = Initial_Y_Position + Y_Position_Velocity * deltaTime + 0.5 * Y_Acceleration * deltaTime * deltaTime;

    Final_Z_Position = Initial_Z_Position + Z_Position_Velocity * deltaTime + 0.5 * Z_Acceleration * deltaTime * deltaTime;

    // Update velocity after the time step
    X_Position_Velocity = X_Position_Velocity + X_Acceleration * deltaTime;

    Y_Position_Velocity = Y_Position_Velocity + Y_Acceleration * deltaTime;

    Z_Position_Velocity = Z_Position_Velocity + Z_Acceleration * deltaTime;
}

   void RadarTrackingSystem::GetDistanceBetweenTargets()
    {
        dx = Final_X_Position - Initial_X_Position;
        dy = Final_Y_Position - Initial_Y_Position;
        dz = Final_Z_Position - Initial_Z_Position;
    }
    
    void RadarTrackingSystem::GetTotalDistanceFromTargets() 
    {
        Total_Distance = std::sqrt((dx * dx) + (dy * dy) + (dz * dz));
        
        std::cout << "Aircraft Distance Traveled:  " << Total_Distance << " Meters"<< std::endl;
    }
    
    double RadarTrackingSystem::GetTotalDistance() const
    {
        return Total_Distance;
    }

    double RadarTrackingSystem::GetTotalVelocity() const
    {
        return std::sqrt((X_Position_Velocity * X_Position_Velocity) + (Y_Position_Velocity * Y_Position_Velocity) +(Z_Position_Velocity * Z_Position_Velocity));
    }

    double RadarTrackingSystem::GetAngleFactor() const
    {
        double DotProduct = (Final_X_Position * X_Position_Velocity) + (Final_Y_Position * Y_Position_Velocity) + (Final_Z_Position * Z_Position_Velocity);

        double PositionMagnitude = std::sqrt((Final_X_Position * Final_X_Position) + (Final_Y_Position * Final_Y_Position) +(Final_Z_Position * Final_Z_Position));

        double VelocityMagnitude = std::sqrt((X_Position_Velocity * X_Position_Velocity) + (Y_Position_Velocity * Y_Position_Velocity) +(Z_Position_Velocity * Z_Position_Velocity));

        return DotProduct / (PositionMagnitude * VelocityMagnitude);
    }
    
    void RadarTrackingSystem::SetInitialPosition(double x_position, double y_position, double z_position)
    {
        Initial_X_Position = x_position;
        Initial_Y_Position = y_position;
        Initial_Z_Position = z_position;
    }

    double RadarTrackingSystem::GetRangeFromRadar() const
    {
        return std::sqrt((Final_X_Position * Final_X_Position) + (Final_Y_Position * Final_Y_Position) + (Final_Z_Position * Final_Z_Position));
    }

    void RadarTrackingSystem::PrintRadarTrackingSystemData() const
    {
        std::cout << "Aircraft Position (x, y, z): " << "X:" << Final_X_Position << " Y:" << Final_Y_Position << " Z:" << Final_Z_Position << std::endl;

        std::cout << "Aircraft Velocity: " << GetTotalVelocity() << " m/s" << std::endl;

        std::cout << "Aircraft Angle Factor: " << GetAngleFactor() << std::endl;
    }