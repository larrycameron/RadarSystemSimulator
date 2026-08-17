#ifndef AIRCRAFTSIMULATORRANDOMGENERATOR_H
#define AIRCRAFTSIMULATORRANDOMGENERATOR_H

#include <iostream>
#include <random>

class Aircraft_Number_Generator
{
private:
    std::mt19937 engine{12345};
    
    double X_Position{};
    double Y_Position{};
    double Z_Position{};

    double X_Velocity{};
    double Y_Velocity{};
    double Z_Velocity{};

    double X_Acceleration{};
    double Y_Acceleration{};
    double Z_Acceleration{};

public:
    void GenerateAircraftValues();
    void PrintAircraftValues() const;

    // ==================== GET POSITION ====================
    double GetXPosition() const;
    double GetYPosition() const;
    double GetZPosition() const;
    
    // ==================== GET VELOCITY ====================

    double GetXVelocity() const;
    double GetYVelocity() const;
    double GetZVelocity() const;
   
    // ==================== GET ACCELERATION ====================

    double GetXAcceleration() const;
    double GetYAcceleration() const;
    double GetZAcceleration() const;
   
};
#endif