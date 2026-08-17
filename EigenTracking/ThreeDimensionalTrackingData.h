#ifndef THREEDIMENSIONALTRACKINGDATA_H
#define THREEDIMENSIONALTRACKINGDATA_H


#include <Eigen/Dense>
#include <string>


class ThreeDimensionalTrackingData
{
private:
    using Vector9d= Eigen::Matrix<double, 9, 1>; //  X,Y, Z, Vx, Vy, Vz, Ax, Ay, Az
    using  Matrix9d = Eigen::Matrix<double, 9, 9>;
    
    Vector9d AircraftState{};
    Matrix9d StateTransitionMatrix{};
    
    double DeltaTime{};

public:
    void SetAircraftState(double X, double Y, double Z, double Vx, double VY, double VZ, double Ax, double AY, double AZ);
    void UpdateAirCraftState(double deltatime); // Time drive the state of change. Each time step updates the aircrafts position and velocity.
    void PrintAirCraftState() const;
    double GetRange() const;
    double GetSpeed() const;
    double GetVerticalRate() const;
    double GetGroundSpeed() const;
    double GetTrackAngle() const;
    std::string GetVerticalStatus() const;
    std::string GetRangeStatus() const;
    double GetRadialVelocity() const;
    double GetAzimuthAngle() const;
    double GetElevationAngle() const;
    void SetAcceleration(double Ax, double Ay, double Az);
    double GetX() const;
    double GetY() const;
    double GetZ() const;
    double GetVelocityX() const;
    double GetVelocityY() const;
    double GetVelocityZ() const;
    double GetAccelerationX() const;
    double GetAccelerationY() const;
    double GetAccelerationZ() const;
    

};

#endif

