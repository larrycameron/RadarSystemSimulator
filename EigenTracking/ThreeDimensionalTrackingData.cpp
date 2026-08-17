#include "ThreeDimensionalTrackingData.h"

#include <iostream>
#include <cmath>

 void ThreeDimensionalTrackingData::SetAircraftState(double X, double Y, double Z, double Vx, double VY, double VZ, double Ax, double AY, double AZ)
    {
        AircraftState << X, Y, Z, Vx, VY, VZ, Ax, AY, AZ;//AircraftState represents the aircraft's complete kinematic state
    }
    void ThreeDimensionalTrackingData::UpdateAirCraftState(double deltatime) // Time drive the state of change. Each time step updates the aircrafts position and velocity.
    {
        DeltaTime = deltatime;

        StateTransitionMatrix << // Matrix Representation of Constant Acceleration Kinematic Model
            1, 0, 0, DeltaTime, 0, 0, 0.5 * DeltaTime * DeltaTime, 0, 0,
            0, 1, 0, 0, DeltaTime, 0, 0, 0.5 * DeltaTime * DeltaTime, 0,
            0, 0, 1, 0, 0, DeltaTime, 0, 0, 0.5 * DeltaTime * DeltaTime,

            0, 0, 0, 1, 0, 0, DeltaTime, 0, 0,
            0, 0, 0, 0, 1, 0, 0, DeltaTime, 0,
            0, 0, 0, 0, 0, 1, 0, 0, DeltaTime,

            0, 0, 0, 0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 1;

         AircraftState = StateTransitionMatrix * AircraftState; // Matrix-Vector Multiplication
    }

    void ThreeDimensionalTrackingData::PrintAirCraftState() const
    {
        std::cout << "Position: " <<AircraftState(0) << " ," << AircraftState(1) << " ," << AircraftState(2)<< std::endl;

        std::cout << "Velocity: " <<AircraftState(3) << " ," << AircraftState(4) << " ," << AircraftState(5)<< std::endl;

        std::cout << "Acceleration: " <<AircraftState(6) << " ," << AircraftState(7) << " ," << AircraftState(8)<< std::endl;
    }

    double ThreeDimensionalTrackingData::GetRange() const
    {
        return std::sqrt(AircraftState(0) * AircraftState(0) +  AircraftState(1) * AircraftState(1) + AircraftState(2) * AircraftState(2));
    }

    double ThreeDimensionalTrackingData::GetSpeed() const
    {
        return std::sqrt(AircraftState(3) * AircraftState(3) +  AircraftState(4) * AircraftState(4) + AircraftState(5) * AircraftState(5));
    }

    double ThreeDimensionalTrackingData::GetVerticalRate() const
    {
        return AircraftState(5);
    }

    double ThreeDimensionalTrackingData::GetGroundSpeed() const
    {
        return std::sqrt(AircraftState(3) * AircraftState(3) +  AircraftState(4) * AircraftState(4));
    }

    double ThreeDimensionalTrackingData::GetTrackAngle() const
    {
        double AngleRadians = std::atan2(AircraftState(4),AircraftState(3));    // // atan2(Vy, Vx)
        return AngleRadians * (180.0 / 3.141592653589793);
    }

    std::string ThreeDimensionalTrackingData::GetVerticalStatus() const
    {
        if (AircraftState(5) > 0)
        {
        return "Climbing";
        }
        else if (AircraftState(5) < 0)
        {
        return "Descending";
        }
        else
        {
        return "Level";
        }
    }

    std::string ThreeDimensionalTrackingData::GetRangeStatus() const
    {
        double DotProduct = AircraftState(0) * AircraftState(3) + AircraftState(1) * AircraftState(4) + AircraftState(2) * AircraftState(5);

        if (DotProduct > 0)
        {
            return "Opening";
        }
        else if (DotProduct < 0)
        {
        return "Closing";
        }
        else
        {
        return "Constant Range";
        }
    }

    double ThreeDimensionalTrackingData::GetRadialVelocity() const
    {
        double Range = GetRange();

        if (Range == 0)
        {
            return 0;
        }

        return (AircraftState(0) * AircraftState(3) + AircraftState(1) * AircraftState(4) + AircraftState(2) * AircraftState(5)) / Range;
    }

    double ThreeDimensionalTrackingData::GetAzimuthAngle() const
    {
        double AngleRadians = std::atan2(AircraftState(1), AircraftState(0));

        return AngleRadians * (180.0 / 3.141592653589793);
    }

    double ThreeDimensionalTrackingData::GetElevationAngle() const
    {
        double HorizontalDistance = std::sqrt(AircraftState(0) * AircraftState(0) + AircraftState(1) * AircraftState(1));

        double AngleRadians = std::atan2(AircraftState(2), HorizontalDistance);

        return AngleRadians * (180.0 / 3.141592653589793);
    }

    void ThreeDimensionalTrackingData::SetAcceleration(double Ax, double Ay, double Az)
    {
        AircraftState(6) = Ax;
        AircraftState(7) = Ay;
        AircraftState(8) = Az;
    }

    //====================X,Y.Z================================
    double ThreeDimensionalTrackingData::GetX() const
    {
        return AircraftState(0);
    }

    double ThreeDimensionalTrackingData::GetY() const
    {
        return AircraftState(1);
    }

    double ThreeDimensionalTrackingData::GetZ() const
    {
         return AircraftState(2);
    }

    //==============VelocityX, VelocityY,  Velocity Z=======================
    double ThreeDimensionalTrackingData::GetVelocityX() const
    {
        return AircraftState(3);
    }

    double ThreeDimensionalTrackingData::GetVelocityY() const
    {
        return AircraftState(4);
    }

    double ThreeDimensionalTrackingData::GetVelocityZ() const
    {
         return AircraftState(5);
    }

    //==========================AcclerationX, AcclerationY, Acceleration Z==============================================
    double ThreeDimensionalTrackingData::GetAccelerationX() const
    {
        return AircraftState(6);
    }

    double ThreeDimensionalTrackingData::GetAccelerationY() const
    {
        return AircraftState(7);
    }

    double ThreeDimensionalTrackingData::GetAccelerationZ() const
    {
         return AircraftState(8);
    }