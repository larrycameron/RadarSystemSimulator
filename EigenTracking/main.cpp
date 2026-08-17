
#include "ThreeDimensionalTrackingData.h"

#include <iostream>

int main()
{
    ThreeDimensionalTrackingData Aircraft;

    Aircraft.SetAircraftState(
        100, 200, 300,   // X, Y, Z
        10, 20, 30,      // Vx, Vy, Vz
        2, 4, 6          // Ax, Ay, Az
    );

    

        std::size_t TimeSamples = 10;
        double DeltaTime = 1.0;

        for (std::size_t i = 0; i < TimeSamples; ++i)
        {
            if (i == 5)
            {
                Aircraft.SetAcceleration(0, 0, -20);
            }

            Aircraft.UpdateAirCraftState(DeltaTime);
            
            std::cout << "\nTime: " << (i + 1) * DeltaTime << " seconds" << std::endl;
            
            Aircraft.PrintAirCraftState();

            
            std::cout<<"==================================Aircraft State & Derived Tracking Values=============================================================="<<std::endl;
            std::cout << "Range: "<< Aircraft.GetRange() << " meters" << std::endl;
            std::cout << "Speed: " << Aircraft.GetSpeed() << " m/s" << std::endl;
            std::cout << "Vertical Rate: " << Aircraft.GetVerticalRate() << " m/s" << std::endl;
            std::cout << "Ground Speed: " << Aircraft.GetGroundSpeed() << " m/s" << std::endl;
            std::cout << "Track Angle: " << Aircraft.GetTrackAngle() << " degrees" << std::endl;
            std::cout << "Vertical Status: " << Aircraft.GetVerticalStatus() << std::endl;
            std::cout << "Range Status: " << Aircraft.GetRangeStatus() << std::endl;
            std::cout << "Radial Velocity: " << Aircraft.GetRadialVelocity() << " m/s" << std::endl;
            std::cout << "Azimuth Angle: " << Aircraft.GetAzimuthAngle() << " degrees"<< std::endl;
            std::cout << "Elevation Angle: " << Aircraft.GetElevationAngle() << " degrees" << std::endl;
            std::cout<<"==================================================================================================================================================="<<std::endl;
        }

    return 0;

}