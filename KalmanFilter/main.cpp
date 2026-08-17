
#include "KalmanFilterEstimator.h"

#include <iostream>

int main()
{
    KalmanFilterEstimator filter;

    filter.CalculateInnovationCovariance();
    filter.CalculateKalmanGain();
    filter.PrintKalmanGainData();
   
    // Test Case: An aircraft is 10,000 meters away,
    // at a 45-degree compass heading, flying 30 degrees high in the sky.
    double test_range = 10000.0;     // meters
    double test_azimuth = 45.0;      // degrees
    double test_elevation = 30.0;    // degrees
   
    // Process step 0
    filter.TransformAndLogCoordinate(0, test_range, test_azimuth, test_elevation);
   
    std::cout << "=================== Coordinate Transformation Test ===================" << std::endl;
    std::cout << "Raw Radar Input -> Range: " << test_range << "m, Azimuth: " << test_azimuth << " deg, Elevation: " << test_elevation << " deg\n\n";
   
    
    // We will build a quick print method or access the data to verify:
    // Expected output for (10000, 45, 30):
    // X should be ~6123.72m, Y should be ~6123.72m, Z should be ~5000.00m
   
    std::cout << "======================================================================" << std::endl;
    filter.KalmanFilterPredictor();
    filter.ExecuteRadarTrackingLoop();
   
    return 0;
}