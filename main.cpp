
#include "RadarSimulation/RadarSimulation.h"

int main()
{
    RadarSimulation simulation;

    simulation.Initialize();
    simulation.Run();

    return 0;
}









/* #include <iostream>
#include <cmath>
#include <random>

#include "RadarTrackingSystem.h"
#include "RFCalculations/RFCalculations.h"
#include "EigenTracking/ThreeDimensionalTrackingData.h"
#include "KalmanFilter/KalmanFilterEstimator.h"
#include "RadarResolutionAndQuality/RadarResolutionAndQuality.h"



int main()
{
        
    RadarTrackingSystem rts;
    RFCalculations RFC;
    RadarResolutionAndQuality RRQ;
    Aircraft_Number_Generator ANG;

    // ==================== SIMULATION TIME STEP ====================
    double DeltaTime= 0.1;
    double PulseRepetitionFrequency = 1000.0;
    double PulseRepetitionInterval = 1.0 / PulseRepetitionFrequency;
    double SignalBandwidth = 30e6;
    double SystemNoiseTemperature = 290.0;

    double RadarFrequency = 18e9;

    double TransmitterPower = 3162.28;
    double AntennaGain = 0.1;
    double RadarCrossSection = 60.0;
    double SystemLosses = 2.51;

    double PowerDensityTransmitterPower = 100.0;
    double PowerDensityAntennaGain = 10.0;

    double ReferencePower = 0.001;
    double GainEfficiency = 0.32;
    double Directivity = 4.8;

    double AntennaDiameter = 1.5;

    double ClutterScatterCoefficient = 1e-4;
    double RadarClutterCellArea = 5000.0;

    double VoltageThreshold = 4.5;
    double NoisePower = 0.5;

    double PulseLength = 50e-6;
    double NumberOfPulses = 128.0;
    

    // ==================== GENERATE ONE AIRCRAFT ====================

    ANG.GenerateAircraftValues();

    ANG.PrintAircraftValues();


    // ==================== AIRCRAFT TRACKING ====================

    rts.SetInitialPosition(ANG.GetXPosition(), ANG.GetYPosition(), ANG.GetZPosition());

    rts.GetUpdateTargetPositions( ANG.GetXVelocity(),ANG.GetYVelocity(), ANG.GetZVelocity(),ANG.GetXAcceleration(),ANG.GetYAcceleration(),ANG.GetZAcceleration(),DeltaTime);
    
    rts.GetDistanceBetweenTargets();

    rts.GetTotalDistanceFromTargets();

    double AircraftRange = rts.GetRangeFromRadar();

    double AircraftVelocity = rts.GetTotalVelocity();

    double AircraftAngleFactor = rts.GetAngleFactor();

    rts.PrintRadarTrackingSystemData();

   // ==================== RF CALCULATIONS ====================

    
    double wavelength = RFC.CalculateWaveLength(RadarFrequency);

    RFC.CalculateWaveNumber(wavelength);

    RFC.CalculateReceivedPower(TransmitterPower, AntennaGain, wavelength, RadarCrossSection, AircraftRange, SystemLosses);

    double ReceivedPower = RFC.GetReceivedPower();

    RFC.CalculateFSPL(AircraftRange, RadarFrequency);

    RFC.CalculatePowerDensity(PowerDensityTransmitterPower, PowerDensityAntennaGain, AircraftRange);

    RFC.CalculatePowerDecibel(10.0, ReferencePower);

    RFC.CalculateGain(GainEfficiency, Directivity);

    RFC.CalculateDopplerFrequencyShift(AircraftVelocity, RadarFrequency, AircraftAngleFactor);

    RFC.PrintRFCalculations();

   // ==================== RADAR RESOLUTION AND QUALITY ====================

    // Calculate wavelength using the same 18 GHz radar frequency
    double QualityWavelength = wavelength;

    RRQ.SetRadarParameters(2.99792458e8, RadarFrequency, wavelength);

    // 1 millisecond Pulse Repetition Interval
    RRQ.Maximum_Unambiguous_Range_Calculation(PulseRepetitionInterval);

    // 1 kHz Pulse Repetition Frequency
    RRQ.Maximum_Unambiguous_Velocity_Calculation(QualityWavelength, PulseRepetitionFrequency);

    // 30 MHz signal bandwidth
    RRQ.Range_Resolution_Calculation(SignalBandwidth);

    // Received power, system noise temperature, and noise bandwidth
    RRQ.Signal_To_Noise_Ratio_Calculation(ReceivedPower, SystemNoiseTemperature, SignalBandwidth);

    // 1.5 meter antenna diameter
    RRQ.Beamwidth_Approximation_Calculation(AntennaDiameter);

    // Clutter scatter coefficient and radar clutter cell area
    RRQ.Signal_To_Clutter_Ratio_Calculation(ClutterScatterCoefficient, RadarClutterCellArea);

    // Detection threshold and mean-square noise power
    RRQ.Probability_Of_False_Alarm_Calculation(VoltageThreshold, NoisePower);

    // 50 microsecond pulse length
    RRQ.Pulse_Compression_Ratio_Time_Bandwidth_Product_Calculation(PulseLength);

    // 128 coherently integrated pulses
    RRQ.Coherent_Integration_Gain_Calculation(NumberOfPulses);

    // Earth radius, effective Earth-radius factor, antenna height
    RRQ.Radar_Horizon_Distance_Calculation(6371000.0, 1.3333, 25.0);

    RRQ.PrintRadarResolutionAndQuality();

    
   // ==================== EIGEN / KALMAN MULTI-STEP INTEGRATION TEST ====================

    ThreeDimensionalTrackingData aircraft;
    KalmanFilterEstimator filter;

    
    // Give the generated state to Eigen
    aircraft.SetAircraftState(ANG.GetXPosition(), ANG.GetYPosition(), ANG.GetZPosition(), ANG.GetXVelocity(), ANG.GetYVelocity(), ANG.GetZVelocity(), ANG.GetXAcceleration(), ANG.GetYAcceleration(), ANG.GetZAcceleration());

    // Give the same starting state to the Kalman filter
    filter.SetInitialState( ANG.GetXPosition(), ANG.GetYPosition(), ANG.GetZPosition(), ANG.GetXVelocity(), ANG.GetYVelocity(), ANG.GetZVelocity(), ANG.GetXAcceleration(), ANG.GetYAcceleration(), ANG.GetZAcceleration());

    std::cout << "\n=================== Eigen / Kalman Multi-Step Tracking Test ===================" << std::endl;

    std::mt19937 generator(12345);

    std::normal_distribution<double> noiseX(0.0, 3.0);
    std::normal_distribution<double> noiseY(0.0, 3.0);
    std::normal_distribution<double> noiseZ(0.0, 3.0);

    double TotalRadarError = 0.0;
    double TotalKalmanError = 0.0;

    for (std::size_t step = 0; step < 10; ++step)
    {
    // ==================== CURRENT AIRCRAFT STATE ====================

        double trueX = aircraft.GetX();
        double trueY = aircraft.GetY();
        double trueZ = aircraft.GetZ();

        double trueVx = aircraft.GetVelocityX();
        double trueVy = aircraft.GetVelocityY();
        double trueVz = aircraft.GetVelocityZ();

        double trueAx = aircraft.GetAccelerationX();
        double trueAy = aircraft.GetAccelerationY();
        double trueAz = aircraft.GetAccelerationZ();

        // ==================== UPDATE RADAR TRACKING SYSTEM ====================

        rts.SetInitialPosition(trueX, trueY,trueZ);

        rts.GetUpdateTargetPositions(trueVx, trueVy, trueVz, trueAx, trueAy, trueAz, DeltaTime);

        rts.GetDistanceBetweenTargets();

        double CurrentAircraftRange = rts.GetRangeFromRadar();
        double CurrentAircraftVelocity = rts.GetTotalVelocity();
        double CurrentAircraftAngleFactor = rts.GetAngleFactor();

        // ==================== UPDATE RF FOR CURRENT STEP ====================

        RFC.CalculateReceivedPower(TransmitterPower, AntennaGain, wavelength, RadarCrossSection, CurrentAircraftRange, SystemLosses);

        double CurrentReceivedPower = RFC.GetReceivedPower();

        RFC.CalculateFSPL(CurrentAircraftRange, RadarFrequency);

        RFC.CalculatePowerDensity(PowerDensityTransmitterPower, PowerDensityAntennaGain, CurrentAircraftRange);

        RFC.CalculateDopplerFrequencyShift(CurrentAircraftVelocity, RadarFrequency, CurrentAircraftAngleFactor);


        // ==================== UPDATE RADAR QUALITY FOR CURRENT STEP ====================

        RRQ.Signal_To_Noise_Ratio_Calculation(CurrentReceivedPower, SystemNoiseTemperature, SignalBandwidth);

        RRQ.Signal_To_Clutter_Ratio_Calculation(ClutterScatterCoefficient,RadarClutterCellArea);
        
        std::cout << "\nStep: " << (step + 1) << std::endl;
        std::cout << "Current Aircraft Range: " << CurrentAircraftRange << " meters" << std::endl;
        std::cout << "Current Aircraft Velocity: " << CurrentAircraftVelocity << " m/s" << std::endl;
        std::cout << "Current Received Power: " << CurrentReceivedPower << " Watts" << std::endl;
        // ==================== ADVANCE TRUE AIRCRAFT ====================

        aircraft.UpdateAirCraftState(DeltaTime);

        trueX = aircraft.GetX();
        trueY = aircraft.GetY();
        trueZ = aircraft.GetZ();


        // ==================== RADAR MEASUREMENT ====================

        double measuredX = trueX + noiseX(generator);
        double measuredY = trueY + noiseY(generator);
        double measuredZ = trueZ + noiseZ(generator);
        
        // ==================== KALMAN UPDATE ====================

        filter.ProcessMeasurement(measuredX, measuredY,measuredZ);

        double estimatedX = filter.GetEstimatedX();
        double estimatedY = filter.GetEstimatedY();
        double estimatedZ = filter.GetEstimatedZ();

        // ==================== ERROR CALCULATION ====================

        double RadarPositionError = std::sqrt(std::pow(measuredX - trueX, 2) + std::pow(measuredY - trueY, 2) + std::pow(measuredZ - trueZ, 2));

        double KalmanPositionError = std::sqrt(std::pow(estimatedX - trueX, 2) + std::pow(estimatedY - trueY, 2) + std::pow(estimatedZ - trueZ, 2));

        TotalRadarError += RadarPositionError;
        TotalKalmanError += KalmanPositionError;

        std::cout << "True Position: "<< trueX << ", " << trueY << ", " << trueZ << std::endl;

        std::cout << "Radar Measurement: " << measuredX << ", " << measuredY << ", " << measuredZ << std::endl;

        std::cout << "Kalman Estimated Position: " << estimatedX << ", " << estimatedY << ", " << estimatedZ << std::endl;

        std::cout << "Radar Position Error: "<< RadarPositionError << " meters" << std::endl;

        std::cout << "Kalman Position Error: "<< KalmanPositionError << " meters" << std::endl;
    }

        double AverageRadarError = TotalRadarError / 10.0;
        double AverageKalmanError = TotalKalmanError / 10.0;

        std::cout << "\n=================== Tracking Accuracy Summary ===================" << std::endl;

        std::cout << "Average Radar Position Error: " << AverageRadarError << " meters" << std::endl;

        std::cout << "Average Kalman Position Error: " << AverageKalmanError << " meters" << std::endl;

        return 0;

}*/