#include "RadarSimulation.h"

#include <cmath>
#include <iostream>
#include <random>

void RadarSimulation::Initialize()
{
    // ==================== GENERATE ONE AIRCRAFT ====================

    ANG.GenerateAircraftValues();

    ANG.PrintAircraftValues();


    // ==================== AIRCRAFT TRACKING ====================

    rts.SetInitialPosition(ANG.GetXPosition(),ANG.GetYPosition(),ANG.GetZPosition());

    rts.GetUpdateTargetPositions( ANG.GetXVelocity(), ANG.GetYVelocity(), ANG.GetZVelocity(), ANG.GetXAcceleration(), ANG.GetYAcceleration(),ANG.GetZAcceleration(),DeltaTime);

    rts.GetDistanceBetweenTargets();

    rts.GetTotalDistanceFromTargets();

    rts.PrintRadarTrackingSystemData();

    // ==================== INITIAL RF CALCULATIONS ====================

    double AircraftRange = rts.GetRangeFromRadar();
    double AircraftVelocity = rts.GetTotalVelocity();
    double AircraftAngleFactor = rts.GetAngleFactor();

    double wavelength = RFC.CalculateWaveLength(RadarFrequency);

    RFC.CalculateWaveNumber(wavelength);

    RFC.CalculateReceivedPower(TransmitterPower, AntennaGain, wavelength, RadarCrossSection, AircraftRange, SystemLosses);

    double ReceivedPower = RFC.GetReceivedPower();

    RFC.CalculateFSPL(AircraftRange,RadarFrequency);

    RFC.CalculatePowerDensity( PowerDensityTransmitterPower, PowerDensityAntennaGain, AircraftRange);

    RFC.CalculatePowerDecibel( 10.0, ReferencePower);

    RFC.CalculateGain( GainEfficiency, Directivity);

    RFC.CalculateDopplerFrequencyShift( AircraftVelocity, RadarFrequency, AircraftAngleFactor);

    RFC.PrintRFCalculations();


    // ==================== INITIAL RADAR QUALITY ====================

    RRQ.SetRadarParameters(2.99792458e8, RadarFrequency, wavelength);

    RRQ.Maximum_Unambiguous_Range_Calculation(PulseRepetitionInterval);

    RRQ.Maximum_Unambiguous_Velocity_Calculation( wavelength, PulseRepetitionFrequency);

    RRQ.Range_Resolution_Calculation(SignalBandwidth);

    RRQ.Signal_To_Noise_Ratio_Calculation(ReceivedPower,SystemNoiseTemperature, SignalBandwidth);

    RRQ.Beamwidth_Approximation_Calculation(AntennaDiameter);

    RRQ.Signal_To_Clutter_Ratio_Calculation(ClutterScatterCoefficient, RadarClutterCellArea);

    RRQ.Probability_Of_False_Alarm_Calculation(VoltageThreshold, NoisePower);

    RRQ.Pulse_Compression_Ratio_Time_Bandwidth_Product_Calculation(PulseLength);

    RRQ.Coherent_Integration_Gain_Calculation(NumberOfPulses);

    RRQ.Radar_Horizon_Distance_Calculation(6371000.0, 1.3333, 25.0);

    RRQ.PrintRadarResolutionAndQuality();


    // ==================== INITIALIZE EIGEN ====================

    aircraft.SetAircraftState(ANG.GetXPosition(),ANG.GetYPosition(),ANG.GetZPosition(),ANG.GetXVelocity(),ANG.GetYVelocity(),ANG.GetZVelocity(),ANG.GetXAcceleration(),ANG.GetYAcceleration(),ANG.GetZAcceleration());


    // ==================== INITIALIZE KALMAN ====================

    filter.SetInitialState(ANG.GetXPosition(),ANG.GetYPosition(),ANG.GetZPosition(),ANG.GetXVelocity(),ANG.GetYVelocity(),ANG.GetZVelocity(),ANG.GetXAcceleration(),ANG.GetYAcceleration(),ANG.GetZAcceleration());
}


void RadarSimulation::Run()
{
    std::cout << "\n=================== Eigen / Kalman Multi-Step Tracking Test ===================" << std::endl;

    std::mt19937 generator(12345);

    std::normal_distribution<double> noiseX(0.0, 3.0);
    std::normal_distribution<double> noiseY(0.0, 3.0);
    std::normal_distribution<double> noiseZ(0.0, 3.0);

    double TotalRadarError = 0.0;
    double TotalKalmanError = 0.0;

    double wavelength = RFC.CalculateWaveLength(RadarFrequency);

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

        rts.SetInitialPosition(trueX,trueY,trueZ);

        rts.GetUpdateTargetPositions(trueVx,trueVy,trueVz,trueAx,trueAy,trueAz,DeltaTime);

        rts.GetDistanceBetweenTargets();

        double CurrentAircraftRange = rts.GetRangeFromRadar();
        double CurrentAircraftVelocity = rts.GetTotalVelocity();
        double CurrentAircraftAngleFactor = rts.GetAngleFactor();

        // ==================== UPDATE RF FOR CURRENT STEP ====================

        RFC.CalculateReceivedPower(TransmitterPower, AntennaGain, wavelength, RadarCrossSection, CurrentAircraftRange, SystemLosses);

        double CurrentReceivedPower = RFC.GetReceivedPower();

        RFC.CalculateFSPL(CurrentAircraftRange,RadarFrequency);

        RFC.CalculatePowerDensity(PowerDensityTransmitterPower,PowerDensityAntennaGain, CurrentAircraftRange);

        RFC.CalculateDopplerFrequencyShift( CurrentAircraftVelocity, RadarFrequency, CurrentAircraftAngleFactor);

        // ==================== UPDATE RADAR QUALITY FOR CURRENT STEP ====================

        RRQ.Signal_To_Noise_Ratio_Calculation(CurrentReceivedPower,SystemNoiseTemperature,SignalBandwidth);

        RRQ.Signal_To_Clutter_Ratio_Calculation(ClutterScatterCoefficient,RadarClutterCellArea);

        std::cout << "\nStep: " << (step + 1) << std::endl;
        std::cout << "Current Aircraft Range: " << CurrentAircraftRange << " meters" << std::endl;

        std::cout << "Current Aircraft Velocity: "<< CurrentAircraftVelocity<< " m/s" << std::endl;

        std::cout << "Current Received Power: "<< CurrentReceivedPower<< " Watts" << std::endl;

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

        filter.ProcessMeasurement(measuredX,measuredY,measuredZ);

        double estimatedX = filter.GetEstimatedX();
        double estimatedY = filter.GetEstimatedY();
        double estimatedZ = filter.GetEstimatedZ();

        // ==================== ERROR CALCULATION ====================

        double RadarPositionError = std::sqrt(std::pow(measuredX - trueX, 2) + std::pow(measuredY - trueY, 2) + std::pow(measuredZ - trueZ, 2));

        double KalmanPositionError = std::sqrt(std::pow(estimatedX - trueX, 2) + std::pow(estimatedY - trueY, 2) + std::pow(estimatedZ - trueZ, 2));

        TotalRadarError += RadarPositionError;
        TotalKalmanError += KalmanPositionError;

        std::cout << "True Position: " << trueX << ", " << trueY << ", " << trueZ << std::endl;

        std::cout << "Radar Measurement: " << measuredX << ", " << measuredY << ", " << measuredZ << std::endl;
        
        std::cout << "Kalman Estimated Position: " << estimatedX << ", "<< estimatedY << ", "<< estimatedZ << std::endl;

        std::cout << "Radar Position Error: "<< RadarPositionError<< " meters" << std::endl;

        std::cout << "Kalman Position Error: "<< KalmanPositionError << " meters" << std::endl;
    }

        double AverageRadarError = TotalRadarError / 10.0;
        double AverageKalmanError = TotalKalmanError / 10.0;

        std::cout << "\n=================== Tracking Accuracy Summary ===================" << std::endl;

        std::cout << "Average Radar Position Error: " << AverageRadarError << " meters" << std::endl; 
        
        std::cout << "Average Kalman Position Error: " << AverageKalmanError  << " meters" << std::endl;
}
