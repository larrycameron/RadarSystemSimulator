#ifndef RADARSIMULATION_H
#define RADARSIMULATION_H

#include "../RadarTrackingSystem.h"
#include "../RFCalculations/RFCalculations.h"
#include "../EigenTracking/ThreeDimensionalTrackingData.h"
#include "../KalmanFilter/KalmanFilterEstimator.h"
#include "../RadarResolutionAndQuality/RadarResolutionAndQuality.h"
#include "../AircraftSimulatorRandomGenerator/AircraftNumberGenerator.h"

class RadarSimulation
{
private:
    RadarTrackingSystem rts;
    RFCalculations RFC;
    RadarResolutionAndQuality RRQ;
    Aircraft_Number_Generator ANG;

    ThreeDimensionalTrackingData aircraft;
    KalmanFilterEstimator filter;

double DeltaTime{0.1};

double PulseRepetitionFrequency{1000.0};
double PulseRepetitionInterval{1.0 / PulseRepetitionFrequency};
double SignalBandwidth{30e6};
double SystemNoiseTemperature{290.0};

double RadarFrequency{18e9};

double TransmitterPower{3162.28};
double AntennaGain{0.1};
double RadarCrossSection{60.0};
double SystemLosses{2.51};

double PowerDensityTransmitterPower{100.0};
double PowerDensityAntennaGain{10.0};

double ReferencePower{0.001};
double GainEfficiency{0.32};
double Directivity{4.8};

double AntennaDiameter{1.5};

double ClutterScatterCoefficient{1e-4};
double RadarClutterCellArea{5000.0};

double VoltageThreshold{4.5};
double NoisePower{0.5};

double PulseLength{50e-6};
double NumberOfPulses{128.0};

public:
    void Initialize();
    void Run();
};
#endif