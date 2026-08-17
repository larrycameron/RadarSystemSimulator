#ifndef RFCALCULATIONS_H
#define RFCALCULATIONS_H

#include <cmath>


class RFCalculations
{
private:
    const double PI{3.141592653589793};        // Dimensionless
    const double Speedoflight{2.99792458e8};   // m/s

    double Received_Power{};                   // W (watts)
    double Antenna_Gained{};                   // Dimensionless (linear gain)
    double WaveLength{};                       // m (meters)
    double WaveNumber{};                       // rad/m (radians per meter)
    double Radar_Cross_Section{};              // m^2 (square meters)
    double RangeofTarget{};                    // m (meters)
    double Losses{};                           // Dimensionless (linear loss factor)

    double FSPL{};                             // Dimensionless (linear path-loss ratio)
    double Distance{};                         // m (meters)
    double Frequency{};                        // Hz (hertz)

    double Transmitterpower{};                 // W (watts)
    double TransmitterAntennaGain{};           // Dimensionless (linear gain)
    double PowerDensity{};                     // W/m^2 (watts per square meter)

    double ReferencePower{};                   // W (watts)
    double PowerDecibel{};                     // dB, or dBm when reference = 0.001 W

    double Gain{};                             // Dimensionless (linear gain)
    double Efficiency{};                       // Dimensionless (0.0 to 1.0)
    double Directivity{};                      // Dimensionless (linear)

    double DopplerFrequencyShift{};            // Hz (hertz)
    double Velocity{};                         // m/s
    double OriginalFrequency{};                // Hz (hertz)

    double Theta{};                            // rad (radians)
    double AngleFactor = std::cos(Theta);       // Dimensionless

    double GetDopplerFrequencyShift() const;
    double GetFSPL() const;
    
public:
        
        double CalculateWaveLength(double frequency);

        double CalculateWaveNumber(double wavelength);

        double CalculateReceivedPower(double transmitterpower, double antennagained, double wavelength, double radarcrosssection, double rangeoftarget, double losses);
                  
        double CalculateFSPL( double distance, double frequency); // Linear Free Path-Space Path Loss
                         
        double CalculatePowerDensity(double transmitterpower, double transmitterantennagain, double distance);
                      
        double CalculatePowerDecibel (double transmitterpower, double referencepower);
                      
        double CalculateGain(double efficiency, double directivity);
                        
        double CalculateDopplerFrequencyShift(double velocity, double originalfrequency, double anglefactor);
        
        double GetReceivedPower() const;
        
        void PrintRFCalculations() const;

};
#endif