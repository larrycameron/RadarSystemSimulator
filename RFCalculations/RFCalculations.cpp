#include <cmath>
#include <iostream>
#include "RFCalculations.h"


    double RFCalculations::CalculateWaveLength(double frequency)
    {
        Frequency = frequency;

        if (Frequency <= 0.0)
        {
            std::cout << "Error: Frequency must be greater than zero." << std::endl;
            return 0.0;
        }

        WaveLength = Speedoflight / Frequency;

        return WaveLength;
    }

        double RFCalculations::CalculateWaveNumber(double wavelength)
        {
            WaveNumber = (2.0 * PI) / wavelength;

            return WaveNumber;
        }

        double RFCalculations::CalculateReceivedPower(double transmitterpower, double antennagained, double wavelength, double radarcrosssection, double rangeoftarget, double losses)
        {
                Transmitterpower = transmitterpower;
                Antenna_Gained = antennagained;
                WaveLength = wavelength;
                Radar_Cross_Section = radarcrosssection;
                RangeofTarget = rangeoftarget;
                Losses = losses;
             
                Received_Power = (Transmitterpower * std::pow(Antenna_Gained, 2) * std::pow(WaveLength, 2) * Radar_Cross_Section) /  (std::pow(4.0 * PI, 3) * std::pow(RangeofTarget, 4) *  Losses);
             
                return Received_Power;
        
        }     
           
        double RFCalculations::CalculateFSPL( double distance, double frequency) // Linear Free Path-Space Path Loss
        {
            Distance = distance;
            Frequency = frequency;
            
            FSPL = std::pow((4.0 * PI * distance * frequency) / Speedoflight, 2.0);
            
            return  FSPL;
        }
                  
        double RFCalculations::CalculatePowerDensity(double transmitterpower, double transmitterantennagain, double distance)
        {
            Transmitterpower = transmitterpower;
            TransmitterAntennaGain = transmitterantennagain;
            Distance = distance;
             
            PowerDensity= (Transmitterpower * TransmitterAntennaGain) /(4.0 * PI *std::pow(Distance,2));
                
            return PowerDensity;
        }
              
        double RFCalculations::CalculatePowerDecibel (double transmitterpower, double referencepower)
        {
            
            Transmitterpower = transmitterpower;
            ReferencePower = referencepower;
                    
            PowerDecibel = 10 * std::log10(transmitterpower / referencepower);
            
            return PowerDecibel;
        }
              
        double RFCalculations::CalculateGain(double efficiency, double directivity)
        {
            Efficiency = efficiency;
            Directivity = directivity;
            
            Gain = efficiency * directivity;
            
            return Gain;
            
        }
                
        double RFCalculations::CalculateDopplerFrequencyShift(double velocity, double originalfrequency, double anglefactor)
        {
            Velocity = velocity;
            OriginalFrequency = originalfrequency;
            AngleFactor = anglefactor;
            
            DopplerFrequencyShift = (2 * velocity * originalfrequency) /(Speedoflight) * anglefactor;
            
            return DopplerFrequencyShift;
            
            
        }

        double RFCalculations::GetReceivedPower() const
        {
            return Received_Power;
        }

        double RFCalculations::GetDopplerFrequencyShift() const
        {
            return DopplerFrequencyShift;
        }

        double RFCalculations::GetFSPL() const
        {
            return FSPL;
        }
        
        void RFCalculations::PrintRFCalculations() const
        {
            std::cout << "================ RF Calculation Summary =================================================="<< std::endl;
            std::cout << "Radar Operating Frequency: " << Frequency / 1e9  << " GHz" << std::endl;
            std::cout << "Wavelength: " << WaveLength << " meters" << std::endl;
            std::cout << "Wavenumber: " << WaveNumber << " radians per meter" << std::endl;
            std::cout << "Received Radar Power: " << Received_Power << " Watts"<< std::endl;
            std::cout << "Free-Space Path Loss: " << FSPL << " (linear)" << std::endl;
            std::cout << "Power Density at Target: " << PowerDensity << " watts per square meter" << std::endl;
            std::cout << "Transmitter Power Level: " << PowerDecibel << " dBm" << std::endl;
            std::cout << "Antenna Gain: "<< Gain << " (linear)" << std::endl;
            std::cout << "Doppler Frequency Shift: " << DopplerFrequencyShift << " Hertz" << std::endl;
            std::cout << "============================================================================================"<< std::endl;
}
        