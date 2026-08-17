#include <iostream>

#include "RadarResolutionAndQuality.h"

int main()
{
    RadarResolutionAndQuality RRQ;

    // Speed of light and 10 GHz radar frequency
    double target_wavelength = RRQ.WaveLength_Calculation(3e8, 10e9);
    // 1 ms Pulse Repetition Interval
    RRQ.Maximum_Unambiguous_Range_Calculation(0.001);
    // Wavelength and 1 kHz PRF
    RRQ.Maximum_Unambiguous_Velocity_Calculation(target_wavelength, 1000.0);
    // 30 MHz signal bandwidth
    RRQ.Range_Resolution_Calculation(30e6);
    // Received Power, 290 K, and 30 MHz Noise Bandwidth
    RRQ.Signal_To_Noise_Ratio_Calculation(1e-12, 290.0, 30e6);
    // 1.5 meter antenna diameter
    RRQ.Beamwidth_Approximation_Calculation(1.5);
    // Clutter coefficient and cell area
    RRQ.Signal_To_Clutter_Ratio_Calculation(1e-4, 5000.0);
    // Threshold and mean-square noise power
    RRQ.Probability_Of_False_Alarm_Calculation(4.5, 0.5);
    // 50 microsecond pulse width
    RRQ.Pulse_Compression_Ratio_Time_Bandwidth_Product_Calculation(50e-6);
    // 128 coherently integrated pulses
    RRQ.Coherent_Integration_Gain_Calculation(128.0);
    // Earth radius, 4/3 effective Earth-radius factor, 25-meter antenna height
    RRQ.Radar_Horizon_Distance_Calculation(6371000.0, 1.3333, 25.0);
    RRQ.PrintRadarResolutionAndQuality();

    return 0;
}