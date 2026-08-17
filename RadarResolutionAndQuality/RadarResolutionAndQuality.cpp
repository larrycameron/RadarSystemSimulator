#include "RadarResolutionAndQuality.h"

#include <iostream>
#include <cmath>


double RadarResolutionAndQuality::WaveLength_Calculation(
    double velocity,
    double frequency)
{
    Velocity = velocity;
    Frequency = frequency;

    if (Frequency <= 0.0)
    {
        std::cout
            << "Error: Frequency must be greater than zero."
            << std::endl;

        return 0.0;
    }

    WaveLength = Velocity / Frequency;

    return WaveLength;
}


double RadarResolutionAndQuality::Maximum_Unambiguous_Range_Calculation(
    double PRI)
{
    Pulse_Repetition_Interval = PRI;

    if (Pulse_Repetition_Interval <= 0.0)
    {
        std::cout
            << "Error: Pulse Repetition Interval must be greater than zero."
            << std::endl;

        return 0.0;
    }

    Maximum_Unambiguous_Range =
        (c * Pulse_Repetition_Interval) / 2.0;

    return Maximum_Unambiguous_Range;
}


double RadarResolutionAndQuality::Maximum_Unambiguous_Velocity_Calculation(
    double wavelength,
    double PRF)
{
    WaveLength = wavelength;
    Pulse_Repetition_Frequency = PRF;

    if (WaveLength <= 0.0 ||
        Pulse_Repetition_Frequency <= 0.0)
    {
        std::cout
            << "Error: Wavelength and PRF must be greater than zero."
            << std::endl;

        return 0.0;
    }

    // Standard monostatic pulse-Doppler relationship
    Maximum_Unambiguous_Velocity =
        (WaveLength * Pulse_Repetition_Frequency) / 4.0;

    return Maximum_Unambiguous_Velocity;
}


double RadarResolutionAndQuality::Range_Resolution_Calculation(
    double signal_bandwidth)
{
    Signal_Bandwidth = signal_bandwidth;

    if (Signal_Bandwidth <= 0.0)
    {
        std::cout
            << "Error: Signal Bandwidth must be greater than zero."
            << std::endl;

        return 0.0;
    }

    Range_Resolution =
        c / (2.0 * Signal_Bandwidth);

    return Range_Resolution;
}


double RadarResolutionAndQuality::Signal_To_Noise_Ratio_Calculation(
    double received_power,
    double system_noise_temperature,
    double noise_bandwidth)
{
    Received_Power = received_power;
    System_Noise_Temperature = system_noise_temperature;
    Noise_Bandwidth = noise_bandwidth;

    if (System_Noise_Temperature <= 0.0 ||
        Noise_Bandwidth <= 0.0)
    {
        std::cout
            << "Error: Noise Temperature and Noise Bandwidth must be greater than zero."
            << std::endl;

        return 0.0;
    }

    Signal_To_Noise_Ratio =
        Received_Power /
        (k * System_Noise_Temperature * Noise_Bandwidth);

    return Signal_To_Noise_Ratio;
}


double RadarResolutionAndQuality::Beamwidth_Approximation_Calculation(
    double diameter)
{
    Diameter = diameter;

    if (Diameter <= 0.0)
    {
        std::cout
            << "Error: Antenna Diameter must be greater than zero."
            << std::endl;

        return 0.0;
    }

    if (WaveLength <= 0.0)
    {
        std::cout
            << "Error: Wavelength must be calculated first."
            << std::endl;

        return 0.0;
    }

    Beamwidth_Approximation =
        WaveLength / Diameter;

    return Beamwidth_Approximation;
}


double RadarResolutionAndQuality::Signal_To_Clutter_Ratio_Calculation(
    double clutter_scatter_coefficient,
    double radar_clutter_cell_area)
{
    Clutter_Scatter_Coefficient =
        clutter_scatter_coefficient;

    Radar_Clutter_Cell_Area =
        radar_clutter_cell_area;

    if (Clutter_Scatter_Coefficient <= 0.0 ||
        Radar_Clutter_Cell_Area <= 0.0)
    {
        std::cout
            << "Error: Clutter coefficient and cell area must be greater than zero."
            << std::endl;

        return 0.0;
    }

    Signal_To_Clutter_Ratio =
        Received_Power /
        (Clutter_Scatter_Coefficient *
         Radar_Clutter_Cell_Area);

    return Signal_To_Clutter_Ratio;
}


double RadarResolutionAndQuality::Probability_Of_False_Alarm_Calculation(
    double voltage_threshold_limit,
    double psi_zero_mean_square_noise_noise_power)
{
    Voltage_Threshold_Limit =
        voltage_threshold_limit;

    Psi_Zero_Mean_Square_Noise_Noise_Power =
        psi_zero_mean_square_noise_noise_power;

    if (Psi_Zero_Mean_Square_Noise_Noise_Power <= 0.0)
    {
        std::cout
            << "Error: Noise Power must be greater than zero."
            << std::endl;

        return 0.0;
    }

    Probability_Of_False_Alarm =
        std::exp(
            -std::pow(Voltage_Threshold_Limit, 2) /
            (2.0 *
             Psi_Zero_Mean_Square_Noise_Noise_Power));

    return Probability_Of_False_Alarm;
}


double RadarResolutionAndQuality::
Pulse_Compression_Ratio_Time_Bandwidth_Product_Calculation(
    double original_pulse_length)
{
    Original_Pulse_Length =
        original_pulse_length;

    if (Original_Pulse_Length <= 0.0)
    {
        std::cout
            << "Error: Pulse Length must be greater than zero."
            << std::endl;

        return 0.0;
    }

    if (Signal_Bandwidth <= 0.0)
    {
        std::cout
            << "Error: Signal Bandwidth must be calculated first."
            << std::endl;

        return 0.0;
    }

    Pulse_Compression_Ratio_Time_Bandwidth_Product =
        Original_Pulse_Length *
        Signal_Bandwidth;

    return Pulse_Compression_Ratio_Time_Bandwidth_Product;
}


double RadarResolutionAndQuality::Coherent_Integration_Gain_Calculation(
    double exact_number_of_pulses)
{
    Exact_Number_Of_Pulses =
        exact_number_of_pulses;

    if (Exact_Number_Of_Pulses <= 0.0)
    {
        std::cout
            << "Error: Number of Pulses must be greater than zero."
            << std::endl;

        return 0.0;
    }

    Coherent_Integration_Gain =
        10.0 * std::log10(Exact_Number_Of_Pulses);

    return Coherent_Integration_Gain;
}


double RadarResolutionAndQuality::Radar_Horizon_Distance_Calculation(
    double earths_actual_radius,
    double effective_earth_radius_factor,
    double antenna_height)
{
    Earths_Actual_Radius =
        earths_actual_radius;

    Effective_Earth_Radius_Factor =
        effective_earth_radius_factor;

    Antenna_Height =
        antenna_height;

    if (Earths_Actual_Radius <= 0.0 ||
        Effective_Earth_Radius_Factor <= 0.0 ||
        Antenna_Height < 0.0)
    {
        std::cout
            << "Error: Invalid Radar Horizon values."
            << std::endl;

        return 0.0;
    }

    Radar_Horizon_Distance =
        std::sqrt(
            2.0 *
            Effective_Earth_Radius_Factor *
            Earths_Actual_Radius *
            Antenna_Height);

    return Radar_Horizon_Distance;
}


    double RadarResolutionAndQuality::GetSignalToNoiseRatio() const
    {
        return Signal_To_Noise_Ratio;
    }

    void RadarResolutionAndQuality::SetRadarParameters( double velocity, double frequency,double wavelength)
    {
        Velocity = velocity;
        Frequency = frequency;
        WaveLength = wavelength;
    }


void RadarResolutionAndQuality::PrintRadarResolutionAndQuality() const
{
    std::cout<< "\n============================= Radar Resolution And Quality Calculations ==================================================="<< std::endl;
    std::cout << "WaveLength: "<< WaveLength << " meters" << std::endl;
    std::cout << "Velocity: " << Velocity << " m/s"<< std::endl;
    std::cout << "Frequency: " << Frequency << " Hz" << std::endl;
    std::cout << "Pulse Repetition Interval: " << Pulse_Repetition_Interval << " seconds" << std::endl;
    std::cout << "Pulse Repetition Frequency: " << Pulse_Repetition_Frequency << " Hz" << std::endl;
    std::cout << "Maximum Unambiguous Range: " << Maximum_Unambiguous_Range << " meters" << std::endl;
    std::cout << "Maximum Unambiguous Velocity: " << Maximum_Unambiguous_Velocity << " m/s" << std::endl;
    std::cout << "Range Resolution: " << Range_Resolution << " meters" << std::endl;
    std::cout << "Signal Bandwidth: " << Signal_Bandwidth << " Hz" << std::endl;
    std::cout << "Signal To Noise Ratio: " << Signal_To_Noise_Ratio << " (linear)" << std::endl;

    std::cout << "Received Power: " << Received_Power << " Watts" << std::endl;
    std::cout << "System Noise Temperature: " << System_Noise_Temperature << " Kelvin" << std::endl;
    std::cout << "Noise Bandwidth: " << Noise_Bandwidth<< " Hz" << std::endl;
    std::cout<< "Beamwidth Approximation: " << Beamwidth_Approximation << " radians" << std::endl;
    std::cout << "Antenna Diameter: " << Diameter << " meters" << std::endl;
    std::cout << "Signal To Clutter Ratio: " << Signal_To_Clutter_Ratio << " (linear)" << std::endl;
    std::cout << "Clutter Scatter Coefficient: " << Clutter_Scatter_Coefficient << std::endl;
    std::cout<< "Radar Clutter Cell Area: " << Radar_Clutter_Cell_Area  << " m^2" << std::endl;
    std::cout << "Probability of False Alarm: "<< Probability_Of_False_Alarm << std::endl;
    std::cout << "Voltage Threshold Limit: " << Voltage_Threshold_Limit << " Volts" << std::endl;
    std::cout << "Psi Zero (Noise Power): " << Psi_Zero_Mean_Square_Noise_Noise_Power << " Watts" << std::endl;
    std::cout << "Original Pulse Length: " << Original_Pulse_Length << " seconds" << std::endl;
    std::cout << "Pulse Compression Ratio (Time-Bandwidth Product): " << Pulse_Compression_Ratio_Time_Bandwidth_Product << std::endl;
    std::cout<< "Coherent Integration Gain: " << Coherent_Integration_Gain << " dB"<< std::endl;
    std::cout<< "Exact Number of Pulses: "<< Exact_Number_Of_Pulses << std::endl;
    std::cout<< "Radar Horizon Distance: " << Radar_Horizon_Distance << " meters" << std::endl;
    std::cout<< "\n===========================================================================================================================================\n"<< std::endl;
}



