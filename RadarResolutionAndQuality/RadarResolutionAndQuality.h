#ifndef RADARRESOLUTIONANDQUALITY_H
#define RADARRESOLUTIONANDQUALITY_H


class RadarResolutionAndQuality
{

private:

    double const c{3e8}; // Speed of Light (m/s)
    double const k{1.380649e-23}; // Boltzmann Constant (J/K)

    double WaveLength{};
    double Velocity{};
    double Frequency{};

    double Pulse_Repetition_Interval{};
    double Pulse_Repetition_Frequency{};

    double Maximum_Unambiguous_Range{};
    double Maximum_Unambiguous_Velocity{};

    double Range_Resolution{};
    double Signal_Bandwidth{};

    double Signal_To_Noise_Ratio{};
    double Received_Power{};
    double System_Noise_Temperature{};
    double Noise_Bandwidth{};

    double Beamwidth_Approximation{};
    double Diameter{};

    double Signal_To_Clutter_Ratio{};
    double Clutter_Scatter_Coefficient{};
    double Radar_Clutter_Cell_Area{};

    double Probability_Of_False_Alarm{};
    double Voltage_Threshold_Limit{};
    double Psi_Zero_Mean_Square_Noise_Noise_Power{};

    double Original_Pulse_Length{};
    double Pulse_Compression_Ratio_Time_Bandwidth_Product{};

    double Coherent_Integration_Gain{};
    double Exact_Number_Of_Pulses{};

    double Radar_Horizon_Distance{};
                                    
    double Earths_Actual_Radius{6371000.0};// Earth radius in meters
    double Effective_Earth_Radius_Factor{1.3333};
    double Antenna_Height{};

    double GetSignalToNoiseRatio() const;

public:

    double WaveLength_Calculation(double velocity, double frequency);
    double Maximum_Unambiguous_Range_Calculation(double PRI);
    double Maximum_Unambiguous_Velocity_Calculation(double wavelength, double PRF);
    double Range_Resolution_Calculation(double signal_bandwidth);
    double Signal_To_Noise_Ratio_Calculation(double received_power, double system_noise_temperature, double noise_bandwidth);
    double Beamwidth_Approximation_Calculation(double diameter);
    double Signal_To_Clutter_Ratio_Calculation(double clutter_scatter_coefficient, double radar_clutter_cell_area);
    double Probability_Of_False_Alarm_Calculation(double voltage_threshold_limit, double psi_zero_mean_square_noise_noise_power);
    double Pulse_Compression_Ratio_Time_Bandwidth_Product_Calculation(double original_pulse_length);
    double Coherent_Integration_Gain_Calculation(double exact_number_of_pulses);
    double Radar_Horizon_Distance_Calculation(double earths_actual_radius, double effective_earth_radius_factor, double antenna_height);
    void SetRadarParameters(double velocity, double frequency, double wavelength);
    void PrintRadarResolutionAndQuality() const;

};
#endif