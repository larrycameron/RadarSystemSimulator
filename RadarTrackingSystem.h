#ifndef RADARTRACKINGSYSTEM_H
#define RADARTRACKINGSYSTEM_H


class RadarTrackingSystem
{
    
private:
        double Initial_X_Position{};
        double Initial_Y_Position{};
        double Initial_Z_Position{};
        
        double Final_X_Position{};
        double Final_Y_Position{};
        double Final_Z_Position{};
        
        double X_Position_Velocity{};
        double Y_Position_Velocity{};
        double Z_Position_Velocity{};
        
        double X_Acceleration{};
        double Y_Acceleration{};
        double Z_Acceleration{};


        double dx{};
        double dy{};
        double dz{};
        double Total_Distance{};
        
       
public:
    void SetInitialPosition(double x_position, double y_position, double z_position);

    void GetUpdateTargetPositions( double X_Velocity,double Y_Velocity, double Z_Velocity, double X_Acceleration, double Y_Acceleration, double Z_Acceleration, double deltaTime);
    
    void GetDistanceBetweenTargets();
   
    void GetTotalDistanceFromTargets();
    
    double GetTotalDistance() const;

    double GetRangeFromRadar() const;

    double GetTotalVelocity() const;

    double GetAngleFactor() const;
    
    void PrintRadarTrackingSystemData() const;
   
};

#endif