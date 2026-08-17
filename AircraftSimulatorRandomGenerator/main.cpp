#include "AircraftNumberGenerator.h"
#include <iostream>


int main()
{
        Aircraft_Number_Generator  ANG;

        const std::size_t Sample_Size = 25;
    
        for (std::size_t sample = 0; sample < Sample_Size; ++sample)
        {
            std::cout << "\nSample: " << (sample + 1) << std::endl;
            
            ANG.GenerateAircraftValues();
            ANG.PrintAircraftValues();
        }

        return 0;
}