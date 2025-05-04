#include "color.h"

/* Global variables for storing the color number values */
int colorNo;

/**
 * @brief Initialize the laser(s)
 * 
 * @param port; (enum) LASERPORT; The port of the laser
 */
 void colorInit(){
    MiniR4.I2C3.MXColor.begin();
 }
