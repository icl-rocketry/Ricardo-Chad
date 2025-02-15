

class OdriveDriver {
public:
    OdriveDriver();

    /**
     * @brief Enum for selecting the motor axis.
     * 
     */
    enum class MotorAxis {
        MOTOR_AXIS_ZERO,
        MOTOR_AXIS_ONE
    };

    /**
     * @brief Request a position in turns from the endstop position.
     * 
     * @param axis_zero Turns for motor axis zero.
     * @param axis_one Turns for motor axis one.
     */
    void commandAxisTurns(float axis_zero, float axis_one);

    /**
     * @brief Arm the ODrive controller for axis.
     * 
     */
    void armAxis(MotorAxis motor);

    
private:
};