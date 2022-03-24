package frc.robot.motor;

import frc.robot.Gains;

/**
 * The MotorController interface creates an abstraction for all motor
 * controller types, allowing the same code to be used with different
 * motor controller types.
 */
public interface MotorController {
    // Motor control functions

    /**
     * Reset the controller to factory defaults.
     */
    public void reset();

    /**
     * Set the percent output for the motor.
     * @param speed The speed to run the motor at
     */
    public void setPercentOutput(double speed);

    /**
     * Set the closed-loop velocity of the motor.
     * @param rpm Speed in rotations per minute
     */
    public void setVelocity(double rpm);

    /**
     * Set the PID controller to move a distance.
     * @param inches The number of inches to move
     */
    public void setDistance(double inches);

    /**
     * Set brake mode on the controller.
     * @param brake True for brake mode, false for coast mode.
     */
    public void setBrakeMode(boolean brake);

    /**
     * Set whether to invert the motor output.
     * @param inverted Inversion mode
     */
    public void setInverted(boolean inverted);

    /**
     * Follow another motor controller. This controller must be the same type
     * as the current motor controller.
     * @param leader The leader to follow
     * @throws IllegalArgumentException If the leader is not the same type of motor controller.
     */
    public void follow(MotorController leader);

    // Encoder functions

    /**
     * Set the sensor source for the motor.
     */
    public void setSensorSource();

    /**
     * Set the neutral deadband.
     * @param percent The neutral deadband from 0.0 to 1.0
     */
    public void setNeutralDeadband(double percent);

    /**
     * Set whether the encoder is inverted from the motor output.
     * @param inverted Inversion mode
     */
    public void setEncoderInverted(boolean inverted);

    /**
     * Set the status frame period.
     * @param period The number of milliseconds (?) per period
     */
    public void setStatusFramePeriod(int period);

    /**
     * Set the output limits for the controller. All numbers range from -1.0 to 1.0.
     * @param nominalForward Desired base forward output
     * @param nominalReverse Desired base reverse output
     * @param peakForward Maximum forward output
     * @param peakReverse Maximum reverse output
     */
    public void setOutputLimits(double nominalForward, double nominalReverse, double peakForward, double peakReverse);
    
    /**
     * Set PID gains for the PID loop.
     * @param P Percentage gain
     * @param I Integral gain
     * @param D Derivative gain
     * @param F F gain
     */
    public void setPID(double P, double I, double D, double F);

    /**
     * Set PID gains for the PID loop using a Gains object.
     * @param gains PID gains to set
     */
    public default void setPID(Gains gains) {
        setPID(gains.P, gains.I, gains.D, gains.F);
    }

    /**
     * Returns the active trajectory velocity.
     * @return The active trajectory velocity.
     */
    public double getActiveTrajectoryVelocity();

    /**
     * Returns the set sensor velocity.
     * @return The velocity as set previously (?)
     */
    public double getSensorVelocity();

    /**
     * Set motion speed coefficients
     * (TODO: Document exactly what this is)
     * @param cruiseVelocity
     * @param acceleration
     */
    public void setMotionSpeed(double cruiseVelocity, double acceleration);

    /**
     * Returns the current position of the encoder.
     * @return The current position of the encoder, in rotations
     */
    public double getEncoderPosition();
    
    /**
     * Set the position of the encoder.
     * @param position The position to set, in rotations
     */
    public void setEncoderPosition(double position);

    /**
     * Returns the status of the forward limit switch.
     * @return Whether the forward limit switch is pressed
     */
    public boolean getForwardLimit();

    /**
     * Returns the status of the reverse limit switch.
     * @return Whether the reverse limit switch is pressed
     */
    public boolean getReverseLimit();

    /**
     * Type of motor controller, for MotorControllerFactory
     */
    public static enum Type {
        Talon,    /// Talon SRX motor controller
        SparkMax  /// Spark MAX motor controller
    }
}
