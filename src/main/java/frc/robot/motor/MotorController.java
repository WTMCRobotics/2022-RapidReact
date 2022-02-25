package frc.robot.motor;

import frc.robot.Gains;

public interface MotorController {
    // Motor control functions
    public void reset();
    public void setSpeed(double speed);
    public void setDistance(double inches);
    public void setBrakeMode(boolean brake);
    public void setInverted(boolean inverted);

    // Encoder functions
    public void setSensorSource(); // todo
    public void setNeutralDeadband(double percent);
    public void setEncoderInverted(boolean inverted);
    public void setStatusFramePeriod(int period);
    public void setOutputLimits(double nominalForward, double nominalReverse, double peakForward, double peakReverse);
    public void setPID(double P, double I, double D, double F);
    public default void setPID(Gains gains) {
        setPID(gains.P, gains.I, gains.D, gains.F);
    }
    public double getActiveTrajectoryVelocity();
    public double getSensorVelocity();
    public void setMotionSpeed(double cruiseVelocity, double acceleration);
    public double getEncoderPosition();
    public void setEncoderPosition(double position);
}
