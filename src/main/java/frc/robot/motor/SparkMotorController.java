package frc.robot.motor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import frc.robot.Robot;

public class SparkMotorController implements MotorController {
    public CANSparkMax controller;
    RelativeEncoder encoder;
    SparkMaxPIDController pid;
    Robot robot;

    SparkMotorController(int canID, Robot robot) {
        controller = new CANSparkMax(canID, MotorType.kBrushed);
        encoder = controller.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096); // 4096 = number of ticks per revolution
        pid = controller.getPIDController();
    }

    @Override
    public void reset() {
        controller.restoreFactoryDefaults();
    }

    @Override
    public void setPercentOutput(double speed) {
        controller.set(speed);
    }

    @Override
    public void setVelocity(double speed) {
        pid.setReference(speed, CANSparkMax.ControlType.kSmartVelocity);
    }

    @Override
    public void setDistance(double inches) {
        pid.setReference(inches / robot.circumference, ControlType.kSmartMotion);
    }

    @Override
    public void setBrakeMode(boolean brake) {
        controller.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public void setInverted(boolean inverted) {
        controller.setInverted(inverted);
    }

    @Override
    public void follow(MotorController leader) {
        if (!(leader instanceof SparkMotorController))
            throw new IllegalArgumentException("Leader must be the same type of motor controller as the follower");
        controller.follow(((SparkMotorController)leader).controller);
    }

    @Override
    public void setSensorSource() {
        // Do nothing
    }

    @Override
    public void setNeutralDeadband(double percent) {
        // Only used for PWM (?)
    }

    @Override
    public void setEncoderInverted(boolean inverted) {
        encoder.setInverted(inverted);
    }

    @Override
    public void setStatusFramePeriod(int period) {
        controller.setPeriodicFramePeriod(PeriodicFrame.kStatus0, period);
        controller.setPeriodicFramePeriod(PeriodicFrame.kStatus1, period);
        controller.setPeriodicFramePeriod(PeriodicFrame.kStatus2, period);
        controller.setPeriodicFramePeriod(PeriodicFrame.kStatus3, period);
    }

    @Override
    public void setOutputLimits(double nominalForward, double nominalReverse, double peakForward, double peakReverse) {
        pid.setOutputRange(peakReverse, peakForward);
        // No need for nominal range
    }

    @Override
    public void setPID(double P, double I, double D, double F) {
        pid.setP(P);
        pid.setI(I);
        pid.setD(D);
        pid.setFF(F);
    }

    @Override
    public double getActiveTrajectoryVelocity() {
        // TODO: figure this out
        return encoder.getVelocity();
    }

    @Override
    public double getSensorVelocity() {
        return encoder.getVelocity();
    }

    @Override
    public void setMotionSpeed(double cruiseVelocity, double acceleration) {
        pid.setSmartMotionMaxVelocity(cruiseVelocity, 0);
        pid.setSmartMotionMaxAccel(acceleration, 0);
    }

    @Override
    public double getEncoderPosition() {
        return encoder.getPosition();
    }

    @Override
    public void setEncoderPosition(double position) {
        encoder.setPosition(position);
    }

    @Override
    public boolean getForwardLimit() {
        return controller.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).isPressed();
    }

    @Override
    public boolean getReverseLimit() {
        return controller.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).isPressed();
    }
    
}
