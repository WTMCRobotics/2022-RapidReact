package frc.robot.motor;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Robot;

public class TalonMotorController implements IMotorController {
    Robot robot;
    TalonSRX controller;

    TalonMotorController(int canID, Robot robot) {
        this.robot = robot;
        controller = new TalonSRX(canID);
    }

    @Override
    public void reset() {
        controller.configFactoryDefault();
    }

    @Override
    public void setSpeed(double speed) {
        controller.set(ControlMode.PercentOutput, speed);
    }

    @Override
    public void setDistance(double inches) {
        controller.set(ControlMode.MotionMagic, inches * Robot.encoderRotation / robot.circumference);
    }

    @Override
    public void setBrakeMode(boolean brake) {
        controller.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
    }

    @Override
    public void setInverted(boolean inverted) {
        controller.setInverted(inverted);
    }

    @Override
    public void follow(IMotorController leader) {
        if (!(leader instanceof TalonMotorController))
            throw new IllegalArgumentException("Leader must be the same type of motor controller as the follower");
        controller.follow(((TalonMotorController)leader).controller);
    }

    @Override
    public void setSensorSource() {
        controller.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Robot.PID_LOOP_IDX, Robot.TIMEOUT_MS);
    }

    @Override
    public void setNeutralDeadband(double percent) {
        controller.configNeutralDeadband(percent, Robot.TIMEOUT_MS);
    }

    @Override
    public void setEncoderInverted(boolean inverted) {
        controller.setSensorPhase(inverted);
    }

    @Override
    public void setStatusFramePeriod(int period) {
        controller.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, period, Robot.TIMEOUT_MS);
        controller.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, period, Robot.TIMEOUT_MS);
    }

    @Override
    public void setOutputLimits(double nominalForward, double nominalReverse, double peakForward, double peakReverse) {
        controller.configNominalOutputForward(nominalForward, Robot.TIMEOUT_MS);
        controller.configNominalOutputReverse(nominalReverse, Robot.TIMEOUT_MS);
        controller.configPeakOutputForward(peakForward, Robot.TIMEOUT_MS);
        controller.configPeakOutputReverse(peakReverse, Robot.TIMEOUT_MS);
    }

    @Override
    public void setPID(double P, double I, double D, double F) {
        controller.selectProfileSlot(Robot.SLOT_IDX, Robot.PID_LOOP_IDX);
        controller.config_kP(Robot.SLOT_IDX, P, Robot.TIMEOUT_MS);
        controller.config_kI(Robot.SLOT_IDX, I, Robot.TIMEOUT_MS);
        controller.config_kD(Robot.SLOT_IDX, D, Robot.TIMEOUT_MS);
        controller.config_kF(Robot.SLOT_IDX, F, Robot.TIMEOUT_MS);
    }

    @Override
    public double getActiveTrajectoryVelocity() {
        return controller.getActiveTrajectoryVelocity();
    }

    @Override
    public double getSensorVelocity() {
        return controller.getSelectedSensorVelocity();
    }

    @Override
    public void setMotionSpeed(double cruiseVelocity, double acceleration) {
        controller.configMotionCruiseVelocity(cruiseVelocity, Robot.TIMEOUT_MS);
        controller.configMotionAcceleration(acceleration, Robot.TIMEOUT_MS);
    }

    @Override
    public double getEncoderPosition() {
        return controller.getSelectedSensorPosition();
    }

    @Override
    public void setEncoderPosition(double position) {
        controller.setSelectedSensorPosition(position);
    }
    
}
