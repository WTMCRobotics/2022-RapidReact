package frc.robot.motor;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.K;
import frc.robot.Robot;

public class TalonMotorController implements MotorController {
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
    public void setPercentOutput(double speed) {
        controller.set(ControlMode.PercentOutput, speed);
    }

    @Override
    public void setVelocity(double speed) {
        // convert from RPM to ticks/100 ms
        // 600 * (100 ms) = 1 minute
        controller.set(ControlMode.Velocity, speed * K.encoderRotation / 600);
    }

    @Override
    public void setDistance(double inches) {
        controller.set(ControlMode.MotionMagic, inches * K.encoderRotation / robot.circumference);
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
    public void follow(MotorController leader) {
        if (!(leader instanceof TalonMotorController))
            throw new IllegalArgumentException("Leader must be the same type of motor controller as the follower");
        controller.follow(((TalonMotorController)leader).controller);
    }

    @Override
    public void setSensorSource() {
        controller.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, K.PID_LOOP_IDX, K.TIMEOUT_MS);
    }

    @Override
    public void setNeutralDeadband(double percent) {
        controller.configNeutralDeadband(percent, K.TIMEOUT_MS);
    }

    @Override
    public void setEncoderInverted(boolean inverted) {
        controller.setSensorPhase(inverted);
    }

    @Override
    public void setStatusFramePeriod(int period) {
        controller.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, period, K.TIMEOUT_MS);
        controller.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, period, K.TIMEOUT_MS);
    }

    @Override
    public void setOutputLimits(double nominalForward, double nominalReverse, double peakForward, double peakReverse) {
        controller.configNominalOutputForward(nominalForward, K.TIMEOUT_MS);
        controller.configNominalOutputReverse(nominalReverse, K.TIMEOUT_MS);
        controller.configPeakOutputForward(peakForward, K.TIMEOUT_MS);
        controller.configPeakOutputReverse(peakReverse, K.TIMEOUT_MS);
    }

    @Override
    public void setPID(double P, double I, double D, double F) {
        controller.selectProfileSlot(K.SLOT_IDX, K.PID_LOOP_IDX);
        controller.config_kP(K.SLOT_IDX, P, K.TIMEOUT_MS);
        controller.config_kI(K.SLOT_IDX, I, K.TIMEOUT_MS);
        controller.config_kD(K.SLOT_IDX, D, K.TIMEOUT_MS);
        controller.config_kF(K.SLOT_IDX, F, K.TIMEOUT_MS);
    }

    @Override
    public double getActiveTrajectoryVelocity() {
        return controller.getActiveTrajectoryVelocity() / K.encoderRotation;
    }

    @Override
    public double getSensorVelocity() {
        return controller.getSelectedSensorVelocity() / K.encoderRotation;
    }

    @Override
    public void setMotionSpeed(double cruiseVelocity, double acceleration) {
        controller.configMotionCruiseVelocity(cruiseVelocity, K.TIMEOUT_MS);
        controller.configMotionAcceleration(acceleration, K.TIMEOUT_MS);
    }

    @Override
    public double getEncoderPosition() {
        return controller.getSelectedSensorPosition() / K.encoderRotation;
    }

    @Override
    public void setEncoderPosition(double position) {
        controller.setSelectedSensorPosition(position * K.encoderRotation);
    }
    
}
