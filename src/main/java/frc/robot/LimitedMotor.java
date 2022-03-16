package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import frc.robot.motor.*;

class LimitedMotor {
    // the motor being limited
    private CANSparkMax spark;
    // the encoder that belongs to the above spark
    private RelativeEncoder encoder;
    // PID controller for the spark
    private SparkMaxPIDController pid;

    // the position of the encoder in rotations when it hits the reverse limit switch.
    private double start;
    // the distance in rotations from the forward to the reverse limit switch.
    private final double range;
    // the maximum speed, in RPM, at which it is safe to run into the limit switch
    private final double safeSpeed;

    // Tells us whether we have found where the reverse limit switch is
    private boolean startIsKnown = false;

    // where we want it to go to
    // Where 1 is the forward limit switch and 0 is the reverse limit switch
    private double targetPos;

    /**
     * Creates a new limited motor
     * 
     * @param spark the motor to be limited
     * @param range the distance in rotations from the forward to the reverse limit switch
     * @param safeSpeed the maximum speed, in RPM, at which it is safe to run into the limit switch
     */
    public LimitedMotor(CANSparkMax spark, double range, double safeSpeed) {
        this.spark = spark;
        this.range = range;
        this.safeSpeed = safeSpeed;
        // 4096 is the ticks per revolution of the encoder
        this.encoder = spark.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, K.encoderRotation);
        this.pid = spark.getPIDController();
    }

    /**
     * Creates a new limited motor
     * 
     * @param spark the motor to be limited
     * @param range the distance in rotations from the forward to the reverse limit switch
     * @param safeSpeed the maximum speed, in RPM, at which it is safe to run into the limit switch
     */
    public LimitedMotor(MotorController motorController, double range, double safeSpeed) {
        this(checkSpark(motorController), range, safeSpeed);
    }

    /**
     * this method should be called in robot periodic
     * moves the motor closer to the target position
     */
    public void tick() {
        if (!this.startIsKnown) {
            System.out.println("Finding start");
            this.pid.setReference(this.safeSpeed * -1, CANSparkMax.ControlType.kDutyCycle);
            if (this.spark.getReverseLimitSwitch(Type.kNormallyOpen).isPressed()) {
                System.out.println("Start found");
                this.start = this.encoder.getPosition(); // This is in rotations, but that can be changed.
                this.startIsKnown = true;
            }
        } else {
            System.out.println("Moving");
            this.pid.setReference(this.start + (this.range) * this.targetPos, CANSparkMax.ControlType.kSmartMotion);
        }
    }

    /**
     * @return where we want it to go to
     * Where 1 is the forward limit switch and 0 is the reverse limit switch
     */
    public double getTargetPos() {
        return this.targetPos;
    }

    /**
     * @param targetPos where we want it to go to
     * Where 1 is the forward limit switch and 0 is the reverse limit switch
     */
    public void setTargetPos(double targetPos){
        if (targetPos < 0) {
            this.targetPos = 0;
        } else if (targetPos > 1) {
            this.targetPos = 1;
        } else {
            this.targetPos = targetPos;
        }
    }

    /**
     * Resets the calibration status of the limiter.
     */
    public void reset() {
        this.start = 0;
        this.startIsKnown = false;
    }

    /**
     * Check that a motor controller is a Spark, and return the controller inside.
     * This is necessary because you cannot call this code in a chained constructor.
     * @param motorController The controller to check
     * @return The CANSparkMax instance for the controller
     * @throws IllegalArgumentException If the motor is not a Spark.
     */
    private static CANSparkMax checkSpark(MotorController motorController) {
        if (!(motorController instanceof SparkMotorController)) {
            throw new IllegalArgumentException("Limited motor must be a spark motor controller");
        }
        return ((SparkMotorController)motorController).controller;
    }
}