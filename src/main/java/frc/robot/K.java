package frc.robot;

import frc.robot.motor.MotorController;
import frc.robot.motor.MotorController.Type;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;

// The K class contains all of the constants that the robot uses.
// This makes it easier to quickly edit parameters without having to dig
// through the main Robot class.
// All members should be `public static final`, and it is recommended to
// give them ALL_UPPERCASE names.

public final class K {
    /** the width of the robot in inches */
    public static final float robotWidth = 32;
    /** the length of the robot in inches */
    public static final float robotLength = 38;

    // ##########################################
    // Digital IO related constants
    // ##########################################

    // DIO IDs

    /** lift sensor - bottom */
    public static final int LIFT_BOTTOM_SENSOR_ID = 0;
    /** lift sensor - top */
    public static final int LIFT_TOP_SENSOR_ID = 1;
    /** sensor for counting balls */
    public static final int SHOOTER_SENSOR_ID = 2;
    /** Pixy LEDs - red */
    public static final int PIXY_LED_RED_ID = 3;
    /** Pixy LEDs - green */
    public static final int PIXY_LED_GREEN_ID = 4;
    /** Pixy LEDs - blue */
    public static final int PIXY_LED_BLUE_ID = 5;

    // Binary Sensors

    /** lift sensor - bottom */
    public static final DigitalInput LIFT_BOTTOM_SENSOR = new DigitalInput(LIFT_BOTTOM_SENSOR_ID);
    /** lift sensor - top */
    public static final DigitalInput LIFT_TOP_SENSOR = new DigitalInput(LIFT_TOP_SENSOR_ID);
    /** sensor for counting balls */
    public static final DigitalInput SHOOTER_SENSOR = new DigitalInput(SHOOTER_SENSOR_ID);
    /** Pixy LEDs - red */
    public static final DigitalOutput PIXY_LED_RED = new DigitalOutput(PIXY_LED_RED_ID);
    /** Pixy LEDs - green */
    public static final DigitalOutput PIXY_LED_GREEN = new DigitalOutput(PIXY_LED_GREEN_ID);
    /** Pixy LEDs - blue */
    public static final DigitalOutput PIXY_LED_BLUE = new DigitalOutput(PIXY_LED_BLUE_ID);


    // ##########################################
    // talon related constants and variables
    // ##########################################

    // can bus IDs. Can be found in Phoenix Tuner
    public static final int LEFT_MASTER_ID = 25;
    public static final MotorController.Type LEFT_MASTER_TYPE = Type.Talon;
    public static final int LEFT_SLAVE_ID = 26;
    public static final MotorController.Type LEFT_SLAVE_TYPE = Type.Talon;
    public static final int RIGHT_MASTER_ID = 27;
    public static final MotorController.Type RIGHT_MASTER_TYPE = Type.Talon;
    public static final int RIGHT_SLAVE_ID = 28;
    public static final MotorController.Type RIGHT_SLAVE_TYPE = Type.Talon;
    public static final int INTAKE_ID = 8;
    public static final MotorController.Type INTAKE_TYPE = Type.SparkMax;
    public static final int LIFT_ID = 33;
    public static final MotorController.Type LIFT_TYPE = Type.Talon;
    public static final int TURRET_ROTATION_ID = 5;
    public static final MotorController.Type TURRET_ROTATION_TYPE = Type.SparkMax;
    public static final int TURRET_SHOOTER_ID = 4;
    public static final MotorController.Type TURRET_SHOOTER_TYPE = Type.SparkMax;
    public static final int TURRET_INTAKE_ID = 3;
    public static final MotorController.Type TURRET_INTAKE_TYPE = Type.SparkMax;
    public static final int TURRET_HOOD_ID = 2;
    public static final MotorController.Type TURRET_HOOD_TYPE = Type.SparkMax;

    /** the number of ticks in a full rotation (Talon only - Spark stores it onboard) */
    public static final int encoderRotation = 4096;

    // talon config

    /** Which PID slot to pull gains from */
    public static final int SLOT_IDX = 0;
    /** Which PID loop to pull gains from */
    public static final int PID_LOOP_IDX = 0;
    /** amount of time in ms to wait for confirmation */
    public static final int TIMEOUT_MS = 30;

    // PID constants
    public static final Gains PRACTICE_ROBOT_GAINS = new Gains(0.2, 0.00035, 1.5, 0.2, 0, 1.0);
    public static final Gains COMPETITION_ROBOT_GAINS = new Gains(0.35, 0.00001, 100, 0.2, 0, 1.0);
    public static final Gains PRACTICE_ROTATION_GAINS = new Gains(0.004, 0.003, 0.001, 0.0, 0, 0.0);
    public static final Gains COMPETITION_ROTATION_GAINS = new Gains(0.06, 0.003, 0.001, 0.0, 0, 0.0);
    public static final Gains TURRET_ROTATION_GAINS = new Gains(0.002, 0, 0, 0, 0, 1.0);
    public static final Constraints ROTATIONAL_GAIN_CONSTRAINTS = new Constraints(Double.POSITIVE_INFINITY, 20); // m/s and m/s^2

    // ##########################################
    // intake and popper related constants and variables
    // ##########################################

    /* the maximum number of balls that can be held */
    public static final int MAX_BALLS = 3;

    // the speed of the intake motor. Accepts values between 1 and -1.
    public static final double INTAKE_SPEED_IN = 1;
    public static final double INTAKE_SPEED_OUT = -1;

    // the speed of the turret intake motor. Accepts values between 0 and 1.
    public static final double TURRET_INTAKE_SPEED = 0.5;

    // the speed of the lift motor. Accepts values between 0 and 1.
    public static final double LIFT_SPEED = 0.1;

    public static final int INTAKE_COUNTER_COUNT_TIME = 3; // the number of cycles that a ball interrupts the sensor for when passing
    public static final int POPPER_COUNTER_JAM_TIME = 20; // the number of cycles that constitutes a popper jam

    public static final double TURRET_ROTATION_ANGLE = 0.75;
    public static final double TURRET_ROTATION_SPEED = 0.25;

    // Controller button IDs
    public static final int START = 7; // the mapping of the start button on a xbox controller
    public static final int SELECT = 8; // the mapping of the select button on a xbox controller
    public static final int A_BUTTON = 1; // the mapping of the A button on a xbox controller
    public static final int B_BUTTON = 2; // the mapping of the A button on a xbox controller
    public static final int X_BUTTON = 3; // the mapping of the A button on a xbox controller
    public static final int Y_BUTTON = 4; // the mapping of the A button on a xbox controller
    public static final int R_STICK = 10; // the mapping of the right shoulder on a xbox controller
    public static final int L_STICK = 9; // the mapping of the left shoulder on a xbox controller
    public static final int R_SHOULDER = 6; // the mapping of the right shoulder on a xbox controller
    public static final int L_SHOULDER = 5; // the mapping of the left shoulder on a xbox controller

    // ##########################################
    // Pneumatics related constants
    // ##########################################

    public static final int PCM_DRAWBRIDGE_IN = 1;
    public static final int PCM_DRAWBRIDGE_OUT = 0;
    public static final int PCM_RATCHET = 2;

    public static final String[] galacticSearchNames = {"Red A", "Blue A", "Red B", "Blue B"};
}
