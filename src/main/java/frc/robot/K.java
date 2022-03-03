package frc.robot;

import frc.robot.motor.IMotorController;
import frc.robot.motor.IMotorController.Type;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;

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

    /** this should be pulled low on the 2016 Practice Robot */
    public static final int ROBOT_SENSOR_ID = 9;
    /** sensor for when the winch is extended */
    public static final int HANG_SET_SENSOR_ID = 0;
    /** sensor for when the winch is retracted */
    public static final int HANG_DEFAULT_SENSOR_ID = 1;
    /** sensor for when a ball is waiting to be popped up */
    public static final int INTAKE_SENSOR_ID = 2;
    /** sensor for counting balls */
    public static final int INTAKE_COUNTER_SENSOR_ID = 3;

    // Binary Sensors

    /** this should be pulled low on the 2016 Practice Robot */
    public static final DigitalInput ROBOT_SENSOR = new DigitalInput(ROBOT_SENSOR_ID);
    /** sensor for when the winch is extended */ 
    public static final DigitalInput HANG_SET_SENSOR = new DigitalInput(HANG_SET_SENSOR_ID);
    /** sensor for when the winch is retracted */
    public static final DigitalInput HANG_DEFAULT_SENSOR = new DigitalInput(HANG_DEFAULT_SENSOR_ID);
    /** sensor for when a ball is waiting to be popped up */
    public static final DigitalInput INTAKE_SENSOR = new DigitalInput(INTAKE_SENSOR_ID);
    /** sensor for counting balls */
    public static final DigitalInput POPPER_SENSOR = new DigitalInput(INTAKE_COUNTER_SENSOR_ID);

    // ##########################################
    // talon related constants and variables
    // ##########################################

    // can bus IDs. Can be found in Phoenix Tuner
    public static final int LEFT_MASTER_ID = 2;
    public static final IMotorController.Type LEFT_MASTER_TYPE = Type.Talon;
    public static final int LEFT_SLAVE_ID = 3;
    public static final IMotorController.Type LEFT_SLAVE_TYPE = Type.Talon;
    public static final int RIGHT_MASTER_ID = 4;
    public static final IMotorController.Type RIGHT_MASTER_TYPE = Type.Talon;
    public static final int RIGHT_SLAVE_ID = 5;
    public static final IMotorController.Type RIGHT_SLAVE_TYPE = Type.Talon;
    public static final int WINCH_MOTOR_ID = 11;
    public static final IMotorController.Type WINCH_MOTOR_TYPE = Type.Talon;
    public static final int INTAKE_ID = 12;
    public static final IMotorController.Type INTAKE_TYPE = Type.Talon;
    public static final int POPPER_ID = 10;
    public static final IMotorController.Type POPPER_TYPE = Type.Talon;

    /** the number of ticks in a full rotation */
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
    public static final Constraints ROTATIONAL_GAIN_CONSTRAINTS = new Constraints(Double.POSITIVE_INFINITY, 20); // m/s and m/s^2

    // ##########################################
    // intake and popper related constants and variables
    // ##########################################

    /* the maximum number of balls that can be held */
    public static final int MAX_BALLS = 3;

    // the speed of the intake motor. Accepts values between 1 and -1.
    public static final double INTAKE_SPEED_IN = 0.25;
    public static final double INTAKE_SPEED_OUT = -0.25;

    // the speed of the popper motor. Accepts values between 1 and -1.
    public static final double POPPER_SPEED_IN = 0.8;
    public static final double POPPER_SPEED_OUT = -0.2;

    // the Amount of time popper motor should go for in robot cycles.
    public static final int POPPER_TIME_IN = 35;
    public static final int POPPER_TIME_OUT = 10;

    public static final int INTAKE_COUNTER_COUNT_TIME = 3; // the number of cycles that a ball interrupts the sensor for when passing
    public static final int POPPER_COUNTER_JAM_TIME = 20; // the number of cycles that constitutes a popper jam

    // Controller button IDs
    public static final int START = 7; // the mapping of the start button on a xbox controller
    public static final int SELECT = 8; // the mapping of the select button on a xbox controller
    public static final int A_BUTTON = 1; // the mapping of the A button on a xbox controller
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
