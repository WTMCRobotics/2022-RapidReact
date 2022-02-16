/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;
import java.util.StringTokenizer;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.MathUtil;
import frc.robot.auton.*;
import io.github.pseudoresonance.pixy2api.*;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import io.github.pseudoresonance.pixy2api.links.SPILink;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

    // ##########################################
    // auton related constants and variables
    // ##########################################

    /** dropdown for choosing which challenge */
    private final SendableChooser<Challenge> CHALLENGE_CHOOSER = new SendableChooser<>();
    /** the challenge selected by the user */
    private Challenge selectedChallenge;

    /** dropdown for choosing which path */
    private final SendableChooser<Path> PATH_CHOOSER = new SendableChooser<>();
    /** the path selected by the user */
    private Path selectedPath;

    /** a list of instructions to follow */
    ArrayList<Instruction> autonInstructions = new ArrayList<Instruction>();

    /** the width of the robot in inches */
    public static final float robotWidth = 32;
    /** the length of the robot in inches */
    public static final float robotLength = 38;

    // ##########################################
    // Digital IO related constants
    // ##########################################

    // DIO IDs

    /** this should be pulled low on the 2016 Practice Robot */
    static final int ROBOT_SENSOR_ID = 9;
    /** sensor for when the winch is extended */
    static final int HANG_SET_SENSOR_ID = 0;
    /** sensor for when the winch is retracted */
    static final int HANG_DEFAULT_SENSOR_ID = 1;
    /** sensor for when a ball is waiting to be popped up */
    static final int INTAKE_SENSOR_ID = 2;
    /** sensor for counting balls */
    static final int INTAKE_COUNTER_SENSOR_ID = 3;

    // Binary Sensors

    /** this should be pulled low on the 2016 Practice Robot */
    static final DigitalInput ROBOT_SENSOR = new DigitalInput(ROBOT_SENSOR_ID);
    /** sensor for when the winch is extended */ 
    static final DigitalInput HANG_SET_SENSOR = new DigitalInput(HANG_SET_SENSOR_ID);
    /** sensor for when the winch is retracted */
    static final DigitalInput HANG_DEFAULT_SENSOR = new DigitalInput(HANG_DEFAULT_SENSOR_ID);
    /** sensor for when a ball is waiting to be popped up */
    static final DigitalInput INTAKE_SENSOR = new DigitalInput(INTAKE_SENSOR_ID);
    /** sensor for counting balls */
    static final DigitalInput POPPER_SENSOR = new DigitalInput(INTAKE_COUNTER_SENSOR_ID);

    // ##########################################
    // talon related constants and variables
    // ##########################################

    // can bus IDs. Can be found in Phoenix Tuner
    static final int LEFT_MASTER_ID = 2;
    static final int LEFT_SLAVE_ID = 3;
    static final int RIGHT_MASTER_ID = 4;
    static final int RIGHT_SLAVE_ID = 5;
    static final int WINCH_MOTOR_ID = 11;
    static final int INTAKE_ID = 12;
    static final int POPPER_ID = 10;

    // creates objects for the talons
    public TalonSRX leftMaster = new TalonSRX(LEFT_MASTER_ID);
    TalonSRX leftSlave = new TalonSRX(LEFT_SLAVE_ID);
    public TalonSRX rightMaster = new TalonSRX(RIGHT_MASTER_ID);
    TalonSRX rightSlave = new TalonSRX(RIGHT_SLAVE_ID);
    TalonSRX hangMotor = new TalonSRX(WINCH_MOTOR_ID);
    TalonSRX intake = new TalonSRX(INTAKE_ID);
    TalonSRX popper = new TalonSRX(POPPER_ID);

    /** the number of ticks in a full rotation */
    static final int encoderRotation = 4096;

    // talon config

    /** Which PID slot to pull gains from */
    public static final int SLOT_IDX = 0;
    /** Which PID loop to pull gains from */
    public static final int PID_LOOP_IDX = 0;
    /** amount of time in ms to wait for confirmation */
    public static final int TIMEOUT_MS = 30;

    // ##########################################
    // drivetrain and pid related constants and variables
    // ##########################################

    // the object that is the navX-MXP
    public AHRS gyro = new AHRS(Port.kMXP);
    static final Gains PRACTICE_ROTATION_GAINS = new Gains(0.004, 0.003, 0.001, 0.0, 0, 0.0);
    static final Gains COMPETITION_ROTATION_GAINS = new Gains(0.06, 0.003, 0.001, 0.0, 0, 0.0);
    static Gains rotationGains;

    static final Constraints ROTATIONAL_GAIN_CONSTRAINTS = new Constraints(Double.POSITIVE_INFINITY, 20); // m/s and m/s^2
    ProfiledPIDController rotationPID;

    /** true if test is done */
    boolean testDone = false;
    /** true = move, false = rotate */
    boolean testRotation = false;
    /** number of inches to move in test */
    double testMoveDistance = 24;
    /** number of degrees to rotate in test */
    double testTurnAmount = 90;

    /** The maximum distance from the destination considered close enough */
    private static final double distanceMarginOfError = 2;

    /** The margin of error for angles when turning in auton */
    private static final double angleMarginOfError = 5;

    /** true if ROBOT_SENSOR is pulled low */
    boolean isPracticeRobot;
    /** the circumference of the drive wheels */
    double circumference;

    /** weather to use Arcade (true) or tank (false) style controls */
    boolean ArcadeDrive = true;

    static final Gains PRACTICE_ROBOT_GAINS = new Gains(0.2, 0.00035, 1.5, 0.2, 0, 1.0);
    static final Gains COMPETITION_ROBOT_GAINS = new Gains(0.35, 0.00001, 100, 0.2, 0, 1.0);
    static Gains gains; // used for drivetrain motion magic when moving and is ste to
                        // PRACTICE_ROBOT_GAINS or COMPETITION_ROBOT_GAINS

    // ##########################################
    // intake and popper related constants and variables
    // ##########################################

    /* the maximum number of balls that can be held */
    static final int MAX_BALLS = 3;

    // the speed of the intake motor. Accepts values between 1 and -1.
    static final double INTAKE_SPEED_IN = 0.25;
    static final double INTAKE_SPEED_OUT = -0.25;

    // the speed of the popper motor. Accepts values between 1 and -1.
    static final double POPPER_SPEED_IN = 0.8;
    static final double POPPER_SPEED_OUT = -0.2;

    // the Amount of time popper motor should go for in robot cycles.
    static final int POPPER_TIME_IN = 35;
    static final int POPPER_TIME_OUT = 10;

    // the amount of time in robot cycles that this will move
    int popperInTime = 0;
    int popperOutTime = 0;

    int popperCounterTime; // the number of cycles that the counter sensor has been interrupted for
    static final int INTAKE_COUNTER_COUNT_TIME = 3; // the number of cycles that a ball interrupts the sensor for when passing
    int intakeTime; // the number of cycles that the counter sensor has bean interrupted for
    static final int POPPER_COUNTER_JAM_TIME = 20; // the number of cycles that constitutes a popper jam
    int ballsStored = 0; // the number of balls in the robot

    // ##########################################
    // Drawbridge and hang related constants and variables
    // ##########################################

    // declares objects for the TwoStateMotor class
    TwoStateMotor hang = new RatchetMotor(0.5, -0.1, hangMotor, HANG_DEFAULT_SENSOR, HANG_SET_SENSOR);;

    // ##########################################
    // Controller related constants and variables
    // ##########################################

    XboxController xboxController = new XboxController(0); // driver
    double leftjoyY; // y-axis of the left joystick on the driver's controller
    double rightjoyY; // y-axis of the right joystick on the driver's controller
    double leftjoyX; // x-axis of the left joystick on the driver's controller
    double rightjoyX; // x-axis of the right joystick on the driver's controller

    static final int START = 7; // the mapping of the start button on a xbox controller
    static final int SELECT = 8; // the mapping of the select button on a xbox controller
    static final int A_BUTTON = 1; // the mapping of the A button on a xbox controller
    static final int R_STICK = 10; // the mapping of the right shoulder on a xbox controller
    static final int L_STICK = 9; // the mapping of the left shoulder on a xbox controller
    static final int R_SHOULDER = 6; // the mapping of the right shoulder on a xbox controller
    static final int L_SHOULDER = 5; // the mapping of the left shoulder on a xbox controller

    // these variables should be updated in teleopPeriodic()
    boolean arcadeButton; // true if the button that selects arcade mode is pressed
    boolean tankButton; // true if the button that selects tank mode is pressed
    boolean drawbridgeButton; // true if the button that lowers the drawbridge is pressed
    boolean hangButton; // true if the button that extends the hang mechanism is pressed
    boolean intakeInButton; // true if the button that intakes is pressed
    boolean intakeOutButton; // true if the button that runs the intake in reverse is pressed
    boolean popperInButton; // true if the button that runs the popper is pressed
    boolean popperOutButton; // true if the button that reverses the popper is pressed

    // ##########################################
    // Pneumatics related constants and variables
    // ##########################################

    static final int PCM_DRAWBRIDGE_IN = 1;
    static final int PCM_DRAWBRIDGE_OUT = 0;

    static final int PCM_RATCHET = 2;

    Compressor compressor = null;

    DoubleSolenoid drawbridgeSol = new DoubleSolenoid(1, PneumaticsModuleType.CTREPCM, PCM_DRAWBRIDGE_IN, PCM_DRAWBRIDGE_OUT);
    Solenoid hangSol = new Solenoid(1, PneumaticsModuleType.CTREPCM, PCM_RATCHET);

    Pixy2 pixy;
    GalacticSearchMode galacticSearch;
    GalacticSearchMode[] galacticSearchResults = new GalacticSearchMode[20];
    int galacticSearchNext = 0;
    static final String[] galacticSearchNames = {"Red A", "Blue A", "Red B", "Blue B"};

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        System.out.println("starting robotInit()");

        compressor = new Compressor(1, PneumaticsModuleType.CTREPCM);

        CHALLENGE_CHOOSER.addOption("Galactic Search", Challenge.GALACTIC_SEARCH);
        CHALLENGE_CHOOSER.addOption("AutoNav", Challenge.AUTONAV);

        SmartDashboard.putData("Challenge", CHALLENGE_CHOOSER);

        PATH_CHOOSER.addOption("Barrel Racing", Path.BARREL_RACING);
        PATH_CHOOSER.addOption("Bounce", Path.BOUNCE);
        PATH_CHOOSER.addOption("Slalom", Path.SLALOM);

        SmartDashboard.putData("Path", PATH_CHOOSER);

        isPracticeRobot = !ROBOT_SENSOR.get();
        if (isPracticeRobot) {
            circumference = 6 * Math.PI;
            gains = PRACTICE_ROBOT_GAINS;
            rotationGains = PRACTICE_ROTATION_GAINS;
            System.out.println("using 6 inch wheels");
        } else {
            circumference = 8 * Math.PI;
            gains = COMPETITION_ROBOT_GAINS;
            rotationGains = COMPETITION_ROTATION_GAINS;
            System.out.println("using 8 inch wheels");
        }

        rotationPID = new ProfiledPIDController(rotationGains.P, rotationGains.I, rotationGains.D, ROTATIONAL_GAIN_CONSTRAINTS);

        initializeTalon(leftMaster, NeutralMode.Brake, false);
        initializeTalon(leftSlave, NeutralMode.Brake, false);
        initializeTalon(rightMaster, NeutralMode.Brake, true);
        initializeTalon(rightSlave, NeutralMode.Brake, true);
        initializeTalon(hangMotor, NeutralMode.Brake, false);
        initializeTalon(intake, NeutralMode.Coast, false);
        initializeTalon(popper, NeutralMode.Coast, false);

        initializeMotionMagicMaster(rightMaster);
        initializeMotionMagicMaster(leftMaster);

        rightSlave.set(ControlMode.Follower, RIGHT_MASTER_ID);
        leftSlave.set(ControlMode.Follower, LEFT_MASTER_ID);
        gyro.reset();

        pixy = Pixy2.createInstance(new SPILink());
        pixy.init();
    }

    public void initializeTalon(TalonSRX talon, NeutralMode neutralMode, boolean inverted) {
        /* Ensure motor output is neutral during init */
        talon.set(ControlMode.PercentOutput, 0);

        /* Factory Default all hardware to prevent unexpected behavior */
        talon.configFactoryDefault();

        /* Set Neutral mode */
        talon.setNeutralMode(neutralMode);

        /* Configure output direction */
        talon.setInverted(inverted);
    }

    public void initializeMotionMagicMaster(TalonSRX masterTalon) {
        /* Factory default hardware to prevent unexpected behavior */
        masterTalon.configFactoryDefault();

        /* Configure Sensor Source for Primary PID */
        masterTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, PID_LOOP_IDX, TIMEOUT_MS);

        /*
         * set deadband to super small 0.001 (0.1 %). The default deadband is 0.04 (4 %)
         */
        masterTalon.configNeutralDeadband(0.001, TIMEOUT_MS);

        /**
         * Configure Talon SRX Output and Sensor direction accordingly Invert Motor to
         * have green LEDs when driving Talon Forward / Requesting Positive Output Phase
         * sensor to have positive increment when driving Talon Forward (Green LED)
         */
        masterTalon.setSensorPhase(false);

        /* Set relevant frame periods to be at least as fast as periodic rate */
        masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, TIMEOUT_MS);
        masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, TIMEOUT_MS);

        /* Set the peak and nominal outputs */
        masterTalon.configNominalOutputForward(0, TIMEOUT_MS);
        masterTalon.configNominalOutputReverse(0, TIMEOUT_MS);
        masterTalon.configPeakOutputForward(1, TIMEOUT_MS);
        masterTalon.configPeakOutputReverse(-1, TIMEOUT_MS);

        /* Set Motion Magic gains in slot0 - see documentation */
        masterTalon.selectProfileSlot(SLOT_IDX, PID_LOOP_IDX);
        masterTalon.config_kF(SLOT_IDX, gains.F, TIMEOUT_MS);
        masterTalon.config_kP(SLOT_IDX, gains.P, TIMEOUT_MS);
        masterTalon.config_kI(SLOT_IDX, gains.I, TIMEOUT_MS);
        masterTalon.config_kD(SLOT_IDX, gains.D, TIMEOUT_MS);

        /* Set acceleration and vcruise velocity - see documentation */
        masterTalon.configMotionCruiseVelocity(15000, TIMEOUT_MS);
        masterTalon.configMotionAcceleration(6000, TIMEOUT_MS);

        /* Zero the sensor once on robot boot up */
        masterTalon.setSelectedSensorPosition(0, PID_LOOP_IDX, TIMEOUT_MS);
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like diagnostics that you want ran during disabled, autonomous,
     * teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        hang.tick();

        SmartDashboard.putNumber("Balls Stored", ballsStored);
    }

    @Override
    public void disabledInit() {
        SmartDashboard.putNumber("Proportion", gains.P);
        SmartDashboard.putNumber("Integral", gains.I);
        SmartDashboard.putNumber("Derivative", gains.D);
        SmartDashboard.putNumber("rotationProportion", rotationGains.P);
        SmartDashboard.putNumber("rotationIntegral", rotationGains.I);
        SmartDashboard.putNumber("rotationDerivative", rotationGains.D);

        SmartDashboard.putBoolean("rotate", testRotation);
        SmartDashboard.putNumber("moveDistance", testMoveDistance);
        SmartDashboard.putNumber("turnAmount", testTurnAmount);
    }

    @Override
    public void disabledPeriodic() {
        gains.P = SmartDashboard.getNumber("Proportion", gains.P);
        gains.I = SmartDashboard.getNumber("Integral", gains.I);
        gains.D = SmartDashboard.getNumber("Derivative", gains.D);
        rotationGains.P = SmartDashboard.getNumber("rotationProportion", rotationGains.P);
        rotationGains.I = SmartDashboard.getNumber("rotationIntegral", rotationGains.I);
        rotationGains.D = SmartDashboard.getNumber("rotationDerivative", rotationGains.D);

        testRotation = SmartDashboard.getBoolean("rotate", testRotation);
        testMoveDistance = SmartDashboard.getNumber("moveDistance", testMoveDistance);
        testTurnAmount = SmartDashboard.getNumber("turnAmount", testTurnAmount);

        try {
            selectedChallenge = CHALLENGE_CHOOSER.getSelected();
            if (selectedChallenge == null) {
                throw new NullPointerException("selectedChallenge can't be null");
            }
            if (selectedChallenge == Challenge.AUTONAV) {
                selectedPath = PATH_CHOOSER.getSelected();
                if (selectedPath == null) {
                    throw new NullPointerException("selectedPath can't be null");
                }
            }
            pixy.setLamp(selectedChallenge == Challenge.GALACTIC_SEARCH ? (byte)1 : (byte)0, (byte)0);

            // this line will run only if the other lines didn't crash
            SmartDashboard.putBoolean("Ready", true);
        } catch (NullPointerException e) {
            DriverStation.reportError("auton path not configured!", true);
            SmartDashboard.putBoolean("Ready", false);
        }

        Pixy2CCC ccc = pixy.getCCC();
        ccc.getBlocks();
        ArrayList<Pixy2CCC.Block> blocks = ccc.getBlockCache();
        GalacticSearchMode mode;
        if (blocks.size() > 0) {
            Pixy2CCC.Block largest = null;
            int area_max = 0;
            for (Pixy2CCC.Block b : blocks) {
                int area = b.getWidth() * b.getHeight();
                if (area > area_max) {
                    largest = b;
                    area_max = area;
                }
            }
            if (area_max < 250) mode = GalacticSearchMode.BlueB;
            else if (Math.abs(largest.getX() + (largest.getWidth() / 2) - (pixy.getFrameWidth() / 2)) < 45)
                mode = GalacticSearchMode.RedB;
            else mode = GalacticSearchMode.RedA;
        } else mode = GalacticSearchMode.BlueA;
        galacticSearchResults[galacticSearchNext++] = mode;
        if (galacticSearchNext >= 20) {
            int[] values = {0, 0, 0, 0};
            for (int i = 0; i < 20; i++) values[galacticSearchResults[i].ordinal()]++;
            int max = 0, maxn = 0;
            for (int i = 0; i < 4; i++) if (values[i] > max) {max = values[i]; maxn = i;}
            galacticSearch = GalacticSearchMode.values()[maxn];
            galacticSearchNext = 0;
            SmartDashboard.putString("GalacticSearch", galacticSearchNames[maxn]);
        }
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable chooser
     * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
     * remove all of the chooser code and uncomment the getString line to get the
     * auto name from the text box below the gyro
     *
     * <p>
     * You can add additional auto modes by adding additional comparisons to the
     * STARTING_POSITION structure below with additional strings. If using the
     * SendableChooser make sure to add them to the chooser code above as well.
     */
    @Override
    public void autonomousInit() {
        autonInstructions.clear();
        resetEncoders();
        gyro.reset();
        rotationPID = new ProfiledPIDController(rotationGains.P, rotationGains.I, rotationGains.D, ROTATIONAL_GAIN_CONSTRAINTS);

        switch (selectedChallenge) {
            case GALACTIC_SEARCH:
                switch (galacticSearch) {
                case BlueA:
                    // Blue A: (B1)
                    autonInstructions.add(new TurnDeg(30.964)); // - Turn 30.964 deg
                    autonInstructions.add(new MoveInch(14.577 * 12)); // - Move 14.577 ft
                    // - Collect 1
                    autonInstructions.add(new TurnDeg(-102.529)); // - Turn -102.529 deg
                    autonInstructions.add(new MoveInch(7.906 * 12)); // - Move 7.906 ft
                    // - Collect 1
                    autonInstructions.add(new TurnDeg(135)); // - Turn 135 deg
                    autonInstructions.add(new MoveInch(12 * 12)); // - Move >11.180 ft (collect 1)
                    break;
                case BlueB:
                    // Blue B: (D1)
                    autonInstructions.add(new MoveInch(12.5 * 12));
                    // - Collect 1
                    autonInstructions.add(new TurnDeg(45));
                    autonInstructions.add(new MoveInch(7.071 * 12));
                    // - Collect 1
                    autonInstructions.add(new TurnDeg(90));
                    autonInstructions.add(new MoveInch(7.071 * 12));
                    // - Collect 1
                    autonInstructions.add(new TurnDeg(-45));
                    autonInstructions.add(new MoveInch(5 * 12));
                    break;
                case RedA:
                    // Red A: (B1)
                    autonInstructions.add(new TurnDeg(26.565));
                    autonInstructions.add(new MoveInch(11.180 * 12));//(collect 1)
                    // - Collect 1
                    autonInstructions.add(new TurnDeg(-81.870));
                    autonInstructions.add(new MoveInch(7.906 * 12));
                    // - Collect 1
                    autonInstructions.add(new TurnDeg(71.565));
                    autonInstructions.add(new MoveInch(15 * 12));
                    break;
                case RedB:
                    // Red B: (B1)
                    autonInstructions.add(new MoveInch(5 * 12));
                    // - Collect 1
                    autonInstructions.add(new TurnDeg(45));
                    autonInstructions.add(new MoveInch(7.071 * 12));
                    // - Collect 1
                    autonInstructions.add(new TurnDeg(-90));
                    autonInstructions.add(new MoveInch(7.071 * 12));
                    // - Collect 1
                    autonInstructions.add(new TurnDeg(45));
                    autonInstructions.add(new MoveInch(12.5 * 12));
                    break;
                default:
                    throw new Error("invalid value of \"galacticSearch\"");
                }
                
                break;
            case AUTONAV:
                switch (selectedPath) {
                    case BARREL_RACING:
                        autonInstructions.add(new MoveInch(135+ Robot.robotLength / 2));
                        autonInstructions.add(new TurnDeg(-60)); // c6.5
                        autonInstructions.add(new MoveInch(-90));
                        autonInstructions.add(new TurnDeg(-60)); // e5
                        autonInstructions.add(new MoveInch(90));
                        autonInstructions.add(new TurnDeg(-60)); // c3.5
                        autonInstructions.add(new MoveInch(-165));
                        autonInstructions.add(new TurnDeg(90)); // c9
                        autonInstructions.add(new MoveInch(60));
                        autonInstructions.add(new TurnDeg(-90)); // a9
                        autonInstructions.add(new MoveInch(90));
                        autonInstructions.add(new TurnDeg(45)); // a6
                        autonInstructions.add(new MoveInch(120 * Math.sqrt(2)));
                        autonInstructions.add(new TurnDeg(-45)); // e6
                        autonInstructions.add(new MoveInch(-30));
                        autonInstructions.add(new TurnDeg(90)); // e11
                        autonInstructions.add(new MoveInch(60));
                        autonInstructions.add(new TurnDeg(-90)); // c11
                        autonInstructions.add(new MoveInch(270));
                        break;
                    case SLALOM:
                        autonInstructions.add(new MoveInch(Robot.robotLength / 2));
                        autonInstructions.add(new TurnDeg(-45)); // e2
                        autonInstructions.add(new MoveInch(60 * Math.sqrt(2)));
                        autonInstructions.add(new TurnDeg(45)); // c4
                        autonInstructions.add(new MoveInch(120));
                        autonInstructions.add(new TurnDeg(45)); // c8
                        autonInstructions.add(new MoveInch(60 * Math.sqrt(2)));
                        autonInstructions.add(new TurnDeg(-90)); // e10
                        autonInstructions.add(new MoveInch(30 * Math.sqrt(2)));
                        autonInstructions.add(new TurnDeg(-90)); // d11
                        autonInstructions.add(new MoveInch(30 * Math.sqrt(2))); 
                        autonInstructions.add(new TurnDeg(90)); // c10
                        autonInstructions.add(new MoveInch(60 * Math.sqrt(2))); 
                        autonInstructions.add(new TurnDeg(45)); // e8
                        autonInstructions.add(new MoveInch(120));
                        autonInstructions.add(new TurnDeg(45)); // e4
                        autonInstructions.add(new MoveInch(60 * Math.sqrt(2))); 
                        break;
                    case BOUNCE:
                        autonInstructions.add(new MoveInch(30 + Robot.robotLength / 2));
                        autonInstructions.add(new TurnDeg(-90)); // c3
                        autonInstructions.add(new MoveInch(60 - Robot.robotLength / 2));
                        //a3
                        autonInstructions.add(new MoveInch(-60 + Robot.robotLength / 2)); 
                        autonInstructions.add(new TurnDeg(-45)); // c3
                        autonInstructions.add(new MoveInch(-60 * Math.sqrt(2)));
                        autonInstructions.add(new TurnDeg(90)); // e5
                        autonInstructions.add(new MoveInch(30 * Math.sqrt(2)));
                        autonInstructions.add(new TurnDeg(-45)); // d6
                        autonInstructions.add(new MoveInch(90 - Robot.robotLength / 2));
                        // a6
                        autonInstructions.add(new MoveInch(-120 + Robot.robotLength / 2));
                        autonInstructions.add(new TurnDeg(90)); // e6
                        autonInstructions.add(new MoveInch(90));
                        autonInstructions.add(new TurnDeg(-90)); // e9
                        autonInstructions.add(new MoveInch(120 - Robot.robotLength / 2));
                        // a9
                        autonInstructions.add(new MoveInch(-30 + Robot.robotLength / 2));
                        autonInstructions.add(new TurnDeg(-45)); // b9
                        autonInstructions.add(new MoveInch(-60));
                        break;
                }
                break;
            default:
                throw new Error("no auton mode selected");
        }
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        switch (selectedChallenge) {
            case GALACTIC_SEARCH:
                handlePopper(true);
                if (ballsStored < MAX_BALLS) {
                    intake.set(ControlMode.PercentOutput, INTAKE_SPEED_IN);
                } else {
                    intake.set(ControlMode.PercentOutput, INTAKE_SPEED_OUT);
                }

            case AUTONAV:
            
                while (!autonInstructions.isEmpty() && autonInstructions.get(0).doit(this)) {
                    autonInstructions.remove(0);
                }
                break;
            default:
                throw new Error("unknown auton mode");
        }
    }

    /**
     * This function is called periodically during teleop.
     */
    @Override
    public void teleopInit() {
        pixy.setLamp((byte)0, (byte)0);
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {

        // this code updates the controller variables to the correct value at the
        // beginning of teleopPeriodic()
        leftjoyY = xboxController.getLeftY();
        rightjoyY = xboxController.getRightY();
        leftjoyX = xboxController.getLeftX();
        rightjoyX = xboxController.getRightX();
        arcadeButton = xboxController.getRawButton(L_STICK);
        tankButton = xboxController.getRawButton(R_STICK);
        drawbridgeButton = xboxController.getRawButton(A_BUTTON);
        intakeInButton = 0.1 < xboxController.getLeftTriggerAxis();
        intakeOutButton = 0.1 < xboxController.getRightTriggerAxis();
        popperInButton = xboxController.getRawButton(L_SHOULDER);
        popperOutButton = xboxController.getRawButton(R_SHOULDER);
        hangButton = false;

        setPistonExtended(drawbridgeSol, drawbridgeButton);

        hang.set(hangButton);
        setPistonExtended(hangSol, hangButton);

        if (arcadeButton) {
            ArcadeDrive = true;
        }
        if (tankButton) {
            ArcadeDrive = false;
        }

        // this code sets the motors to the correct speed based on driver input
        if (ArcadeDrive) {
            double x = rightjoyX;
            double y = leftjoyY;
            leftMaster.set(ControlMode.PercentOutput, (y * (2 - Math.abs(x)) - x * (2 - Math.abs(y))) / 2);
            rightMaster.set(ControlMode.PercentOutput, (y * (2 - Math.abs(x)) + x * (2 - Math.abs(y))) / 2);
        } else {
            leftMaster.set(ControlMode.PercentOutput, leftjoyY);
            rightMaster.set(ControlMode.PercentOutput, rightjoyY);
        }

        if (intakeInButton && ballsStored >= MAX_BALLS) {
            xboxController.setRumble(RumbleType.kLeftRumble, 1.0);
            xboxController.setRumble(RumbleType.kRightRumble, 1.0);
        } else {
            xboxController.setRumble(RumbleType.kLeftRumble, 0.0);
            xboxController.setRumble(RumbleType.kRightRumble, 0.0);
        }

        // this code handles intake
        if (intakeInButton) {
            intake.set(ControlMode.PercentOutput, INTAKE_SPEED_IN);
        } else if (intakeOutButton || ballsStored >= MAX_BALLS) {
            intake.set(ControlMode.PercentOutput, INTAKE_SPEED_OUT);
        } else {
            intake.set(ControlMode.PercentOutput, 0);
        }

        // this code handles the popper
        if (popperInButton) {
            popper.set(ControlMode.PercentOutput, POPPER_SPEED_IN);
            handlePopper(false);
        } else if (popperOutButton) {
            popper.set(ControlMode.PercentOutput, POPPER_SPEED_OUT);
            handlePopper(false);
        } else {
            handlePopper(true);
        }

    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {

        if (!testDone) {
            if (testRotation) {
                testDone = turnDegs(testTurnAmount);
            } else {
                testDone = moveInches(testMoveDistance);
            }
        } else {
            leftMaster.set(ControlMode.PercentOutput, 0);
            rightMaster.set(ControlMode.PercentOutput, 0);
        }
    }

    public void testInit() {
        resetEncoders();
        gyro.reset();
        testDone = false;
        rotationPID = new ProfiledPIDController(rotationGains.P, rotationGains.I, rotationGains.D, ROTATIONAL_GAIN_CONSTRAINTS);

        initializeMotionMagicMaster(rightMaster);
        initializeMotionMagicMaster(leftMaster);
    }

    /**
     * sets encoder position to zero
     * 
     * @return true if successful and false if error
     */
    public boolean resetEncoders() {
        ErrorCode rightError = rightMaster.setSelectedSensorPosition(0);
        ErrorCode leftError = leftMaster.setSelectedSensorPosition(0);
        return rightError.value == 0 && leftError.value == 0;
    }

    /**
     * move a distance in s straight line
     * 
     * @param inches the distance in inches to move forward (use negative number to go backward)
     * 
     * @return true when done
     */
    public boolean moveInches(double inches) {
        inches = -inches;
        leftMaster.set(ControlMode.MotionMagic, inchesToTicks(inches));
        rightMaster.set(ControlMode.MotionMagic, inchesToTicks(inches));
        System.out.println(
            (Math.abs(leftMaster.getSelectedSensorPosition() - inchesToTicks(inches)) - inchesToTicks(distanceMarginOfError))
            + " " + (Math.abs(leftMaster.getActiveTrajectoryVelocity()) < inchesToTicks(1) * 10)
            + " " + (Math.abs(rightMaster.getSelectedSensorPosition() - inchesToTicks(inches)) - inchesToTicks(distanceMarginOfError))
            + " " + (Math.abs(rightMaster.getActiveTrajectoryVelocity()) < inchesToTicks(1) * 10)
        );
        if (
            Math.abs(leftMaster.getSelectedSensorPosition() - inchesToTicks(inches)) < inchesToTicks(distanceMarginOfError)
            && Math.abs(leftMaster.getActiveTrajectoryVelocity()) < inchesToTicks(1) * 10
            && Math.abs(rightMaster.getSelectedSensorPosition() - inchesToTicks(inches)) < inchesToTicks(distanceMarginOfError)
            && Math.abs(rightMaster.getActiveTrajectoryVelocity()) < inchesToTicks(1) * 10
        ) {
            System.out.println("done");
            rightMaster.set(ControlMode.PercentOutput, 0);
            leftMaster.set(ControlMode.PercentOutput, 0);
            return true;
        } else {
            return false;
        }
    }

    /**
     * converts inches to the value needed by the talon encoder for motion magic
     * 
     * @param inches a distance measured in inches
     * 
     * @return the number of encoder ticks equivalent to the input distance
     */
    double inchesToTicks(double inches) {
        return encoderRotation * inches / circumference;
    }

    /**
     * Turns robot a number of degrees. Should be called every tick until it returns true.
     * 
     * @param radians the number of radians to turn
     * 
     * @return true if done
     */
    boolean turnRads(double radians) {
        return turnDegs(radians * 180 / Math.PI);
    }

    /**
     * Turns robot a number of degrees. Should be called every tick until it returns true.
     * 
     * @param degrees the number of degrees to turn
     * 
     * @return true if done
     */
    public boolean turnDegs(double degrees) {
        degrees %= 360;
        if (180 < degrees) {
            degrees -= 360;
        } else if (-180 > degrees) {
            degrees += 360;
        }
        double output = MathUtil.clamp(rotationPID.calculate(gyro.getAngle(), degrees), -1, 1);
        System.out.println(rotationPID.getPositionError() + "    " + output);
        // if (output > 0) {
        //     output += 0.10;
        // } else if (output < 0) {
        //     output -= 0.10;
        // }
        if (
            Math.abs(gyro.getAngle() - degrees) < angleMarginOfError
            && Math.abs(rightMaster.getSelectedSensorVelocity()) < 1024 / 4
            && Math.abs(leftMaster.getSelectedSensorVelocity()) < 1024 / 4
        ) {
            rightMaster.set(ControlMode.PercentOutput, 0);
            leftMaster.set(ControlMode.PercentOutput, 0);
            return true;
        } else {
            rightMaster.set(ControlMode.PercentOutput, output);
            leftMaster.set(ControlMode.PercentOutput, -output);
            return false;
        }
    }

    /**
     * sets a pneumatic piston to be extended or retracted
     * 
     * @param solenoid the Solenoid or DoubleSolenoid to be extended or retracted
     * @param value whether the solenoid should be extended
     */
    public void setPistonExtended(Object solenoid, boolean value) {
        if (solenoid instanceof Solenoid) {
            ((Solenoid) solenoid).set(value);
        } else if (solenoid instanceof DoubleSolenoid) {
            if (value) {
                ((DoubleSolenoid) solenoid).set(DoubleSolenoid.Value.kForward);
            } else {
                ((DoubleSolenoid) solenoid).set(DoubleSolenoid.Value.kReverse);
            }

        } else {
            throw new Error(solenoid.getClass().getSimpleName() + " is not a Solenoid or a DoubleSolenoid");
        }
    }

    // this code is called from auton and teleop periodic and uses sensors to automatically handle the popper
    void handlePopper(boolean shouldSetPopper) {
        // if a ball is ready to be popped
        if (!INTAKE_SENSOR.get()) {
            popperInTime = POPPER_TIME_IN;
            intakeTime++;
        } else {
            if (intakeTime > INTAKE_COUNTER_COUNT_TIME) {
                System.out.println("ball incoming");
                //ballsStored++;
                System.out.println("ballsStored: " + ballsStored);
            } else if (intakeTime > 0) {
                System.out.println("intakeTime: " + intakeTime);
            }
            intakeTime = 0;
        }

        // if there is a ball at the top of the popper
        if (!POPPER_SENSOR.get()) {
            if (++popperCounterTime > POPPER_COUNTER_JAM_TIME) {
                System.out.println("popper jammed");
                popperInTime = POPPER_TIME_IN;
                popperOutTime = POPPER_TIME_OUT;
            }
        } else {
            popperCounterTime = 0;
        }

        if (shouldSetPopper) {
            if (popperOutTime-- > 0) {
                popper.set(ControlMode.PercentOutput, POPPER_SPEED_OUT);
            } else if (popperInTime-- > 0) {
                popper.set(ControlMode.PercentOutput, POPPER_SPEED_IN);
            } else {
                popper.set(ControlMode.PercentOutput, 0);
            }
        }

        if (drawbridgeSol.get() == Value.kForward) {
            ballsStored = 0;
            System.out.println("ballsStored: " + ballsStored);
        }
    }
}

enum Challenge {
    GALACTIC_SEARCH, AUTONAV, HYPERDRIVE, INTERSTELLAR_ACCURACY, POWERPORT
}

enum Path {
    BARREL_RACING, SLALOM, BOUNCE
}

enum GalacticSearchMode {
    RedA(0), BlueA(1), RedB(2), BlueB(3);
    private final int value;
    private GalacticSearchMode(int value) {
        this.value = value;
    }
    public int getValue() {
        return value;
    }
}