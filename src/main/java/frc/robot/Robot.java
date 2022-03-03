/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.motor.*;

import java.util.ArrayList;
import java.util.StringTokenizer;

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

    // creates objects for the talons
    public IMotorController leftMaster = MotorControllerFactory.create(this, K.LEFT_MASTER_ID, K.LEFT_MASTER_TYPE);
    IMotorController leftSlave = MotorControllerFactory.create(this, K.LEFT_SLAVE_ID, K.LEFT_SLAVE_TYPE);
    public IMotorController rightMaster = MotorControllerFactory.create(this, K.RIGHT_MASTER_ID, K.RIGHT_MASTER_TYPE);
    IMotorController rightSlave = MotorControllerFactory.create(this, K.RIGHT_SLAVE_ID, K.RIGHT_SLAVE_TYPE);
    IMotorController hangMotor = MotorControllerFactory.create(this, K.WINCH_MOTOR_ID, K.WINCH_MOTOR_TYPE);
    IMotorController intake = MotorControllerFactory.create(this, K.INTAKE_ID, K.INTAKE_TYPE);
    IMotorController popper = MotorControllerFactory.create(this, K.POPPER_ID, K.POPPER_TYPE);

    // ##########################################
    // drivetrain and pid related constants and variables
    // ##########################################

    // the object that is the navX-MXP
    public AHRS gyro = new AHRS(Port.kMXP);
    static Gains rotationGains;

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

    /** true if K.ROBOT_SENSOR is pulled low */
    boolean isPracticeRobot;
    /** the circumference of the drive wheels */
    public double circumference;

    /** weather to use Arcade (true) or tank (false) style controls */
    boolean ArcadeDrive = true;

    static Gains gains; // used for drivetrain motion magic when moving and is ste to
                        // K.PRACTICE_ROBOT_GAINS or COMPETITION_ROBOT_GAINS

    // the amount of time in robot cycles that this will move
    int popperInTime = 0;
    int popperOutTime = 0;

    int popperCounterTime; // the number of cycles that the counter sensor has been interrupted for
    int intakeTime; // the number of cycles that the counter sensor has bean interrupted for
    int ballsStored = 0; // the number of balls in the robot

    // ##########################################
    // Drawbridge and hang related constants and variables
    // ##########################################

    // declares objects for the TwoStateMotor class
    TwoStateMotor hang = new RatchetMotor(0.5, -0.1, hangMotor, K.HANG_DEFAULT_SENSOR, K.HANG_SET_SENSOR);;

    // ##########################################
    // Controller related constants and variables
    // ##########################################

    XboxController xboxController = new XboxController(0); // driver
    double leftjoyY; // y-axis of the left joystick on the driver's controller
    double rightjoyY; // y-axis of the right joystick on the driver's controller
    double leftjoyX; // x-axis of the left joystick on the driver's controller
    double rightjoyX; // x-axis of the right joystick on the driver's controller

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
    // Pneumatics related variables
    // ##########################################

    Compressor compressor = null;

    DoubleSolenoid drawbridgeSol = new DoubleSolenoid(1, PneumaticsModuleType.CTREPCM, K.PCM_DRAWBRIDGE_IN, K.PCM_DRAWBRIDGE_OUT);
    Solenoid hangSol = new Solenoid(1, PneumaticsModuleType.CTREPCM, K.PCM_RATCHET);

    Pixy2 pixy;
    GalacticSearchMode galacticSearch;
    GalacticSearchMode[] galacticSearchResults = new GalacticSearchMode[20];
    int galacticSearchNext = 0;

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

        isPracticeRobot = !K.ROBOT_SENSOR.get();
        if (isPracticeRobot) {
            circumference = 6 * Math.PI;
            gains = K.PRACTICE_ROBOT_GAINS;
            rotationGains = K.PRACTICE_ROTATION_GAINS;
            System.out.println("using 6 inch wheels");
        } else {
            circumference = 8 * Math.PI;
            gains = K.COMPETITION_ROBOT_GAINS;
            rotationGains = K.COMPETITION_ROTATION_GAINS;
            System.out.println("using 8 inch wheels");
        }

        rotationPID = new ProfiledPIDController(rotationGains.P, rotationGains.I, rotationGains.D, K.ROTATIONAL_GAIN_CONSTRAINTS);

        initializeMotor(leftMaster, true, false);
        initializeMotor(leftSlave, true, false);
        initializeMotor(rightMaster, true, true);
        initializeMotor(rightSlave, true, true);
        initializeMotor(hangMotor, true, false);
        initializeMotor(intake, false, false);
        initializeMotor(popper, false, false);

        initializeMotionMagicMaster(rightMaster);
        initializeMotionMagicMaster(leftMaster);

        rightSlave.follow(rightMaster);
        leftSlave.follow(leftMaster);
        gyro.reset();

        pixy = Pixy2.createInstance(new SPILink());
        pixy.init();
    }

    public void initializeMotor(IMotorController talon, boolean neutralMode, boolean inverted) {
        /* Ensure motor output is neutral during init */
        talon.setSpeed(0);

        /* Factory Default all hardware to prevent unexpected behavior */
        talon.reset();

        /* Set Neutral mode */
        talon.setBrakeMode(neutralMode);

        /* Configure output direction */
        talon.setInverted(inverted);
    }

    public void initializeMotionMagicMaster(IMotorController masterTalon) {
        /* Factory default hardware to prevent unexpected behavior */
        masterTalon.reset();

        /* Configure Sensor Source for Primary PID */
        masterTalon.setSensorSource();

        /*
         * set deadband to super small 0.001 (0.1 %). The default deadband is 0.04 (4 %)
         */
        masterTalon.setNeutralDeadband(0.001);

        /**
         * Configure Talon SRX Output and Sensor direction accordingly Invert Motor to
         * have green LEDs when driving Talon Forward / Requesting Positive Output Phase
         * sensor to have positive increment when driving Talon Forward (Green LED)
         */
        masterTalon.setEncoderInverted(false);

        /* Set relevant frame periods to be at least as fast as periodic rate */
        masterTalon.setStatusFramePeriod(10);

        /* Set the peak and nominal outputs */
        masterTalon.setOutputLimits(0, 0, 1, -1);

        /* Set Motion Magic gains in slot0 - see documentation */
        masterTalon.setPID(gains);

        /* Set acceleration and vcruise velocity - see documentation */
        masterTalon.setMotionSpeed(15000, 6000);

        /* Zero the sensor once on robot boot up */
        masterTalon.setEncoderPosition(0);
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
            SmartDashboard.putString("GalacticSearch", K.galacticSearchNames[maxn]);
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
     * K.STARTING_POSITION structure below with additional strings. If using the
     * SendableChooser make sure to add them to the chooser code above as well.
     */
    @Override
    public void autonomousInit() {
        autonInstructions.clear();
        resetEncoders();
        gyro.reset();
        rotationPID = new ProfiledPIDController(rotationGains.P, rotationGains.I, rotationGains.D, K.ROTATIONAL_GAIN_CONSTRAINTS);

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
                        autonInstructions.add(new MoveInch(135+ K.robotLength / 2));
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
                        autonInstructions.add(new MoveInch(K.robotLength / 2));
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
                        autonInstructions.add(new MoveInch(30 + K.robotLength / 2));
                        autonInstructions.add(new TurnDeg(-90)); // c3
                        autonInstructions.add(new MoveInch(60 - K.robotLength / 2));
                        //a3
                        autonInstructions.add(new MoveInch(-60 + K.robotLength / 2)); 
                        autonInstructions.add(new TurnDeg(-45)); // c3
                        autonInstructions.add(new MoveInch(-60 * Math.sqrt(2)));
                        autonInstructions.add(new TurnDeg(90)); // e5
                        autonInstructions.add(new MoveInch(30 * Math.sqrt(2)));
                        autonInstructions.add(new TurnDeg(-45)); // d6
                        autonInstructions.add(new MoveInch(90 - K.robotLength / 2));
                        // a6
                        autonInstructions.add(new MoveInch(-120 + K.robotLength / 2));
                        autonInstructions.add(new TurnDeg(90)); // e6
                        autonInstructions.add(new MoveInch(90));
                        autonInstructions.add(new TurnDeg(-90)); // e9
                        autonInstructions.add(new MoveInch(120 - K.robotLength / 2));
                        // a9
                        autonInstructions.add(new MoveInch(-30 + K.robotLength / 2));
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
                if (ballsStored < K.MAX_BALLS) {
                    intake.setSpeed(K.INTAKE_SPEED_IN);
                } else {
                    intake.setSpeed(K.INTAKE_SPEED_OUT);
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
        arcadeButton = xboxController.getRawButton(K.L_STICK);
        tankButton = xboxController.getRawButton(K.R_STICK);
        drawbridgeButton = xboxController.getRawButton(K.A_BUTTON);
        intakeInButton = 0.1 < xboxController.getLeftTriggerAxis();
        intakeOutButton = 0.1 < xboxController.getRightTriggerAxis();
        popperInButton = xboxController.getRawButton(K.L_SHOULDER);
        popperOutButton = xboxController.getRawButton(K.R_SHOULDER);
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
            leftMaster.setSpeed((y * (2 - Math.abs(x)) - x * (2 - Math.abs(y))) / 2);
            rightMaster.setSpeed((y * (2 - Math.abs(x)) + x * (2 - Math.abs(y))) / 2);
        } else {
            leftMaster.setSpeed(leftjoyY);
            rightMaster.setSpeed(rightjoyY);
        }

        if (intakeInButton && ballsStored >= K.MAX_BALLS) {
            xboxController.setRumble(RumbleType.kLeftRumble, 1.0);
            xboxController.setRumble(RumbleType.kRightRumble, 1.0);
        } else {
            xboxController.setRumble(RumbleType.kLeftRumble, 0.0);
            xboxController.setRumble(RumbleType.kRightRumble, 0.0);
        }

        // this code handles intake
        if (intakeInButton) {
            intake.setSpeed(K.INTAKE_SPEED_IN);
        } else if (intakeOutButton || ballsStored >= K.MAX_BALLS) {
            intake.setSpeed(K.INTAKE_SPEED_OUT);
        } else {
            intake.setSpeed(0);
        }

        // this code handles the popper
        if (popperInButton) {
            popper.setSpeed(K.POPPER_SPEED_IN);
            handlePopper(false);
        } else if (popperOutButton) {
            popper.setSpeed(K.POPPER_SPEED_OUT);
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
            leftMaster.setSpeed(0);
            rightMaster.setSpeed(0);
        }
    }

    public void testInit() {
        resetEncoders();
        gyro.reset();
        testDone = false;
        rotationPID = new ProfiledPIDController(rotationGains.P, rotationGains.I, rotationGains.D, K.ROTATIONAL_GAIN_CONSTRAINTS);

        initializeMotionMagicMaster(rightMaster);
        initializeMotionMagicMaster(leftMaster);
    }

    /**
     * sets encoder position to zero
     * 
     * @return true if successful and false if error
     */
    public boolean resetEncoders() {
        // TODO: Add error checking for new motor controller methods
        /*ErrorCode rightError = */rightMaster.setEncoderPosition(0);
        /*ErrorCode leftError = */leftMaster.setEncoderPosition(0);
        return true; // return rightError.value == 0 && leftError.value == 0;
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
        leftMaster.setDistance(inchesToTicks(inches));
        rightMaster.setDistance(inchesToTicks(inches));
        System.out.println(
            (Math.abs(leftMaster.getEncoderPosition() - inchesToTicks(inches)) - inchesToTicks(distanceMarginOfError))
            + " " + (Math.abs(leftMaster.getActiveTrajectoryVelocity()) < inchesToTicks(1) * 10)
            + " " + (Math.abs(rightMaster.getEncoderPosition() - inchesToTicks(inches)) - inchesToTicks(distanceMarginOfError))
            + " " + (Math.abs(rightMaster.getActiveTrajectoryVelocity()) < inchesToTicks(1) * 10)
        );
        if (
            Math.abs(leftMaster.getEncoderPosition() - inchesToTicks(inches)) < inchesToTicks(distanceMarginOfError)
            && Math.abs(leftMaster.getActiveTrajectoryVelocity()) < inchesToTicks(1) * 10
            && Math.abs(rightMaster.getEncoderPosition() - inchesToTicks(inches)) < inchesToTicks(distanceMarginOfError)
            && Math.abs(rightMaster.getActiveTrajectoryVelocity()) < inchesToTicks(1) * 10
        ) {
            System.out.println("done");
            rightMaster.setSpeed(0);
            leftMaster.setSpeed(0);
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
        return K.encoderRotation * inches / circumference;
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
            && Math.abs(rightMaster.getSensorVelocity()) < 1024 / 4
            && Math.abs(leftMaster.getSensorVelocity()) < 1024 / 4
        ) {
            rightMaster.setSpeed(0);
            leftMaster.setSpeed(0);
            return true;
        } else {
            rightMaster.setSpeed(output);
            leftMaster.setSpeed(-output);
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
        if (!K.INTAKE_SENSOR.get()) {
            popperInTime = K.POPPER_TIME_IN;
            intakeTime++;
        } else {
            if (intakeTime > K.INTAKE_COUNTER_COUNT_TIME) {
                System.out.println("ball incoming");
                //ballsStored++;
                System.out.println("ballsStored: " + ballsStored);
            } else if (intakeTime > 0) {
                System.out.println("intakeTime: " + intakeTime);
            }
            intakeTime = 0;
        }

        // if there is a ball at the top of the popper
        if (!K.POPPER_SENSOR.get()) {
            if (++popperCounterTime > K.POPPER_COUNTER_JAM_TIME) {
                System.out.println("popper jammed");
                popperInTime = K.POPPER_TIME_IN;
                popperOutTime = K.POPPER_TIME_OUT;
            }
        } else {
            popperCounterTime = 0;
        }

        if (shouldSetPopper) {
            if (popperOutTime-- > 0) {
                popper.setSpeed(K.POPPER_SPEED_OUT);
            } else if (popperInTime-- > 0) {
                popper.setSpeed(K.POPPER_SPEED_IN);
            } else {
                popper.setSpeed(0);
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