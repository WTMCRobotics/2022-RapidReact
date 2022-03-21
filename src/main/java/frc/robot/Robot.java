/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.motor.*;

import java.util.ArrayList;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.ColorSensorResolution;
import com.revrobotics.ColorSensorV3.ColorSensorMeasurementRate;
import com.revrobotics.ColorSensorV3.GainFactor;
import com.revrobotics.ColorSensorV3.ProximitySensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ProximitySensorResolution;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.hal.AllianceStationID;
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

    /** a list of instructions to follow */
    ArrayList<Instruction> autonInstructions = new ArrayList<Instruction>();

    // creates objects for the talons
    public MotorController leftMaster = MotorControllerFactory.create(this, K.LEFT_MASTER_ID, K.LEFT_MASTER_TYPE);
    MotorController leftSlave = MotorControllerFactory.create(this, K.LEFT_SLAVE_ID, K.LEFT_SLAVE_TYPE);
    public MotorController rightMaster = MotorControllerFactory.create(this, K.RIGHT_MASTER_ID, K.RIGHT_MASTER_TYPE);
    MotorController rightSlave = MotorControllerFactory.create(this, K.RIGHT_SLAVE_ID, K.RIGHT_SLAVE_TYPE);
    MotorController intake = MotorControllerFactory.create(this, K.INTAKE_ID, K.INTAKE_TYPE);
    MotorController lift = MotorControllerFactory.create(this, K.LIFT_ID, K.LIFT_TYPE);
    MotorController turretRotation = MotorControllerFactory.create(this, K.TURRET_ROTATION_ID, K.TURRET_ROTATION_TYPE);
    LimitedMotor turretRotationLimiter = new LimitedMotor(turretRotation, K.TURRET_ROTATION_ANGLE, K.TURRET_ROTATION_SPEED);
    MotorController turretShooter = MotorControllerFactory.create(this, K.TURRET_SHOOTER_ID, K.TURRET_SHOOTER_TYPE);
    MotorController turretHood = MotorControllerFactory.create(this, K.TURRET_HOOD_ID, K.TURRET_HOOD_TYPE);
    MotorController turretIntake = MotorControllerFactory.create(this, K.TURRET_INTAKE_ID, K.TURRET_INTAKE_TYPE);

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

    // The current state of the lift system
    static enum LiftState {
        Bottom,
        MovingUp,
        Top,
        MovingDown
    }
    LiftState liftState = LiftState.Bottom;

    int ballsStored = 0; // the number of balls in the robot

    ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    Alliance currentAlliance = Alliance.Invalid;

    // ##########################################
    // Controller related constants and variables
    // ##########################################

    XboxController xboxController = new XboxController(0); // driver
    XboxController guitarController = new XboxController(1); // codriver
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
    boolean liftButton; // true if the button that runs the lift is pressed
    boolean turretLeftButton; // true if the button that turns the turret left is pressed
    boolean turretRightButton; // true if the button that turns the turret right is pressed
    boolean turretShooterSpinButton; // true if the button that spins the turret shooter is pressed
    boolean turretLaunchButton; // true if the button that spins the turret intake is pressed

    Pixy2 pixy;

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        System.out.println("starting robotInit()");

        circumference = 8 * Math.PI;
        gains = K.COMPETITION_ROBOT_GAINS;
        rotationGains = K.COMPETITION_ROTATION_GAINS;

        rotationPID = new ProfiledPIDController(rotationGains.P, rotationGains.I, rotationGains.D, K.ROTATIONAL_GAIN_CONSTRAINTS);

        initializeMotor(leftMaster, false, false);
        initializeMotor(leftSlave, false, false);
        initializeMotor(rightMaster, false, true);
        initializeMotor(rightSlave, false, true);
        initializeMotor(intake, false, false);
        initializeMotor(lift, true, false);
        initializeMotor(turretRotation, true, false);
        initializeMotor(turretShooter, false, false);
        initializeMotor(turretHood, true, false);
        initializeMotor(turretIntake, false, false);

        initializeMotionMagicMaster(rightMaster, gains);
        initializeMotionMagicMaster(leftMaster, gains);
        initializeMotionMagicMaster(turretRotation, K.TURRET_ROTATION_GAINS);

        rightSlave.follow(rightMaster);
        leftSlave.follow(leftMaster);
        gyro.reset();

        pixy = Pixy2.createInstance(new SPILink());
        pixy.init();

        // TODO: Make a proper field-selectable color selector
        K.PIXY_LED_RED.set(true);
        K.PIXY_LED_GREEN.set(true);
        K.PIXY_LED_BLUE.set(false);

        colorSensor.configureColorSensor(ColorSensorResolution.kColorSensorRes18bit, ColorSensorMeasurementRate.kColorRate100ms, GainFactor.kGain1x);
        colorSensor.configureProximitySensor(ProximitySensorResolution.kProxRes11bit, ProximitySensorMeasurementRate.kProxRate100ms);

        CameraServer.startAutomaticCapture();
        CameraServer.startAutomaticCapture(); // two cameras

        turretRotationLimiter.disabled = true; // TODO: DISABLE ONCE LIMIT SWITCHES EXIST
    }

    public void initializeMotor(MotorController talon, boolean neutralMode, boolean inverted) {
        /* Ensure motor output is neutral during init */
        talon.setPercentOutput(0);

        /* Factory Default all hardware to prevent unexpected behavior */
        talon.reset();

        /* Set Neutral mode */
        talon.setBrakeMode(neutralMode);

        /* Configure output direction */
        talon.setInverted(inverted);
    }

    public void initializeMotionMagicMaster(MotorController masterTalon, Gains gains) {
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

        /*Pixy2CCC ccc = pixy.getCCC();
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
        }*/
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

        autonInstructions.add(new StartPushing());
        autonInstructions.add(new WaitMs(4000));
        autonInstructions.add(new Stop());
        autonInstructions.add(new WaitMs(100000));
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        while (!autonInstructions.isEmpty() && autonInstructions.get(0).doit(this)) {
            autonInstructions.remove(0);
        }
    }

    /**
     * This function is called periodically during teleop.
     */
    @Override
    public void teleopInit() {
        pixy.setLamp((byte)0, (byte)0);
        currentAlliance = DriverStation.getAlliance();
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {

        // this code updates the controller variables to the correct value at the
        // beginning of teleopPeriodic()
        leftjoyY = deadzone(xboxController.getLeftY(), 0.1);
        rightjoyY = deadzone(xboxController.getRightY(), 0.1);
        leftjoyX = deadzone(xboxController.getLeftX(), 0.1);
        rightjoyX = deadzone(xboxController.getRightX(), 0.25);
        arcadeButton = xboxController.getRawButton(K.L_STICK);
        tankButton = xboxController.getRawButton(K.R_STICK);
        intakeInButton = guitarController.getPOV() == 0;
        intakeOutButton = guitarController.getPOV() == 180;
        liftButton = guitarController.getRawButton(K.Y_BUTTON);
        turretRightButton = guitarController.getRawButton(K.A_BUTTON);
        turretLeftButton = guitarController.getRawButton(K.B_BUTTON);
        turretShooterSpinButton = guitarController.getRightX() > 0.1;
        turretLaunchButton = guitarController.getRawButton(K.X_BUTTON);

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
            leftMaster.setPercentOutput((y * (2 - Math.abs(x)) - x * (2 - Math.abs(y))) / 2);
            rightMaster.setPercentOutput((y * (2 - Math.abs(x)) + x * (2 - Math.abs(y))) / 2);
        } else {
            leftMaster.setPercentOutput(leftjoyY);
            rightMaster.setPercentOutput(rightjoyY);
        }

        // this code handles turret stuff
        if (turretLeftButton) {
            turretRotationLimiter.setTargetPos(turretRotationLimiter.getTargetPos() + 0.01);
        }
        if (turretRightButton) {
            turretRotationLimiter.setTargetPos(turretRotationLimiter.getTargetPos() - 0.01);
        }
        if (turretShooterSpinButton) {
            // TODO: Calibrate velocity settings
            turretShooter.setPercentOutput(guitarController.getRightX());
        } else {
            turretShooter.setPercentOutput(0);
        }
        if (turretLaunchButton) {
            turretIntake.setPercentOutput(K.TURRET_INTAKE_SPEED);
        } else {
            turretIntake.setPercentOutput(0);
        }

        turretRotationLimiter.tick();

        if (intakeInButton && ballsStored >= K.MAX_BALLS) {
            xboxController.setRumble(RumbleType.kLeftRumble, 1.0);
            xboxController.setRumble(RumbleType.kRightRumble, 1.0);
        } else {
            xboxController.setRumble(RumbleType.kLeftRumble, 0.0);
            xboxController.setRumble(RumbleType.kRightRumble, 0.0);
        }

        // this code handles intake
        if (intakeInButton) {
            intake.setPercentOutput(K.INTAKE_SPEED_IN);
        } else if (intakeOutButton || ballsStored >= K.MAX_BALLS) {
            intake.setPercentOutput(K.INTAKE_SPEED_OUT);
        } else {
            intake.setPercentOutput(0);
        }

        // this code handles the lift
        if (liftButton) {
            lift.setPercentOutput(K.LIFT_SPEED);
            handleLift(false);
        } else {
            handleLift(true);
        }

        // Send status of the robot to the dashboard
        SmartDashboard.putBoolean("Ball is loaded in shooter?", !K.SHOOTER_SENSOR.get());
        switch (liftState) {
            case Bottom: SmartDashboard.putString("Lift Status", "Waiting"); break;
            case MovingUp: SmartDashboard.putString("Lift Status", "Lifting"); break;
            case Top: SmartDashboard.putString("Lift Status", "Running Turret Intake"); break;
            case MovingDown: SmartDashboard.putString("Lift Status", "Returning"); break;
        }
        SmartDashboard.putNumber("Turret Rotation", turretRotationLimiter.getTargetPos());
        SmartDashboard.putNumber("Shooter Velocity", turretShooter.getSensorVelocity());
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
        System.out.println(colorSensor.getRed() + " " + colorSensor.getGreen() + " " + colorSensor.getBlue() + " " + colorSensor.getProximity());
        /*if (!testDone) {
            if (testRotation) {
                testDone = turnDegs(testTurnAmount);
            } else {
                testDone = moveInches(testMoveDistance);
            }
        } else {
            leftMaster.setPercentOutput(0);
            rightMaster.setPercentOutput(0);
        }*/
        //turretRotationLimiter.tick();
    }

    public void testInit() {
        resetEncoders();
        gyro.reset();
        testDone = false;
        rotationPID = new ProfiledPIDController(rotationGains.P, rotationGains.I, rotationGains.D, K.ROTATIONAL_GAIN_CONSTRAINTS);

        //initializeMotionMagicMaster(rightMaster);
        //initializeMotionMagicMaster(leftMaster);
        //turretRotationLimiter.reset();
        //turretRotationLimiter.setTargetPos(0.5);
    }

    /**
     * sets encoder position to zero
     * 
     * @return true if successful and false if error
     */
    public boolean resetEncoders() {
        // TODO: Add error checking for new motor controller methods
        rightMaster.setEncoderPosition(0);
        leftMaster.setEncoderPosition(0);
        turretRotation.setEncoderPosition(0);
        turretHood.setEncoderPosition(0);
        turretShooter.setEncoderPosition(0);
        turretRotationLimiter.reset();
        return true; // return rightError.value == 0 && leftError.value == 0;
    }

    public double deadzone(double n, double min) {
        if (Math.abs(n) < min) return 0;
        else return n;
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
        leftMaster.setDistance(inches);
        rightMaster.setDistance(inches);
        System.out.println(
            (Math.abs(leftMaster.getEncoderPosition() - (inches / circumference)) - distanceMarginOfError)
            + " " + (Math.abs(leftMaster.getActiveTrajectoryVelocity()) < (1.0 / circumference) * 10)
            + " " + (Math.abs(rightMaster.getEncoderPosition() - (inches / circumference)) - distanceMarginOfError)
            + " " + (Math.abs(rightMaster.getActiveTrajectoryVelocity()) < (1.0 / circumference) * 10)
        );
        if (
            Math.abs(leftMaster.getEncoderPosition() - (inches / circumference)) < distanceMarginOfError
            && Math.abs(leftMaster.getActiveTrajectoryVelocity()) < (1.0 / circumference) * 10
            && Math.abs(rightMaster.getEncoderPosition() - (inches / circumference)) < distanceMarginOfError
            && Math.abs(rightMaster.getActiveTrajectoryVelocity()) < (1.0 / circumference) * 10
        ) {
            System.out.println("done");
            rightMaster.setPercentOutput(0);
            leftMaster.setPercentOutput(0);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Turns robot a number of radians. Should be called every tick until it returns true.
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
            rightMaster.setPercentOutput(0);
            leftMaster.setPercentOutput(0);
            return true;
        } else {
            rightMaster.setPercentOutput(output);
            leftMaster.setPercentOutput(-output);
            return false;
        }
    }

    // this code is called from auton and teleop periodic and uses sensors to automatically handle the popper
    void handleLift(boolean shouldSetLift) {
        if (shouldSetLift && liftState != LiftState.MovingDown) {
            // if there is a ball in the shooter, do not lift
            if (!K.SHOOTER_SENSOR.get()) {
                liftState = LiftState.Bottom;
                lift.setPercentOutput(0);
                return;
            }

            // if there is a ball that is the wrong color, do not lift
            int blue = colorSensor.getBlue(), red = colorSensor.getRed();
            if ((currentAlliance == Alliance.Blue && red > blue) || (currentAlliance == Alliance.Red && blue > red)) {
                liftState = LiftState.Bottom;
                lift.setPercentOutput(0);
                return;
            }
        }

        // if a ball is ready to be popped
        if (liftState == LiftState.Bottom && colorSensor.getProximity() > 100) { // TODO: Calibrate this! CANNOT RUN WITHOUT!
            liftState = LiftState.MovingUp;
        }

        if (shouldSetLift) {
            switch (liftState) {
            case Bottom:
                lift.setPercentOutput(0);
                break;
            case MovingUp:
                if (!K.LIFT_TOP_SENSOR.get()) {
                    liftState = LiftState.Top;
                    lift.setPercentOutput(0);
                } else lift.setPercentOutput(K.LIFT_SPEED);
                break;
            case Top:
                if (!K.SHOOTER_SENSOR.get()) {
                    liftState = LiftState.MovingDown;
                    turretIntake.setPercentOutput(0);
                } else turretIntake.setPercentOutput(K.TURRET_INTAKE_SPEED);
                lift.setPercentOutput(0);
                break;
            case MovingDown:
                if (!K.LIFT_BOTTOM_SENSOR.get()) {
                    liftState = LiftState.Bottom;
                    lift.setPercentOutput(0);
                } else lift.setPercentOutput(K.LIFT_SPEED);
                break;
            }
        }
    }
}