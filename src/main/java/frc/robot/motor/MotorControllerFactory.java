package frc.robot.motor;

import frc.robot.Robot;

public class MotorControllerFactory {
    public static IMotorController create(Robot robot, int id, IMotorController.Type type) {
        switch (type) {
            case Talon: return new TalonMotorController(id, robot);
            case SparkMax: return new SparkMotorController(id);
            default: return null;
        }
    }
}
