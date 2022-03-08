package frc.robot.motor;

import frc.robot.Robot;

public class MotorControllerFactory {
    public static MotorController create(Robot robot, int id, MotorController.Type type) {
        switch (type) {
            case Talon: return new TalonMotorController(id, robot);
            case SparkMax: return new SparkMotorController(id, robot);
            default: return null;
        }
    }
}
