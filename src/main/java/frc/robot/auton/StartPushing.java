package frc.robot.auton;

import frc.robot.Robot;

/**
 * sets the robot pushing lightly forward to keep it on the wall while dumping
 * powercells
 */
public class StartPushing extends Instruction {

    public StartPushing() {
    }

    @Override
    public boolean doit(Robot robot) {
        System.out.println("pushing");
        robot.rightMaster.setSpeed(-0.2);
        robot.leftMaster.setSpeed(-0.2);
        return true;
    }

}