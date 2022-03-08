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
        robot.rightMaster.setPercentOutput(-0.2);
        robot.leftMaster.setPercentOutput(-0.2);
        return true;
    }

}