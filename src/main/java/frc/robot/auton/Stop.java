package frc.robot.auton;

import frc.robot.Robot;

/**
 * sets the robot to stop immediately
 */
public class Stop extends Instruction {

    public Stop() {
    }

    @Override
    public boolean doit(Robot robot) {
        System.out.println("stopping");
        robot.rightMaster.setPercentOutput(0);
        robot.leftMaster.setPercentOutput(0);
        return true;
    }

}