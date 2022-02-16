package frc.robot.auton;

import frc.robot.Robot;

public class MoveInch extends Instruction {

	double inches;

	/**
	 * @param inches the number of inches to move forward (negative value to go backward)
	 */
	public MoveInch(double inches) {
		this.inches = inches;
	}

	@Override
	public boolean doit(Robot robot) {
		boolean done = robot.moveInches(inches);
		if (done) {
			return robot.resetEncoders();
		} else {
			return false;
		}
	}

}
