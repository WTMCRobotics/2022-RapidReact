package frc.robot.auton;

import frc.robot.Robot;

public class TurnDeg extends Instruction {

	double degrees;

	/**
	 * @param degrees the number of degres to turn
	 */
	public TurnDeg(double degrees) {
		this.degrees = degrees;
	}

	@Override
	public boolean doit(Robot robot) {
		boolean done = robot.turnDegs(degrees);
		if (done && robot.resetEncoders()) {
			robot.gyro.reset();
			return true;
		}
		return false;
	}

}
