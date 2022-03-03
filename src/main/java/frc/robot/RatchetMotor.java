package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.motor.IMotorController;

public class RatchetMotor extends TwoStateMotor {

    int backwardTime = 0;

    RatchetMotor(double speed, IMotorController motor, DigitalInput defaultSensor, DigitalInput setSensor) {
        super(speed, motor, defaultSensor, setSensor);
    }
    RatchetMotor(double speed, double speedOffset, IMotorController motor, DigitalInput defaultSensor, DigitalInput setSensor) {
        super(speed, speedOffset, motor, defaultSensor, setSensor);
    }

    // this method should be called in robot periodic
    // checks sensor values and motor speed
    public void tick() {
        
        isDefault = defaultSensor.get();
        isSet = setSensor.get();
        if(--backwardTime > 0){
            motor.setSpeed(((direction * speed ) + speedOffset) * -1);
        } else if (isSet && direction == 1 || isDefault && direction == -1) {
            motor.setSpeed((direction * speed) + speedOffset);
            //System.out.println("moving at "+ ((direction * speed) + speedOffset));
        } else {
            motor.setSpeed(0);
            //System.out.println("stopped");
        }
    }

    // call this function to change the limit that the motor will go to and stay at
    public void set(boolean set) {
        if (set && direction != 1) {
            backwardTime = 25;
            direction = 1;
        } else if (direction != -1) {
            backwardTime = 25;
            direction = -1;
        }
    }
}