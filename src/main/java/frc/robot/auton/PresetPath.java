package frc.robot.auton;

import java.util.List;
import frc.robot.Robot;

class PresetPath extends Instruction {
    static final PresetPath AutoNav_BarrelRace = new PresetPath(List.of(new MoveInch(1.0)));
    static final PresetPath AutoNav_Slalom = new PresetPath(List.of(new MoveInch(1.0)));
    static final PresetPath AutoNav_Bounce = new PresetPath(List.of(new MoveInch(1.0)));

    public List<Instruction> instructions;
    private int currentInstruction = 0;

    PresetPath(List<Instruction> insts) {
        instructions = insts;
    }

    @Override
    public boolean doit(Robot robot) {
        if (currentInstruction >= instructions.size())
            return true;
        boolean res = instructions.get(currentInstruction).doit(robot);
        if (res)
            currentInstruction++;
        return currentInstruction >= instructions.size();
    }
}