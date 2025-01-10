package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VariablePneumaticSubsystem;

public class SwitchSolenoidValue extends Command{
    //This creates a new command for switching solenoid values
    
    private VariablePneumaticSubsystem pneumatic;

    private boolean solenoidValue = false;

    public SwitchSolenoidValue(VariablePneumaticSubsystem pneumatic) {
        this.pneumatic = pneumatic;
        addRequirements(pneumatic);
    }
    //This turns "pneumatic" into a reference

    @Override
    public void execute() {
        solenoidValue = !solenoidValue;
    }

    @Override
    public void end() {
        solenoidValue = !solenoidValue;
    }
    //This when executed changes the value from false to true and back to false when it ends. When false, the piston is pushed back, and when true pushed out
}
