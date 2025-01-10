package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VariablePneumaticSubsystem extends SubsystemBase{
    //This creates a new subsystem for pneumatics

    private Solenoid solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.SOLENOID_CHANNEL);
    //This makes solenoid a reference and makes a constant for it

    public VariablePneumaticSubsystem() {}
    //This makes a method for VariablePneumaticSubsystem

    public void setPneumaticForward() {
        solenoid.set(true);
    }
    public void setPneumaticReverse() {
        solenoid.set(false);
    }
    //This means that on true, the piston will go forward, and on false, the piston will go back
    
    
}
