package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BallIntakePneumaticSubsystem extends SubsystemBase {
    
    // Make a new Solenoid object
    private Solenoid ballIntakeSolenoid = new Solenoid(Constants.SOLENOID_MODULE, PneumaticsModuleType.CTREPCM, Constants.SOLENOID_CHANNEL);
    private Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

    public BallIntakePneumaticSubsystem() {
        this.initialize();
    }

    // Intialize Method
    public void initialize() {
        compressor.enableDigital();
        ballIntakeSolenoid.set(false);
    }
    // Intake Up Method
    public void intakeUp() {
        ballIntakeSolenoid.toggle();
    }

    // Intake Down Method
    public void intakeDown() {
        ballIntakeSolenoid.toggle();
    }

    // Intake Ended Method
    public void ended() {
        ballIntakeSolenoid.set(false);
    }



}
