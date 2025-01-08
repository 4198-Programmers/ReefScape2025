package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BallIntakePneumaticSubsystem extends SubsystemBase {
    
    private Solenoid ballIntakeSolenoidLeft = new Solenoid(Constants.LEFT_SOLENOID_MODULE, PneumaticsModuleType.CTREPCM, Constants.LEFT_SOLENOID_CHANNEL);
    private Solenoid ballIntakeSolenoidRight = new Solenoid(Constants.RIGHT_SOLENOID_MODULE, PneumaticsModuleType.CTREPCM, Constants.RIGHT_SOLENOID_CHANNEL);

    public BallIntakePneumaticSubsystem() {
        ballIntakeSolenoidLeft.set(false);
        ballIntakeSolenoidRight.set(false);
    }

    public void intakeUp() {
        ballIntakeSolenoidLeft.toggle();
        ballIntakeSolenoidRight.toggle();
    }

    public void intakeDown() {
        ballIntakeSolenoidLeft.toggle();
        ballIntakeSolenoidRight.toggle();
    }

    


}
