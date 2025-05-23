package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.RotateManipulatorSubsystem;

public class ManipulatorToPoint extends Command {
    private final ManipulatorSubsystem manipulatorSubsystem;
    private int switchCaseValue;
    private final ElevatorSubsystem elevatorSubsystem;
    private final RotateManipulatorSubsystem rotateManipulatorSubsystem;

    /**
     * Used for both elevator and manipulator movement at once
     * @param manipulatorSubsystem
     * @param elevatorSubsystem
     * @param rotateManipulatorSubsystem
     * @param switchCaseValue
     */
    public ManipulatorToPoint(ManipulatorSubsystem manipulatorSubsystem, ElevatorSubsystem elevatorSubsystem, RotateManipulatorSubsystem rotateManipulatorSubsystem, int switchCaseValue) {
        this.manipulatorSubsystem = manipulatorSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.switchCaseValue = switchCaseValue;
        this.rotateManipulatorSubsystem = rotateManipulatorSubsystem;
        addRequirements(manipulatorSubsystem, elevatorSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        switch (switchCaseValue) {
            case 0://human player height
                elevatorSubsystem.moveToPosition(ElevatorConstants.ELEVATOR_POSITION_0);
                rotateManipulatorSubsystem.setIntakePosition(ManipulatorConstants.INTAKE_ZERO);
                manipulatorSubsystem.turnPrimaryJointToPosition(42.118);

                break;
            case 1: //Level 2
                elevatorSubsystem.moveToPosition(ElevatorConstants.ELEVATOR_POSITION_1);
                rotateManipulatorSubsystem.setIntakePosition(ManipulatorConstants.INTAKE_ROTATED);
                manipulatorSubsystem.turnPrimaryJointToPosition(61.667);

                break;
            case 2: //Level 3
                elevatorSubsystem.moveToPosition(ElevatorConstants.ELEVATOR_POSITION_2);
                rotateManipulatorSubsystem.setIntakePosition(ManipulatorConstants.INTAKE_ROTATED);
                manipulatorSubsystem.turnPrimaryJointToPosition(53.381);

                break;
            case 3: //Level 4
                manipulatorSubsystem.turnPrimaryJointToPosition(33.761);
                rotateManipulatorSubsystem.setIntakePosition(ManipulatorConstants.INTAKE_ROTATED);
                elevatorSubsystem.moveToPosition(ElevatorConstants.ELEVATOR_POSITION_3);
                
                break;

            case 4: //Level 1
                elevatorSubsystem.moveToPosition(ElevatorConstants.ELEVATOR_POSITION_0);
                rotateManipulatorSubsystem.setIntakePosition(ManipulatorConstants.INTAKE_ZERO);
                manipulatorSubsystem.turnPrimaryJointToPosition(80);

                break;
            case 5: 
                rotateManipulatorSubsystem.setIntakePosition(ManipulatorConstants.INTAKE_ZERO);
                manipulatorSubsystem.turnPrimaryJointToPosition(41.8805);
                break;
            case 6:
                elevatorSubsystem.moveToPosition(ElevatorConstants.ELEVATOR_POSITION_3);
                rotateManipulatorSubsystem.setIntakePosition(ManipulatorConstants.INTAKE_ROTATED);
                manipulatorSubsystem.turnPrimaryJointToPosition(56.3577);
                break;
            case 7:
                manipulatorSubsystem.turnPrimaryJoint(40.3805);
                break;
            case 8: // Level 4 auto
                manipulatorSubsystem.turnPrimaryJointToPosition(33.761);
                try {
                    Thread.sleep(1000); // Wait for 0.5 seconds
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                rotateManipulatorSubsystem.setIntakePosition(ManipulatorConstants.INTAKE_ROTATED);
                elevatorSubsystem.moveToPosition(ElevatorConstants.ELEVATOR_POSITION_3);
            
            break;

            default:
                break;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
    
}
