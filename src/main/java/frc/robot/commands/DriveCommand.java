package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Joystick;

import frc.robot.subsystems.TankSubsystem;

public class DriveCommand extends Command {
    Joystick leftStick;
    Joystick rightStick;
    private TankSubsystem drive;

    public DriveCommand(Joystick leftStick, Joystick rightStick, TankSubsystem drive)
    {
        this.leftStick = leftStick;
        this.rightStick = rightStick;
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        // System.out.println("EXECUTING!!");
        // System.out.println("Left: " + leftStick.getY() + " Right: " + rightStick.getX());
        drive.drive(leftStick.getY(), rightStick.getX());
    }
}
