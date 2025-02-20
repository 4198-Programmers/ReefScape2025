package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class NewLedSubsytem extends SubsystemBase{
    private AddressableLED led;

    public NewLedSubsytem(){
        led = new AddressableLED(0);
        led.setLength(60);
        led.start();
    }

    public void test(){
        System.out.println("test");
    }

    public void RunLEDS() {
        System.out.println("HAII");

    }
    
}
