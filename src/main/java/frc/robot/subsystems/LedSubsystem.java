package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LedSubsystem extends SubsystemBase {
    
    private static final String LedBuffer = null;

    private static final String m_ledBuffer = null;

    private AddressableLED driveBaseLED = new AddressableLED(Constants.DRIVE_BASE_LED_ID);

    // length(num of stips)
    private AddressableLEDBuffer driveBaseLEDBuffer = new AddressableLEDBuffer(Constants.AddressableLEDBuffer);

    public static int m_rainbowFirstPixelHue = 180;

    private Optional<Alliance> alliance;

    private boolean circleMode = false;

    public void LEDSubsystem() {
        driveBaseLED.setLength(driveBaseLEDBuffer.getLength());
        driveBaseLED.setData(driveBaseLEDBuffer);
        driveBaseLED.start();
        this.alliance = DriverStation.getAlliance();
    }

    public Command getDefaultCommand() {
        return runOnce(
                () -> {
                    //System.out.println("Led set up");
                    Alliance currentAlliance = this.alliance.get();
                    if (Alliance.Red == currentAlliance) {
                        this.setRGBColor(Color.kRed);
                    } else if (Alliance.Blue == currentAlliance) {
                        this.setRGBColor(Color.kBlue);
                    } else {
                        this.setRGBColor(Color.kPurple);
                    }
                });
    }

    private int circleDivisor = 10;

    private int circlePostionOn = 0;

    private Color currentColor = Color.kBlack;

    private void RGBCircle() {
        for (var i = 0; i < driveBaseLEDBuffer.getLength(); i++) {
            int currentLightPosition = i % this.circleDivisor;

            if (currentLightPosition == this.circlePostionOn) {
                driveBaseLEDBuffer.setLED(i, this.currentColor);
            } else {
                driveBaseLEDBuffer.setLED(i, Color.kBlack);
            }
        }
    }

    public Command setColorCommand(Color color) {
        return runOnce(() -> {
            System.out.println("Setting Color: " + color.toHexString());
            setColorCommand(color);
        });
    }

    private Command circleSillyTime(boolean StupidIdiot) {
        return runOnce(
                () -> {
                    circleMode = StupidIdiot;
                });
    }

    // setHSV() -> color saturation

    public void setRGBColor(Color color) {
        this.currentColor = color;
        for (var i = 0; i < driveBaseLEDBuffer.getLength(); i++) {
            this.driveBaseLEDBuffer.setLED(i, this.currentColor);
        }
        this.driveBaseLED.setData(this.driveBaseLEDBuffer);
    }
    // }

    public void changingColor() {
        // red = 0-60
        // yellow 61-120
        // green 121-180
        // cyan 181-240
        // blue 241-300
        // magenta 301-360
        for (var i = 0; i < LedBuffer.length(); i++) {

            final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.length()));

            driveBaseLEDBuffer.setHSV(i, hue, 255, 128);
        }

        m_rainbowFirstPixelHue += 3;

        m_rainbowFirstPixelHue %= 180;

    }

}    
