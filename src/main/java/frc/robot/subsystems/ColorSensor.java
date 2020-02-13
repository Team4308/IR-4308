package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import bbb.wrapper.LogSubsystem;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;

public class ColorSensor extends LogSubsystem {
    
    private final I2C.Port i2cPort = I2C.Port.kOnboard;

    private ColorSensorV3 m_ColorSensorV3 = new ColorSensorV3(i2cPort);

    private ColorMatch m_ColorMatch = new ColorMatch();

    public ColorSensor() {
        m_ColorMatch.addColorMatch(Constants.Config.ColorSensor.kBlueTarget);
        m_ColorMatch.addColorMatch(Constants.Config.ColorSensor.kGreenTarget);
        m_ColorMatch.addColorMatch(Constants.Config.ColorSensor.kRedTarget);
        m_ColorMatch.addColorMatch(Constants.Config.ColorSensor.kYellowTarget);
    }

    public ColorMatchResult getColorMatch() {
        Color detectedColor = m_ColorSensorV3.getColor();

        return m_ColorMatch.matchClosestColor(detectedColor);
    }

    public String getColorDetected() {
        ColorMatchResult match = getColorMatch();

        if (match.color == Constants.Config.ColorSensor.kBlueTarget) {
            return "blue";
        } else if (match.color == Constants.Config.ColorSensor.kGreenTarget) {
            return "green";
        } else if (match.color == Constants.Config.ColorSensor.kRedTarget) {
            return "red";
        } else {
            return "yellow";
        }
    }

    @Override
    public Sendable log() {
        Shuffleboard.getTab("Log").addString("Color Detected", () -> getColorDetected());
        Shuffleboard.getTab("Log").addNumber("Color Confidence", () -> getColorMatch().confidence);
        return this;
    }
}