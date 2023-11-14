// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorSensor extends SubsystemBase {
  /** Creates a new ColorSensor. */
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

  private final ColorMatch m_colorMatcher = new ColorMatch();

  
  public ColorSensor() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    System.out.println("ColorSensorRunning");
    Color detectedColor = m_colorSensor.getColor();

    String colorString = "NoColor";
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

   

    if (match.color == Constants.ColorConstants.kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == Constants.ColorConstants.kRedTarget) {
      colorString = "Red";
    } else if (match.color == Constants.ColorConstants.kGreenTarget) {
      colorString = "Green";
    } else if (match.color == Constants.ColorConstants.kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }
    System.out.println("Red: " + m_colorSensor.getRed() + "Green:" + m_colorSensor.getGreen() + "Blue: " + m_colorSensor.getBlue());
  }
}
