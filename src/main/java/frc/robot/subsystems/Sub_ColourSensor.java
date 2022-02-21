// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
  /*
import java.util.ArrayList;
import io.github.pseudoresonance.pixy2api.*;
import edu.wpi.first.wpilibj.RobotBase;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
  */

public class Sub_ColourSensor extends SubsystemBase {
  /** Creates a new Sub_ColourSensor. */


private final ColorMatch tCreference = new ColorMatch();
private ColorMatchResult colourName = null;
private Color colourResult = null;
public ColorSensorV3 lightSensor;

  /*
private boolean isCamera = false;
private int state = -1;
public Pixy2 lightCam;
  */

private int ballCount = 0;
private int count = 0;

public String colorString;
public int proximity;


  public Sub_ColourSensor() {
    lightSensor = new ColorSensorV3(Constants.colourPort);
    //lightCam = Pixy2.createInstance(Constants.cameraPort);
  }

  public void setColorValues(Color RD, Color GN, Color BE, Color YW) {
    tCreference.addColorMatch(RD);
    tCreference.addColorMatch(GN);
    tCreference.addColorMatch(BE);
    tCreference.addColorMatch(YW);
  }

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run

    // if no camera present, try to initialize
    /*
    if(!isCamera) {
       state = lightCam.init(4);
       if (RobotBase.isReal()) {
       lightCam.setLamp((byte) 1, (byte) 1); // Turns the LEDs on
       lightCam.setLED(0, 255, 0); // Sets the RGB LED to full green
       }
    }
    isCamera = state >= 0;

    if (isCamera) {

    //publish if we are connected
      SmartDashboard.putBoolean("PixyCam_Connection:", isCamera);

    //run getBlocks with arguments to have the camera
      lightCam.getCCC().getBlocks(false, 255, 255); 

    //acquire target data 
      ArrayList<Block> blocks = lightCam.getCCC().getBlockCache();

    //assign the data to an ArrayList for convinience
    if(blocks.size() > 0) {
      double xcoord = blocks.get(0).getX(); // x position of the largest target
      double ycoord = blocks.get(0).getY(); // y position of the largest target
      String data = blocks.get(0).toString(); // string containing target info
      SmartDashboard.putBoolean("locatedTarget", true); // show there is a target present
      SmartDashboard.putNumber("targetCoordX:",xcoord); 
      SmartDashboard.putNumber("targetCoordY:", ycoord); 
      SmartDashboard.putString("Data", data );
    } else  SmartDashboard.putBoolean("present", false);

    //push to dashboard how many targets are detected
      SmartDashboard.putNumber("locatedTarget", blocks.size());
    } //  */

    colourResult = lightSensor.getColor();
    proximity = lightSensor.getProximity();

    //Run the color match algorithm on our detected color
      colourName = tCreference.matchClosestColor(colourResult);

    if (colourName.color == Constants.YW) {
      colorString = "Yellow";
      if (count < 1) {
        count++;
        ballCount++;
      }
    } else if (colourName.color == Constants.RD) {
      colorString = "Red";
    } else if (colourName.color == Constants.GN) {
      colorString = "Green";
    } else if (colourName.color == Constants.BE) {
      colorString = "Blue";
    } else {
      colorString = "Unknown";
    }

    if (count == 1 && colourName.color != Constants.YW) count = 0;

    SmartDashboard.putNumber("Proximity", proximity);
    SmartDashboard.putNumber("Confidence", colourName.confidence);
    SmartDashboard.putNumber("ballCount@", ballCount);
    SmartDashboard.putString("Detected Color", colorString);

  }
}
