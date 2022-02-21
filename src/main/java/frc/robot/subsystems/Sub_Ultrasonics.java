/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Sub_Ultrasonics extends SubsystemBase {
  /**
   * Creates a new Sub_Ultrasonics.
   */
  public Ultrasonic left = new Ultrasonic(Constants.ultraLeftPing, Constants.ultraLeftEcho);
  public Ultrasonic right = new Ultrasonic(Constants.ultraRightPing, Constants.ultraRightEcho);

  public double distLeft = 0;
  public double distRight = 0;
  public double angleToWall;

  public double distAvg = 0;
  public double distFromFront = 325;
  // distance between ULT sensors... to be updated!!!
  public double distApart = 533.4;
  public double distDelay;

  public Sub_Ultrasonics() {

  }

  public void setAuto() {
    Ultrasonic.setAutomaticMode(true);
    // right.setAutomaticMode(true);
  }

  public void calcDists() {
    if (RobotBase.isSimulation()) {
      distFromFront = 0;
    }
    distLeft = left.getRangeMM();
    distRight = right.getRangeMM();
    distAvg = (distLeft + distRight) / 2;

    angleToWall = Math.atan((distRight - distLeft) / distApart) / Math.PI * 180;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    calcDists();
    SmartDashboard.putNumber("l_UltDist:", distLeft);
    SmartDashboard.putNumber("r_UltDist:", distRight);
    SmartDashboard.putNumber("UltAngle", angleToWall);
    
    if (distDelay != distAvg) {
      // System.out.println(distRight + " , " + distLeft + " angleToWall = " +
      // angleToWall); //print avg. dist for debug
    }
    distDelay = distAvg;
  }
}
