/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import com.analog.adis16470.frc.ADIS16470_IMU.ADIS16470CalibrationTime;
import com.analog.adis16470.frc.ADIS16470_IMU.IMUAxis;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpiutil.math.numbers.N2;
//import io.github.pseudoresonance.pixy2api.links.SPILink;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    // Controller Index
    //  /*
    public static final int controller = 0;
    public static final int joy_left_y = 1;
    public static final int joy_right_x = 4;
    public static final int trigger_left = 2;
    public static final int trigger_right = 3;
    public static final int button_A = 1;
    public static final int button_B = 2;
    public static final int button_X = 3;
    public static final int button_Y = 4;
    public static final int button_LeftBumper = 5;
    public static final int button_RightBumper = 6;
    public static final int button_Back = 7;
    public static final int button_Start = 8;
    public static final int button_joy_left = 9;
    public static final int button_joy_right = 10;
    //  */

    // RobotSimulation Controller Mapping (X)
      /*
    public static final int controller = 0; 
    public static final int joy_left_x = 0; 
    public static final int joy_left_y = 3; 
    public static final int joy_right_x = 1; 
    public static final int joy_right_y = 4; 
    public static final int trigger_left = 2; 
    public static final int trigger_right = 5;
    public static final int button_A = 1; 
    public static final int button_B = 2; 
    public static final int button_X = 3; 
    public static final int button_Y = 4; 
    public static final int button_LeftBumper = 5; 
    public static final int button_RightBumper = 6; 
    public static final int button_joy_left = 7; 
    public static final int button_joy_right = 8; 
    public static final int button_Start = 9; 
    public static final int button_Back = 10; 
    public static final int button_Center = 11;
      */

    // Universal constants: No difference between simulation and actual robot drive.
    // Use "RobotContainer.stick.getPOV(Constants.controller)" to get DPad angle
    // button Convey.java checks to reverse conveyor

    public static final int bttn_reverse = button_RightBumper;

    // Drivetrain ports
    public static final int m_leftFront = 0;
    public static final int m_leftBack = 1;
    public static final int m_rightFront = 2;
    public static final int m_rightBack = 3;

    // On-board Inputs
    public static final int ultraLeftPing = 0;
    public static final int ultraLeftEcho = 1;
    public static final int ultraRightPing = 2;
    public static final int ultraRightEcho = 3;
    public static final int encoder1A = 4;
    public static final int encoder1B = 5;
    public static final int encoder2A = 6;
    public static final int encoder2B = 7;
    public static final SPI.Port imuPort = SPI.Port.kOnboardCS0;
    public static final I2C.Port colourPort = I2C.Port.kOnboard;
    //public static final SPILink cameraPort = new SPILink();

    //SendableChooser values
    public static int autoRoute;
    public static final int aRouteSlaDR = 0;
    public static final int aRouteBrrlDR = 1;
    public static final int aRouteHmpDR = 2;
    public static final int aRouteGSch_ABE = 3;
    public static final int aRouteGSch_ARD = 4;
    public static final int aRouteGSch_BBE = 5;
    public static final int aRouteGSch_BRD = 6;
    public static final int aRouteBrrlDRsml = 7;

    public static int autoSpeed;
    public static final int teleOperation = 0;
    public static final int aRouteSpd_One = 1;
    public static final int aRouteSpd_Two = 2;
    public static final int aRouteSpd_Three = 3;

    // PathWeaver trajectory file locations
    public static final String[] traJAutoPaths = {
        "SlalomPath.wpilib.json", 
        "BarrelPath.wpilib.json", 
        "BouncePath.wpilib.json", 
        "GlArouteBE.wpilib.json",
        "GIArouteRD.wpilib.json",
        "GlBrouteBE.wpilib.json",
        "GlBrouteRD.wpilib.json",
        "SmallTestPathBR.wpilib.json"
    };

    public static final String[] traJAutoSpd = {
        "teleOperation",
        "paths/threeSixKMperH/",
        "paths/sevenTwoKMperH/",
        "paths/tenEightKMperH/"
    };


    // Units
    public static final double shootDist = 370;
    public static final double conveySpeed = 0.3;
    public static final double convey_manualSpeed = 0.4;

    public static final IMUAxis gyroAxis = IMUAxis.kZ;
    public static final ADIS16470CalibrationTime CalTime = ADIS16470CalibrationTime._4s;

    public static final Color RD = ColorMatch.makeColor(0.561, 0.232, 0.114);
    public static final Color GN = ColorMatch.makeColor(0.197, 0.561, 0.240);
    public static final Color BE = ColorMatch.makeColor(0.143, 0.427, 0.429);
    public static final Color YW = ColorMatch.makeColor(0.361, 0.524, 0.113);


    // This is a for a test of the difference between ultrasonic distances relative
    // to their average
    public static final double approach_angleThreshold = 0.05;

    
    // Characterization toolsuite feedback and feedforward gain values for FrameBot.
    // Real DriveTrain values (important)
    public static final double ksVolts = 2.01;
    public static final double kVelocityLinear = 2.96;
    public static final double kAccelerationLinear = 0.31;
    public static final double kPDriveVel = 1.95;
    public static final double trackWidth = 0.6257635; // Trackwidth in Meters

    
    // Simulation Drive values (unimportant) 
    public static final double kVelocityAngular = 1.5;  // 2.5;
    public static final double kAccelerationAngular = 0.0155;
    public static final DCMotor motorConfig = DCMotor.getCIM(2);

    public static final double gearingConfig = 10.71; // 10.71:1 gearing reduction.
    public static final double wheelRadius = 0.0762; // Wheel radius in Meters

    // Ramsete follower paramaters
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    // Chassis speed to wheel speed conversion
    public static DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(trackWidth);

    // Robot characterization values for simutation
    public static LinearSystem<N2, N2, N2> kDrivetrainPlant =
    LinearSystemId.identifyDrivetrainSystem(kVelocityLinear, 
        kAccelerationLinear, kVelocityAngular, kAccelerationAngular);

}
