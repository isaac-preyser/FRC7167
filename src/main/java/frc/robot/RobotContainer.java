/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.AutoGsearch;
import frc.robot.commands.DegreesRotate;
import frc.robot.commands.ManualOverride;
import frc.robot.commands.ToggleDriveMode;
import frc.robot.commands.ToggleSpeed;
import frc.robot.subsystems.Sub_ColourSensor;
import frc.robot.subsystems.Sub_DriveTrain;
import frc.robot.subsystems.Sub_Encoders;
import frc.robot.subsystems.Sub_Gyroscope;
import frc.robot.subsystems.Sub_RobotOD;
import frc.robot.subsystems.Sub_Ultrasonics;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // Don't know what this really is but it fixes issues. Don't mess with it.
  public static PowerDistributionPanel powerDist = new PowerDistributionPanel();
  public static SendableChooser<Integer> choose_autoRouteDR = new SendableChooser<>();
  public static SendableChooser<Integer> choose_autoRouteSpd = new SendableChooser<>();

  // Universal Robot subsystem instantiations
  public static Sub_DriveTrain driveTrain = new Sub_DriveTrain();
  public static Sub_Encoders encoders = new Sub_Encoders();
  public static Sub_Gyroscope gyroscope = new Sub_Gyroscope();
  public static Sub_RobotOD odometry = new Sub_RobotOD();
  public static AutoGsearch autoSearch = new AutoGsearch();
  public static Sub_Ultrasonics ultrasonics = new Sub_Ultrasonics();
  public static Sub_ColourSensor colourSensor = new Sub_ColourSensor();

  // Universal Robot control config
  public static Joystick stick = new Joystick(Constants.controller);

  public JoystickButton manual = new JoystickButton(stick, Constants.button_Back);
  
  public JoystickButton revHeading = new JoystickButton(stick, Constants.button_A);
  public JoystickButton drive_speedToggle = new JoystickButton(stick, Constants.button_B);
  public JoystickButton alignParallel = new JoystickButton(stick, Constants.button_X);

  public POVButton turn0 = new POVButton(stick, 0);
  public POVButton turn90 = new POVButton(stick, 90);
  public POVButton turn180 = new POVButton(stick, 180);
  public POVButton turn270 = new POVButton(stick, 270);

  // RobotState
  public static boolean isTeleop = false;
  public static boolean autoEnd = false;


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Autonomous Chooser Instantiations
    choose_autoRouteSpd.setDefaultOption("TeleOp Mode", Constants.teleOperation);
    choose_autoRouteSpd.addOption("3.6 Km/h TargetSpd", Constants.aRouteSpd_One);
    choose_autoRouteSpd.addOption("7.2 Km/h TargetSpd", Constants.aRouteSpd_Two);
    choose_autoRouteSpd.addOption("10.8 Km/h TargetSpd", Constants.aRouteSpd_Three);

    choose_autoRouteDR.setDefaultOption("Slalom Route", Constants.aRouteSlaDR);
    choose_autoRouteDR.addOption("Barrel Route", Constants.aRouteBrrlDR);
    choose_autoRouteDR.addOption("Bounce Route", Constants.aRouteHmpDR);
    choose_autoRouteDR.addOption("GS Route 'Blue A'", Constants.aRouteGSch_ABE);
    choose_autoRouteDR.addOption("GS Route 'Red A'", Constants.aRouteGSch_ARD);
    choose_autoRouteDR.addOption("GS Route 'Blue B'", Constants.aRouteGSch_BBE);
    choose_autoRouteDR.addOption("GS Route 'Red B'", Constants.aRouteGSch_BRD);
    choose_autoRouteDR.addOption("SmallBarrel Route", Constants.aRouteBrrlDRsml);

    SmartDashboard.putData(choose_autoRouteSpd);
    SmartDashboard.putData(choose_autoRouteDR);
    SmartDashboard.putString("Note On Running Autonomous: ", "Select TeleOp Mode between Speed Changes");
  }

  public static void setAutoEnd(Boolean Aend) {
    autoEnd = Aend;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
    drive_speedToggle.whenPressed(new ToggleSpeed());
    revHeading.whenPressed(new ToggleDriveMode());
    
    manual.whenPressed(new ManualOverride());

    if (RobotBase.isReal()) {
      turn0.whenPressed(new DegreesRotate(0.6, 0));
      turn90.whenPressed(new DegreesRotate(0.6, 90));
      turn180.whenPressed(new DegreesRotate(0.6, 180));
      turn270.whenPressed(new DegreesRotate(0.6, 270));
    }
  
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoSearch;
  }

}
