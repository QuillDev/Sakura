/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.TeleopSwerveDriveCommand;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public static final DriveSubsystem SWERVE_DRIVE = new DriveSubsystem();
  public static final Joystick CONTROLS = new Joystick(0);

  public RobotContainer() {

    if (RobotBase.isReal()) {

      SWERVE_DRIVE.setDefaultCommand(new TeleopSwerveDriveCommand());

      // Zero Gyro Command
      new JoystickButton(CONTROLS, Button.kA.value).whenPressed(() -> SWERVE_DRIVE.zeroGyro());

      // Zero Azimuths Command
      new JoystickButton(CONTROLS, Button.kB.value).whenPressed(() -> SWERVE_DRIVE.zeroAzimuths());

      // Save Azimuth zeroes Command
      new JoystickButton(CONTROLS, Button.kX.value).whenPressed(() -> SWERVE_DRIVE.saveAzimuthPositions());
    }
  }

}
