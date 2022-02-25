// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drive drive = new Drive();

  private Joystick joystick = new Joystick(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    SensitivityController forwardSensitivityController = new SensitivityController();
    SensitivityController rotationalSensitivityController = new SensitivityController();

    SlewRateLimiter slewFilter = new SlewRateLimiter(3); // 3 units per second

    SmartDashboard.putNumber("F Positive Sensitivity", -40.0);
    SmartDashboard.putNumber("F Negative Sensitivity", -40.0);

    drive.setDefaultCommand(new RunCommand(() -> {
      double forward = joystick.getRawAxis(1);
      double lateral = joystick.getRawAxis(0);
      double rotational = joystick.getRawAxis(2);

      forwardSensitivityController.setPercentageSensitivity(
          SmartDashboard.getNumber("F Positive Sensitivity", -40.0),
          SmartDashboard.getNumber("F Negative Sensitivity", -40.0)
      );
      rotationalSensitivityController.setPercentageSensitivity(
          SmartDashboard.getNumber("R Positive Sensitivity", -40.0),
          SmartDashboard.getNumber("R Negative Sensitivity", -40.0)
      );

      forward = forwardSensitivityController.calculate(forward);
      forward = slewFilter.calculate(forward);
      rotational = rotationalSensitivityController.calculate(rotational);

      drive.drive(forward, lateral, rotational);
    }, drive));
  }
}
