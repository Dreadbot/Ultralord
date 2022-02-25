// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
  private CANSparkMax leftFrontMotor = new CANSparkMax(10, MotorType.kBrushless);
  private CANSparkMax rightFrontMotor = new CANSparkMax(1, MotorType.kBrushless);
  private CANSparkMax leftBackMotor = new CANSparkMax(2, MotorType.kBrushless);
  private CANSparkMax rightBackMotor = new CANSparkMax(3, MotorType.kBrushless);

  private AHRS ahrs;

  private MecanumDrive mecanumDrive;

  /** Creates a new ExampleSubsystem. */
  public Drive() {
    leftFrontMotor.restoreFactoryDefaults();
    rightFrontMotor.restoreFactoryDefaults();
    leftBackMotor.restoreFactoryDefaults();
    rightBackMotor.restoreFactoryDefaults();

    ahrs = new AHRS(I2C.Port.kMXP);

    rightFrontMotor.setInverted(true);
    rightBackMotor.setInverted(true);

    leftFrontMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    rightFrontMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    leftBackMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    rightBackMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

    mecanumDrive = new MecanumDrive(leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor);
  }

  public void drive(double joystickForward, double joystickLateral, double joystickRotational) {
    mecanumDrive.driveCartesian(joystickForward, -joystickLateral, -joystickRotational);
  }
}
