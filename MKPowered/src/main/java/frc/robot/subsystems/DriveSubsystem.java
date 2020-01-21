/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.RobotMap;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  private DifferentialDrive drive;
  private GenericHID driverInput;

  public DriveSubsystem() {
    drive = RobotMap.drive;
    drive.setSafetyEnabled(false);
    driverInput = RobotMap.driverInput;
  }

  public void teleopDrive() {
    
    drive.arcadeDrive( -0.6*driverInput.getRawAxis(1), 0.5*driverInput.getRawAxis(0), true);
  }

  public void forward(DoubleSupplier speed) {
    drive.arcadeDrive(speed.getAsDouble(), 0);
  }

  public void turn(DoubleSupplier rotation) {
    drive.arcadeDrive(0, rotation.getAsDouble());
  }

  public void driveStop() {
    drive.arcadeDrive(0, 0, true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
