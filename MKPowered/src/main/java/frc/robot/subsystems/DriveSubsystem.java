/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  private DifferentialDrive drive;
  private DoubleSupplier forward, turn;

  public DriveSubsystem(DifferentialDrive drive, DoubleSupplier forward, DoubleSupplier turn) {
    this.drive = drive;
    this.forward = forward;
    this.turn = turn;
  }

  public void drive() {
    
    drive.arcadeDrive(forwardAxis, turnAxis, true);
    drive.setSafetyEnabled(false);
  }

  public void driveStop() {
    drive.arcadeDrive(0, 0, true)
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
