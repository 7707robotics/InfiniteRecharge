/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class PIDLinearCommand extends CommandBase {
  /**
   * Creates a new PIDLinearCommand.
   */
  Encoder encoder;
  PIDController controller;
  DriveSubsystem driveSubsystem;
  DifferentialDrive drive;
  public PIDLinearCommand(Encoder encoder, double setpoint, double Kp, double Ki, double Kd) {
    this.encoder = encoder;
    this.controller = new PIDController(Kp, Ki, Kd);
    controller.setSetpoint(setpoint);

    //Temp:
    SpeedControllerGroup leftController = new SpeedControllerGroup(new PWMVictorSPX(2), new PWMVictorSPX(3));
    SpeedControllerGroup rightController = new SpeedControllerGroup(new PWMVictorSPX(0), new PWMVictorSPX(1));
    this.drive = new DifferentialDrive(leftController, rightController);
    //end of temp

    driveSubsystem = new DriveSubsystem(
      () -> controller.calculate(encoder.getDistance()), 
      () -> 0.0,
      drive
    );
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.drive();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
