/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.RobotMap;
import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PIDLinearCommand extends CommandBase {
  /**
   * Creates a new PIDLinearCommand.
   */
  Encoder encoder;
  PIDController controller;
  DriveSubsystem driveSubsystem;
  public PIDLinearCommand(double setpoint) {
    encoder = RobotMap.encoder;
    controller = new PIDController(RobotMap.Kp_FORWARD, RobotMap.Ki_FORWARD, RobotMap.Kd_FORWARD);
    controller.setSetpoint(setpoint);

    driveSubsystem = new DriveSubsystem(); 
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Calculates the necessary speed using PID and runs the forward method
    driveSubsystem.forward(() -> controller.calculate(encoder.getDistance()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }
}
