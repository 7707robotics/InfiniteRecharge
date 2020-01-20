/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.RobotMap;
import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PIDTurnCommand extends CommandBase {
  /**
   * Creates a new PIDTurnCommand.
   */
  GyroBase gyro;
  PIDController controller;
  DriveSubsystem driveSubsystem;

    public PIDTurnCommand(double setpoint) {
    gyro = RobotMap.gyro;
    controller = new PIDController(RobotMap.Kp_TURN, RobotMap.Ki_TURN, RobotMap.Kd_TURN);
    driveSubsystem = new DriveSubsystem();

    controller.setSetpoint(setpoint);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  public void setSetpoint(double setpoint) {
    controller.setSetpoint(setpoint);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Calculates the necessary speed using PID and runs the turn method
    driveSubsystem.turn(() -> controller.calculate(gyro.getAngle()));
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
