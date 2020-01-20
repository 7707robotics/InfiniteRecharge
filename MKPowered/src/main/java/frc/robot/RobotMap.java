/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * Add your docs here.
 */



public class RobotMap {

    /* * * * * * * *
     *   Devices   *
     * * * * * * * */

    //Sensors
    public static final Encoder encoder = new Encoder(0, 1);
    public static final GyroBase gyro = new AnalogGyro(0);
    
    //Speed Controllers

    public static final SpeedController frontLeftController = new PWMVictorSPX(2);
    public static final SpeedController backLeftController = new PWMVictorSPX(3);
    public static final SpeedController frontRightController = new PWMVictorSPX(0);
    public static final SpeedController backRightController = new PWMVictorSPX(1);
    public static final SpeedControllerGroup leftControllers = new SpeedControllerGroup(frontLeftController, backLeftController);
    public static final SpeedControllerGroup rightControllers = new SpeedControllerGroup(frontRightController, backRightController);

    //Driver Controllers
    public static final GenericHID driverInput = new XboxController(0);

    /* * * * * * * *
     *   Systems   *
     * * * * * * * */

    //Drive Train
    public static final DifferentialDrive drive = new DifferentialDrive(leftControllers, rightControllers);
    
    /* * * * * * * *
     *  Constants  *
     * * * * * * * */

    //PID Forward
    public static final double Kp_FORWARD = 0.0;
    public static final double Ki_FORWARD = 0.0;
    public static final double Kd_FORWARD = 0.0;

    //PID Turning
    public static final double Kp_TURN = 0.0;
    public static final double Ki_TURN = 0.0;
    public static final double Kd_TURN = 0.0;
}
