// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import java.io.File;
import swervelib.SwerveInputStream;

import frc.robot.subsystems.BallGrabber;
import frc.robot.commands.BallGrabberCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  final XboxController firstXbox = new XboxController(0); 
  final XboxController secondxbox = new XboxController(1);
  final ManualControls manualControls = new ManualControls(firstXbox, secondxbox);

  private final BallGrabber ballGrabber; 
  private final BallGrabberCommand ballGrabberCommand;

  
  public RobotContainer(){

    ballGrabber = new BallGrabber();
    ballGrabberCommand = new BallGrabberCommand(ballGrabber, manualControls); 
    ballGrabber.setDefaultCommand(ballGrabberCommand);
  }
}
