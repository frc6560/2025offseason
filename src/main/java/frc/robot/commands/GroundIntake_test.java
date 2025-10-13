package frc.robot.commands;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;

import frc.robot.subsystems.swervedrive.*;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.BallGrabber; 

import frc.robot.ManualControls;
import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class GroundIntake_test extends SequentialCommandGroup {
    private final Elevator elevator;
    private final Arm arm;
    private final BallGrabber ballGrabber;

    public GroundIntake_test(Elevator elevator, Arm arm, BallGrabber ballGrabber) {
        this.elevator = elevator;
        this.arm = arm;
        this.ballGrabber = ballGrabber;

        super.addCommands(
            // Start intake process
            Commands.parallel(
                Commands.runOnce(() -> elevator.setGoal(ElevatorConstants.STOW)),
                Commands.runOnce(() -> arm.setGoal(ArmConstants.PICKUP_POSITION_DEG)),
                Commands.runOnce(() -> ballGrabber.runIntake())
            ),
            // Wait until ball is detected
            Commands.waitUntil(() -> ballGrabber.hasBall()),
            // Retract to stow position
            Commands.parallel(
                Commands.runOnce(() -> ballGrabber.stop()),
                Commands.runOnce(() -> arm.setGoal(ArmConstants.STOW_POSITION_DEG))
            )
        );

        super.addRequirements(elevator, arm, ballGrabber);
    }
}