package frc.robot.commands;

import frc.robot.ManualControls;

import frc.robot.subsystems.BallGrabber;

import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

import frc.robot.Constants;
import frc.robot.subsystems.SubsystemManager;

import java.util.Optional;

public class SubsystemManagerCommand extends Command{

    private final SwerveSubsystem swerveSubsystem;
    private final Elevator elevator;
    private final Arm arm;
    private final BallGrabber ballGrabber;

    private final ManualControls controls;
    private final SubsystemManager subsystemManager;


    public SubsystemManagerCommand(
        SwerveSubsystem swerve,
        Elevator elevator,
        Arm arm,
        BallGrabber ballGrabber,
        ManualControls controls,
        SubsystemManager subsystemManager
        ) {
        this.swerveSubsystem = swerve;
        this.elevator = elevator;
        this.arm = arm;
        this.ballGrabber = ballGrabber;
        this.controls = controls;
        this.subsystemManager = subsystemManager;

        addRequirements(subsystemManager);
    }
    


    public void initialize() {
        ballGrabber.stop();
    }

    public void execute() {

        if (controls.goToStow()) { //A

            subsystemManager.stow();

        } else if (controls.goToL2Ball()) { //X

            subsystemManager.removeBallL2();

        } else if (controls.goToL3Ball()) { //B

            subsystemManager.removeBallL3();

        } else if (controls.goToShootBall()) { //Y

            subsystemManager.shootBall();

        } else if (controls.goToGroundBall()) { //Back Button (3 lines)

            subsystemManager.groundBallIntake();
        }

        if (controls.runGrabberIntake()) { //Left Trigger
            ballGrabber.runIntake();

        } else if (controls.runGrabberOuttake()) { //Right Trigger
            ballGrabber.runOuttake();
        }
    }
}
