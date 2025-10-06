package frc.robot.subsystems;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
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


import java.util.Optional;



public class SubsystemManager extends SubsystemBase {
    private final SwerveSubsystem swerveSubsystem;
    private final Elevator elevator;
    private final Arm arm;
    private final BallGrabber ballGrabber;

    private final CommandXboxController controller = new CommandXboxController(0);


    public enum WantedSuperState {
        Stow,
        ScoreL1,
        ScoreL2,
        ScoreL3,
        ScoreL4,
        RemoveBallL2,
        RemoveBallL3,
        ShootBall,
        StationIntake,
        GroundCoralIntake,
        GroundBallIntake
    }

    public enum CurrentSuperState {
        Stow,
        ScoreL1,
        ScoreL2,
        ScoreL3,
        ScoreL4,
        RemoveBallL2,
        RemoveBallL3,
        ShootBall,
        StationIntake,
        GroundCoralIntake,
        GroundBallIntake
    }

    private WantedSuperState wantedSuperState = WantedSuperState.Stow;
    private CurrentSuperState currentSuperState = CurrentSuperState.Stow;
    private CurrentSuperState previousSuperState;


    public SubsystemManager(
        SwerveSubsystem swerve,
        Elevator elevator,
        Arm arm,
        BallGrabber ballGrabber
        ) {
        this.swerveSubsystem = swerve;
        this.elevator = elevator;
        this.arm = arm;
        this.ballGrabber = ballGrabber;
    }

    public void periodic() {
        currentSuperState = handStateTransitions();
        applyStates();
    }

    private CurrentSuperState handStateTransitions() {
        if (previousSuperState != currentSuperState) {
            switch (wantedSuperState) {
                default:
                    currentSuperState = CurrentSuperState.Stow;
                    break;
                case ScoreL1:
                    currentSuperState = CurrentSuperState.ScoreL1;
                    break;
                case ScoreL2:
                    currentSuperState = CurrentSuperState.ScoreL2;
                    break;
                case RemoveBallL2:
                    currentSuperState = CurrentSuperState.RemoveBallL2;
                    break;
                case RemoveBallL3:
                    currentSuperState = CurrentSuperState.RemoveBallL3;
                    break;
                case ShootBall:
                    currentSuperState = CurrentSuperState.ShootBall;
                    break;
                case StationIntake:
                    currentSuperState = CurrentSuperState.StationIntake;
                    break;
                case GroundCoralIntake:
                    currentSuperState = CurrentSuperState.GroundCoralIntake;
                    break;
                case GroundBallIntake:
                    currentSuperState = CurrentSuperState.GroundBallIntake;
                    break;
            }
        }
        return currentSuperState;
    }

    private void applyStates() {
        switch (currentSuperState) {
            case Stow:
                stow();
                break;
            case ScoreL1:
                scoreL1();
                break;
            case ScoreL2:
                scoreL2();
                break;
            case RemoveBallL2:
                removeBallL2();
                break;
            case RemoveBallL3:
                removeBallL3();
                break;
            case ShootBall:
                shootBall();
                break;
            case StationIntake:
                stationIntake();
                break;
            case GroundCoralIntake:
                groundCoralIntake();
                break;
            case GroundBallIntake:
                groundBallIntake();
                break;
        }
    }

    public void stow() {
        if (previousSuperState != CurrentSuperState.Stow) {
            
            wantedSuperState = WantedSuperState.Stow;
            elevator.setGoal(ElevatorConstants.STOW);
            arm.setGoal(ArmConstants.STOW_POSITION_DEG);
        }

        else {
            ballGrabber.stop();
        }
    }

    private void scoreL1() {
/* */
    }

    private void scoreL2() {

    }

    public void removeBallL2() {
        if (previousSuperState != CurrentSuperState.RemoveBallL2) {
            wantedSuperState = WantedSuperState.RemoveBallL2;
            elevator.setGoal(ElevatorConstants.L2BALL);
            arm.setGoal(ArmConstants.REEF_POSITION_DEG_low);
            ballGrabber.runIntake();
        }
    }

    public void removeBallL3() {
        if (previousSuperState != CurrentSuperState.RemoveBallL3) {
            wantedSuperState = WantedSuperState.RemoveBallL3;
            elevator.setGoal(ElevatorConstants.L3BALL);
            arm.setGoal(ArmConstants.REEF_POSITION_DEG_high);
            ballGrabber.runIntake();
        }
    }

    public void shootBall() {
        if (previousSuperState != CurrentSuperState.ShootBall) {
            wantedSuperState = WantedSuperState.ShootBall;
            elevator.setGoal(ElevatorConstants.SHOOTBALL);
            arm.setGoal(ArmConstants.BARGE);
        }
    }

    private void stationIntake() {

    }

    private void groundCoralIntake() {

    }

    public void groundBallIntake() {
        if (previousSuperState != CurrentSuperState.GroundBallIntake) {

            elevator.setGoal(ElevatorConstants.STOW);
            arm.setGoal(ArmConstants.PICKUP_POSITION_DEG);
            ballGrabber.runIntake();
            wantedSuperState = WantedSuperState.GroundBallIntake;
             
        } else if (previousSuperState == CurrentSuperState.GroundBallIntake) {
            if (ballGrabber.hasBall()) {
                wantedSuperState = WantedSuperState.Stow;
                controller.setRumble(RumbleType.kBothRumble, 0.5);
                System.out.println("Has ball");
            } else {
                arm.setGoal(ArmConstants.PICKUP_POSITION_DEG);
                wantedSuperState = WantedSuperState.GroundBallIntake;
                System.out.println("NO BALL`");
            }
            System.out.println("REPEAT");
        }
    }
}
