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

    public void stow() {
        elevator.setWantedState(Elevator.WantedState.Stow);
        elevator.setGoal(ElevatorConstants.STOW);

        arm.setArmGoal(ArmConstants.STOW_POSITION_DEG);
        
        ballGrabber.stop();
        

    }

    private void scoreL1() {
/* */
    }

    private void scoreL2() {

    }

    public void removeBallL2() {
        elevator.setWantedState(Elevator.WantedState.L2Ball);
        elevator.setGoal(ElevatorConstants.L3BALL);

        arm.setArmGoal(ArmConstants.REEF_POSITION_DEG_low);
        
        ballGrabber.runIntakeOuttake();

    }

    public void removeBallL3() {
        elevator.setWantedState(Elevator.WantedState.L3Ball);
        elevator.setGoal(ElevatorConstants.L3BALL);

        arm.setArmGoal(ArmConstants.REEF_POSITION_DEG_high);
        
        ballGrabber.runIntakeOuttake();

    }

    public void shootBall() {
        elevator.setWantedState(Elevator.WantedState.ShootBall);
        elevator.setGoal(ElevatorConstants.SHOOTBALL);

        arm.setArmGoal(ArmConstants.BARGE);

    }

    private void stationIntake() {

    }

    private void groundCoralIntake() {

    }

    public void groundBallIntake() {

        if (ballGrabber.hasBall()) {
            arm.setArmGoal(ArmConstants.STOW_POSITION_DEG);
        } else {
            arm.setArmGoal(ArmConstants.GPICKUP_POSITION_DEG); //Should hopefully work, updated
        }

        elevator.setWantedState(Elevator.WantedState.Stow);
        elevator.setGoal(ElevatorConstants.STOW);
        ballGrabber.runIntakeOuttake();
    }
}
