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

public class ShootBall extends SequentialCommandGroup {

    private Arm arm;
    private Elevator elevator;
    
    public ShootBall( Elevator elevator, Arm arm, BallGrabber ballGrabber) {
    this.elevator = elevator;
    this.arm = arm;
    super.addCommands(
        actuateCommand()
    );
  }
  public Command actuateCommand() {
    return new ParallelCommandGroup(
        new RunCommand(() -> elevator.setGoal(ElevatorConstants.SHOOTBALL), elevator),
        new RunCommand(() -> arm.setGoal(ArmConstants.BARGE), arm)
    );
  }
  }