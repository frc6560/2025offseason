package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.BallGrabber;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ArmConstants;
import java.io.File;

public class RobotContainer {

    // Controllers
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    // Subsystems
    private final Elevator elevator = new Elevator();
    private final Arm arm = new Arm();
    private final BallGrabber ballGrabber = new BallGrabber();
    private final SwerveSubsystem drivebase = new SwerveSubsystem(
        new File(Filesystem.getDeployDirectory(), "swerve/falcon")
    );

    public RobotContainer() {
        configureDefaultCommands();
        configureButtonBindings();
    }

    private void configureDefaultCommands() {
        // Only drivetrain needs a default command for continuous control
        drivebase.setDefaultCommand(
            drivebase.driveCommand(
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX()
            )
        );
        
        // NO default commands for elevator, arm, or ballGrabber
        // They are controlled by button bindings only
    }

    private void configureButtonBindings() {
        // ===== OPERATOR CONTROLLER BINDINGS =====
        
        // ----- CLICK ONCE BEHAVIORS (onTrue) -----
        
        // A Button: Stow position
        operatorController.a().onTrue(
            Commands.runOnce(() -> {
                elevator.setWantedState(Elevator.WantedState.Stow);
                elevator.setGoal(ElevatorConstants.STOW);
                arm.setArmGoal(ArmConstants.STOW_POSITION_DEG);
                ballGrabber.stop();
            }, elevator, arm, ballGrabber)
        );
        
        // X Button: L2 Ball scoring position
        operatorController.x().onTrue(
            Commands.runOnce(() -> {
                elevator.setWantedState(Elevator.WantedState.L2Ball);
                elevator.setGoal(ElevatorConstants.L2BALL);
                arm.setArmGoal(ArmConstants.REEF_POSITION_DEG_low);
                ballGrabber.runIntakeOuttake();
            }, elevator, arm, ballGrabber)
        );
        
        // B Button: L3 Ball scoring position
        operatorController.b().onTrue(
            Commands.runOnce(() -> {
                elevator.setWantedState(Elevator.WantedState.L3Ball);
                elevator.setGoal(ElevatorConstants.L3BALL);
                arm.setArmGoal(ArmConstants.REEF_POSITION_DEG_high);
                ballGrabber.runIntakeOuttake();
            }, elevator, arm, ballGrabber)
        );
        
        // Y Button: Shoot ball position
        operatorController.y().onTrue(
            Commands.runOnce(() -> {
                elevator.setWantedState(Elevator.WantedState.ShootBall);
                elevator.setGoal(ElevatorConstants.SHOOTBALL);
                arm.setArmGoal(ArmConstants.BARGE);
            }, elevator, arm)
        );
        
        // Back Button: Ground ball pickup (click to start)
        operatorController.back().onTrue(
            Commands.runOnce(() -> {
                if (ballGrabber.hasBall()) {
                    arm.setArmGoal(ArmConstants.STOW_POSITION_DEG);
                } else {
                    arm.setArmGoal(ArmConstants.GPICKUP_POSITION_DEG);
                }
                elevator.setWantedState(Elevator.WantedState.Stow);
                elevator.setGoal(ElevatorConstants.STOW);
                ballGrabber.runIntakeOuttake();
            }, elevator, arm, ballGrabber)
        );
        
        // Left Bumper: Place position (if you implement it)
        operatorController.leftBumper().onTrue(
            Commands.runOnce(() -> {
                // Add your place position logic here
                // arm.setArmGoal(ArmConstants.PLACE_POSITION_DEG);
            }, arm)
        );
        
        // ----- HOLD BEHAVIORS (whileTrue) -----
        
        // Right Trigger: Ground pickup - HOLD to run intake
        operatorController.rightTrigger(0.25).whileTrue(
            Commands.run(() -> {
                if (ballGrabber.hasBall()) {
                    arm.setArmGoal(ArmConstants.STOW_POSITION_DEG);
                } else {
                    arm.setArmGoal(ArmConstants.GPICKUP_POSITION_DEG);
                }
                elevator.setWantedState(Elevator.WantedState.Stow);
                elevator.setGoal(ElevatorConstants.STOW);
                ballGrabber.runIntakeOuttake();
            }, elevator, arm, ballGrabber)
            .finallyDo(() -> ballGrabber.stop())
        );
        
        // Left Trigger: Manual grabber control - HOLD to run
        operatorController.leftTrigger(0.25).whileTrue(
            Commands.run(() -> {
                ballGrabber.runIntakeOuttake();
            }, ballGrabber)
            .finallyDo(() -> ballGrabber.stop())
        );
        
        // Right Stick Y-axis: Climb controls - HOLD to climb
        // Climb DOWN (right stick pushed forward, Y > 0.7)
        operatorController.axisGreaterThan(5, 0.7).whileTrue(
            Commands.run(() -> {
                // Climb down - adjust speed as needed
                double climbSpeed = -0.3; // Negative to go down
                // You'll need to add a manual control method to Elevator
                // For now, this is a placeholder - implement in Elevator.java
                elevator.getElevLeft().set(climbSpeed);
                elevator.getElevRight().set(climbSpeed);
            }, elevator)
            .finallyDo(() -> {
                elevator.stopElev();
            })
        );
        
        // Climb UP (right stick pulled back, Y < -0.7)
        operatorController.axisLessThan(5, -0.7).whileTrue(
            Commands.run(() -> {
                // Climb up - adjust speed as needed
                double climbSpeed = 0.3; // Positive to go up
                elevator.getElevLeft().set(climbSpeed);
                elevator.getElevRight().set(climbSpeed);
            }, elevator)
            .finallyDo(() -> {
                elevator.stopElev();
            })
        );
        
        // Right Bumper: Shifted controls (if you implement it)
        // This could be a toggle or modifier for other controls
        operatorController.rightBumper().onTrue(
            Commands.runOnce(() -> {
                // Implement shifted controls logic here
            })
        );
    }

    public Command getAutonomousCommand() {
        return null; // Replace with your autonomous command
    }
}