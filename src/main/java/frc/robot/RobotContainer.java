package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.subsystems.BallGrabber;
import frc.robot.commands.BallGrabberCommand;
import java.io.File;
import swervelib.SwerveInputStream;
import frc.robot.subsystems.swervedrive.*;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.commands.SubsystemManagerCommand;

public class RobotContainer {

    // Controllers
    private final CommandXboxController driverXbox = new CommandXboxController(0);
    private final XboxController firstXbox = new XboxController(0);
    private final XboxController secondXbox = new XboxController(1);
    private final ManualControls controls = new ManualControls(firstXbox, secondXbox);
    

    // Subsystems
    private final Elevator elevator = new Elevator();
    private final Arm arm = new Arm();
    private final BallGrabber ballGrabber = new BallGrabber();
    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/falcon"));
    private final SubsystemManager subsystemManager = new SubsystemManager(drivebase, elevator, arm, ballGrabber, controls);

    

    public RobotContainer() {
      arm.setDefaultCommand(new ArmCommand(arm, controls));

      
      configureBindings();
      // Default elevator command
      elevator.setDefaultCommand(new ElevatorCommand(elevator,controls));
        
      ballGrabber.setDefaultCommand(new BallGrabberCommand(ballGrabber, controls));

      subsystemManager.setDefaultCommand(new SubsystemManagerCommand(drivebase, elevator, arm, ballGrabber, controls, subsystemManager));
      

    }

    private void configureBindings() {
        // Simulation: Use SmartDashboard buttons for elevator
        // No actual keyboard bindings needed
    }

    public Command getAutonomousCommand() {
        return null; // Replace with your auto
    }
}
