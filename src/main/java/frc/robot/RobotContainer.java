package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.commands.ElevatorCommand;

public class RobotContainer {

    // Controllers
    private final CommandXboxController driverXbox = new CommandXboxController(0);
    private final XboxController firstXbox = new XboxController(0);
    private final XboxController secondXbox = new XboxController(1);
    private final ManualControls controls = new ManualControls(firstXbox, secondXbox);
    

    // Subsystems
    private final Elevator elevator = new Elevator();

    public RobotContainer() {
        configureBindings();
        // Default elevator command
        elevator.setDefaultCommand(new ElevatorCommand(elevator,controls));
    }

    private void configureBindings() {
        // Simulation: Use SmartDashboard buttons for elevator
        // No actual keyboard bindings needed
    }

    public Command getAutonomousCommand() {
        return null; // Replace with your auto
    }
}
