
package frc.robot.commands;

import frc.robot.subsystems.CoralIntake;
import edu.wpi.first.wpilibj2.command.Command;

public class CoralIntakeCommand extends Command {
    private final CoralIntake coralIntake;
    private final Controls controls;
    
    public static interface Controls {
        boolean getIntakeIn();
        boolean getIntakeInReleased();
        boolean getIntakeOut();
        boolean getIntakeOutReleased();
    }
    
    /** Creates a new CoralIntakeCommand. */
    public CoralIntakeCommand(CoralIntake coralIntake, Controls controls) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(coralIntake);
        this.coralIntake = coralIntake;
        this.controls = controls;
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (controls.getIntakeIn()) {
            coralIntake.setSpeed(0.8);  // Intake coral at 80% speed
        } else if (controls.getIntakeOut()) {
            coralIntake.setSpeed(-0.6); // Outtake coral at 60% speed
        } else {
            coralIntake.setSpeed(0.0);  // Stop intake
        }
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        coralIntake.stop();
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }