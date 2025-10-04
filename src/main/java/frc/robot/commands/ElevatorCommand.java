package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import com.ctre.phoenix6.controls.PositionVoltage;
import frc.robot.ManualControls;
import frc.robot.subsystems.SubsystemManager;

public class ElevatorCommand extends Command {

    private final Elevator elevator;
    private final PIDController elevatorPIDController;
    private final SimpleMotorFeedforward feedforward;
    private final ManualControls controls;
    

    public ElevatorCommand(Elevator elevator, ManualControls controls) {
        this.elevator = elevator;
        this.controls = controls;
        

        this.elevatorPIDController = new PIDController(
                ElevatorConstants.kP,
                ElevatorConstants.kI,
                ElevatorConstants.kD);

        this.feedforward = new SimpleMotorFeedforward(
                ElevatorConstants.kS,
                ElevatorConstants.kV,
                ElevatorConstants.kA);

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double currentPosition = elevator.getElevatorHeight();
        double targetPosition = elevator.getGoalValue();
        double targetVelocity = elevator.getSetpoint().velocity;

        double pidOutput = elevatorPIDController.calculate(currentPosition, targetPosition);
        double ffOutput = feedforward.calculate(targetVelocity) / 12.0;

        PositionVoltage output = new PositionVoltage(pidOutput + ffOutput);
        elevator.getElevLeft().setControl(output);
        elevator.getElevRight().setControl(output);
    }

    public void periodic() {
        
    }
    
    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stopElev();
    }
}
