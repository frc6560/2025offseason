package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.WantedState;
import frc.robot.ManualControls;

public class ElevatorCommand extends Command {

    private final Elevator elevator;
    private final ManualControls controls;
    private WantedState targetState;

    public ElevatorCommand(Elevator elevator, ManualControls controls) {
        this.elevator = elevator;
        this.controls = controls;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.stopElev();
    }

    @Override
    public void execute() {
        // Check controller inputs to determine target state
        // You'll need to add these methods to XboxControls:
        // goToStow(), goToL2Ball(), goToL3Ball(), goToShootBall()
        
        if (controls.goToStow()) {
            targetState = WantedState.Stow;
        } else if (controls.goToL2Ball()) {
            targetState = WantedState.L2Ball;
        } else if (controls.goToL3Ball()) {
            targetState = WantedState.L3Ball;
        } else if (controls.goToShootBall()) {
            targetState = WantedState.ShootBall;
        }

        // Set goal based on target state
        if (targetState == WantedState.Stow) {
            elevator.setGoal(ElevatorConstants.ElevState.STOW.getValue());
        } else if (targetState == WantedState.L2Ball) {
            elevator.setGoal(ElevatorConstants.ElevState.L2BALL.getValue());
        } else if (targetState == WantedState.L3Ball) {
            elevator.setGoal(ElevatorConstants.ElevState.L3BALL.getValue());
        } else if (targetState == WantedState.ShootBall) {
            elevator.setGoal(ElevatorConstants.ElevState.SHOOTBALL.getValue());
        }

        // Execute the motion profile control
        
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