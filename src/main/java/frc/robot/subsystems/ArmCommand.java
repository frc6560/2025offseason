package frc.robot.subsystems;

import frc.robot.subsystems.Arm;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm.State;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ManualControls;

public class ArmCommand extends Command {
    private final Arm arm;
    private final ManualControls manualControls;
    private State targetState;

    public ArmCommand(Arm arm, ManualControls manualControls) {
        this.arm = arm;
        this.manualControls = manualControls;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setGoal(ArmConstants.STOW_POSITION_DEG); // Initialize arm to stowed position
        targetState = Arm.State.STOW;
    }

    @Override
    public void execute() {
        // Check manual controls for desired state
        if (manualControls.goToStow()) {
            targetState = Arm.State.STOW;
        } else if (manualControls.goToPickup()) {
            targetState = Arm.State.PICKUP;
        } else if (manualControls.goToReef()) {
            targetState = Arm.State.REEF;
        } else if (manualControls.goToGround()) {
            targetState = Arm.State.GROUND;
        } else if (manualControls.goToProcessor()) {
            targetState = Arm.State.PROCESSOR;
        } else {
            targetState = Arm.State.IN_MOTION;
        }

        // Pick target angle based on state
        double targetAngleDeg;
        switch (targetState) {
            case STOW:
                targetAngleDeg = ArmConstants.STOW_POSITION_DEG;
                break;
            case PICKUP:
                targetAngleDeg = ArmConstants.PICKUP_POSITION_DEG;
                break;
            case REEF:
                targetAngleDeg = ArmConstants.REEF_POSITION_DEG;
                break;
            case GROUND:
                targetAngleDeg = ArmConstants.GROUND_PICKUP_DEG;
                break;
            case PROCESSOR:
                targetAngleDeg = ArmConstants.PROCESSOR_DEG;
                break;
            case IN_MOTION:
            default:
                // If moving or unknown, hold current position
                targetAngleDeg = arm.getArmAngleDeg();
                break;
        }

        // Send goal to arm subsystem
        arm.setGoal(targetAngleDeg);
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted
    }

    @Override
    public void end(boolean interrupted) {
        arm.setGoal(arm.getArmAngleDeg()); // Hold current position when command ends
    }
}
