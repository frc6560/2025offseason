package frc.robot.commands;

import frc.robot.subsystems.Arm;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm.State;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.keyboardControls;

public class ArmCommand extends Command {
    private final Arm arm;
    private final keyboardControls keyboardControls;
    public State target_state;
    
    public ArmCommand(Arm arm, keyboardControls keyboardControls) {
        this.arm = arm;
        this.keyboardControls = keyboardControls;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setGoal(ArmConstants.STOW_POSITION_DEG); // Use the correct constant name
        target_state = Arm.State.STOW; // Set initial target state to STOW
    }

    @Override
    public void execute() {
        // Debug output to see what's happening
        if (keyboardControls.goToStow()) {
            System.out.println("STOW button pressed!");
            target_state = Arm.State.STOW;
        } else if (keyboardControls.goToPickup()) { 
            System.out.println("PICKUP button pressed!");
            target_state = Arm.State.PICKUP;
        } else if (keyboardControls.goToReef()) { 
            System.out.println("REEF button pressed!");
            target_state = Arm.State.REEF;
        } else if (keyboardControls.goToGround()) { 
            System.out.println("GROUND button pressed!");
            target_state = Arm.State.GROUND;
        } else if (keyboardControls.goToProcessor()) { 
            System.out.println("PROCESSOR button pressed!");
            target_state = Arm.State.PROCESSOR;
        }
        
        double targetAngle;
        switch (target_state) {
            case STOW:
                targetAngle = ArmConstants.STOW_POSITION_DEG;
                break;
            case PICKUP:
                targetAngle = ArmConstants.PICKUP_POSITION_DEG;
                break;
            case REEF:
                targetAngle = ArmConstants.REEF_POSITION_DEG; 
                break;
            case GROUND:
                targetAngle = ArmConstants.GROUND_PICKUP_DEG; 
                break;
            case PROCESSOR:
                targetAngle = ArmConstants.PROCESSOR_DEG; 
                break;
            case IN_MOTION:
                return; 
            default:
                return; 
        }

        // Command arm to move to target position
        arm.setGoal(targetAngle);
    }

    @Override
    public boolean isFinished() {
        return false; 
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}