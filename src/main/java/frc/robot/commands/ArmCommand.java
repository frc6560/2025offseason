package frc.robot.commands;

import frc.robot.subsystems.Arm;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm.State;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ManualControls;
import frc.robot.commands.keyboardControls;

public class ArmCommand extends Command {
    private final Arm arm;
    private final ManualControls controls;
    public State targetState;
    
    public ArmCommand(Arm arm, ManualControls controls) {
        this.arm = arm;
        this.controls = controls;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setArmGoal(ArmConstants.STOW_POSITION_DEG); // Use the correct constant name
        targetState = Arm.State.STOW; // Set initial target state to STOW
    }

    @Override
    public void execute(){

    }

    @Override
    public boolean isFinished() {
        return false; 
    }

    @Override
    public void end(boolean interrupted) {
        arm.stopMotor();
    }
}