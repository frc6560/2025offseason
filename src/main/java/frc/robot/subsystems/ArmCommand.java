package frc.robot.subsystems;

import frc.robot.subsystems.Arm;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm.State;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.XboxControls;
import frc.robot.commands.keyboardControls;

public class ArmCommand extends Command {
    private final Arm arm;
    private final XboxControls controls;
    public State targetState;
    
    public ArmCommand(Arm arm, XboxControls controls) {
        this.arm = arm;
        this.controls = controls;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setGoal(ArmConstants.STOW_POSITION_DEG); // Use the correct constant name
        targetState = Arm.State.STOW; // Set initial target state to STOW
    }

    @Override
    public void execute(){

    if (controls.goToStow()){
        targetState = State.STOW;
      } else if (controls.goToPickup()){
        targetState = State.PICKUP;
      } else if (controls.goToGround()){
        targetState = State.GROUND;
      } else if (controls.goToReef()){
        targetState = State.REEF;
      } else if (controls.goToProcessor()){
        targetState = State.PROCESSOR;
      } 
       
    
      if(targetState == State.STOW){

        arm.setGoal(ArmConstants.STOW_POSITION_DEG);

      } else if (targetState == State.PICKUP){

        arm.setGoal(ArmConstants.PICKUP_POSITION_DEG);

      } else if (targetState == State.GROUND){

        arm.setGoal(ArmConstants.GROUND_PICKUP_DEG);

      } else if (targetState == State.REEF) {

        arm.setGoal(ArmConstants.REEF_POSITION_DEG);

      } else if (targetState == State.PROCESSOR) {

        arm.setGoal(ArmConstants.PROCESSOR_DEG);

      } else {}

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