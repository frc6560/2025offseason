package frc.robot.commands;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.State;
import frc.robot.Constants.ElevatorConstants.State.ElevState;
import frc.robot.subsystems.Elevator;
import frc.robot.ManualControls;

public class ElevatorCommand extends Command{

    public Elevator elevator;
    public double elevSetpointState;
    public TrapezoidProfile currentTrapezoidProfile;
    public ElevatorFeedforward feedforward;
    public PIDController elevatorPIDController;
    public ManualControls controls;

    private State targetState;
            

    public ElevatorCommand(Elevator elevator, ManualControls controls, ElevState elevatorState){
        this.elevator = elevator;
        this.controls = controls;
        this.elevSetpointState = elevatorState.elevatorSetPoint;
            this.feedforward = 
                new ElevatorFeedforward(
                    ElevatorConstants.kS, 
                    ElevatorConstants.kG, 
                    ElevatorConstants.kV, 
                    ElevatorConstants.kA);

        this.elevatorPIDController = new PIDController(0.3,0,0);
        addRequirements(elevator);

    }   
//STOP HEREERERERERERER START HERERERERERER
    public void initialize() {

        elevator.stopElev();
        elevator.setElevatorPosition(ElevatorConstants.State.L1ORSTOW);
        System.out.println("Initialize elevator");
    }

    public void execute() {

        if (elevator.getLimitSwitchTop()) {
            elevator.stopElev();
        }

        else if (elevator.getLimitSwitchBot()) {
            elevator.stopElev();
            System.out.println("Bottom Limit Switch");
        }

        if (controls.shiftedControls()) {
            System.out.println("ShiftedControls");

    
            if (controls.goToL2()) {
                targetState = State.REMOVEBALLL2;
                System.out.println("L1/2 Ball Removal");
            }
            
            else if (controls.goToL3()) {
                targetState = State.REMOVEBALLL3;
                System.out.println("L2/3 Ball Removal");
            }

            else if (controls.goToL4()) {
                targetState = State.SHOOTBALL;
                System.out.println("Shoot Ball");
            }
        }

        else {

            if (controls.goToL1()) {
                targetState = State.L1ORSTOW;
                System.out.println("L1");
            }
    
            else if (controls.goToL2()) {
                targetState = State.L2;
                System.out.println("L2");
            }
            
            else if (controls.goToL3()) {
                targetState = State.L3;
                System.out.println("L3");
            }
    
            else if (controls.goToL4()) {
                targetState = State.L4;
                System.out.println("L4");
            }

        }

        

        
        double targetRotations = 0;

        if (targetState == State.L1ORSTOW) {
            targetRotations = (ElevatorConstants.State.L1ORSTOW);
        }
        else if (targetState == State.L2) {
            targetRotations = (ElevatorConstants.State.L2);
        }
        else if (targetState == State.L3) {
            targetRotations = (ElevatorConstants.State.L3);
        }
        else if (targetState == State.L4) {
            targetRotations = (ElevatorConstants.State.L4);
        }
        else if (targetState == State.REMOVEBALLL2) {
            targetRotations = (ElevatorConstants.State.REMOVEBALLL2);
        }
        else if (targetState == State.REMOVEBALLL3) {
            targetRotations = (ElevatorConstants.State.REMOVEBALLL3);
        }
        else if (targetState == State.SHOOTBALL) {
            targetRotations = (ElevatorConstants.State.SHOOTBALL);
        }
        
        elevator.setElevatorPosition(targetRotations);

    }

    public boolean isFinished() {
        return false;
    }

    public void periodic(){
        if (controls.goToL1()) {
        Elevator.State.elevGoalState = new TrapezoidProfile.State(5, 0);
        }
        else if (controls.goToL2()) {
        m_goal = new TrapezoidProfile.State();
    }
    }
    public void end() {
        elevator.stopElev();
    }

}