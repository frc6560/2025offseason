package frc.robot.commands;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevState;
import frc.robot.subsystems.Elevator;
import frc.robot.ManualControls;
import com.ctre.phoenix6.controls.PositionVoltage;

public class ElevatorCommand extends Command{

    public Elevator elevator;
    public double elevSetpointState;
    public TrapezoidProfile currentTrapezoidProfile;
    public ElevatorFeedforward feedforward;
    public PIDController elevatorPIDController;
    public ManualControls controls;
    public ElevState elevatorState;
            

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

        this.elevatorPIDController = new PIDController(
            ElevatorConstants.kP,
            ElevatorConstants.kI,
            ElevatorConstants.kD);

        addRequirements(elevator);

    }   

    public void initialize() {

        elevator.stopElev();
        elevator.setGoal(ElevState.STOW.getValue());
        elevator.setSetpoint(new TrapezoidProfile.State(ElevState.STOW.getValue(),0));
        System.out.println("Initialize elevator");
    }

    public void execute() {

        currentTrapezoidProfile = new TrapezoidProfile(
            elevator.getConstraints());

        double currentPosition = elevator.getElevatorHeight();

        TrapezoidProfile.State nextSetpoint = currentTrapezoidProfile.calculate(
            0.02, 
            elevator.getSetpoint(), 
            elevator.getGoal());
        
        elevator.setSetpoint(nextSetpoint);

        elevatorPIDController.setSetpoint(nextSetpoint.position);

        SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
            ElevatorConstants.kS,
            ElevatorConstants.kV,
            ElevatorConstants.kA);

        double feedForwardPower = feedforward.calculate(nextSetpoint.velocity)/12;

        double elevatorPower = elevatorPIDController.calculate(currentPosition);

        double TotalPower = elevatorPower + feedForwardPower;
        PositionVoltage m_request = new PositionVoltage(TotalPower);

        elevator.getElevLeft().setControl(m_request);
        elevator.getElevRight().setControl(m_request);

    }

    public boolean isFinished() {
        return false;
    }

    public void periodic(){
        if (controls.goToL2Ball()) {
            elevator.setGoal(ElevState.L2BALL.getValue());
        }
        else if (controls.goToL3Ball()) {
            elevator.setGoal(ElevState.L3BALL.getValue());
        }
        else if (controls.goToShootBall()) {
            elevator.setGoal(ElevState.SHOOTBALL.getValue());
        }
        else if (controls.goToStow()) {
            elevator.setGoal(ElevState.STOW.getValue());
        }
    }
    
    public void end() {
        elevator.stopElev();
    }

}