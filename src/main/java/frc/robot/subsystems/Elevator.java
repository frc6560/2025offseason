// Anish

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;

import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase{

    private final TalonFX ElevLeft = new TalonFX(ElevatorConstants.ElevLeftCanID, "Canivore");
    private final TalonFX ElevRight = new TalonFX(ElevatorConstants.ElevRightCanID, "Canivore");

    private final DigitalInput topLimitSwitch = new DigitalInput(ElevatorConstants.TopLimitSwitchID);
    private final DigitalInput botLimitSwitch = new DigitalInput(ElevatorConstants.BotLimitSwitchID);

    private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(
        ElevatorConstants.kS, ElevatorConstants.kV,ElevatorConstants.kA);
    private double targetPos = 0;

    public final TrapezoidProfile elevTrapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
        ElevatorConstants.kMaxV, ElevatorConstants.kMaxA));

    public TrapezoidProfile.State elevGoalState = new TrapezoidProfile.State();
    public TrapezoidProfile.State elevSetpointState = new TrapezoidProfile.State(); 


    public Elevator() {

        TalonFXConfiguration config = new TalonFXConfiguration();
        
        final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
        setSetpoint(elevTrapezoidProfile.calculate(0.02, elevSetpointState, elevGoalState));
        m_request.Position = elevSetpointState.position;
        m_request.Velocity = elevSetpointState.velocity;
        ElevLeft.setControl(m_request);
        ElevRight.setControl(m_request);


        Slot0Configs elevatorPID = new Slot0Configs();
        
        elevatorPID.kP = 0.8;
        elevatorPID.kI = 0.05;
        elevatorPID.kD = 0.1;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        ElevLeft.getConfigurator().apply(config.withSlot0(elevatorPID));
        ElevRight.getConfigurator().apply(config.withSlot0(elevatorPID));
    }

    public void periodic() {

    }

    public void setSetpoint(TrapezoidProfile.State nextSetpoint) {
        elevSetpointState = nextSetpoint;
    }

    public TrapezoidProfile.State getSetpoint() {
        return elevSetpointState;
    }

    public void setGoal(double goalState) {
        elevGoalState = new TrapezoidProfile.State(goalState, 0);
    }
    
    public TrapezoidProfile.State getGoal() {
        return elevGoalState;
    }

    public void stopElev() {
        ElevLeft.stopMotor();
        ElevRight.stopMotor();
    }

    public boolean getLimitSwitchTop() {
        return topLimitSwitch.get();
    }

    public boolean getLimitSwitchBot() {
        return botLimitSwitch.get();
    }

    public double getElevatorHeight() {
        return ElevLeft.getPosition().getValueAsDouble();
    }
}