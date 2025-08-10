// Anish

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.ExponentialProfile.Constraints;
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

    public final TrapezoidProfile.Constraints elevConstraints = new TrapezoidProfile.Constraints(
        ElevatorConstants.kMaxV, ElevatorConstants.kMaxA);

    public final TrapezoidProfile elevTrapezoidProfile = new TrapezoidProfile(elevConstraints);

    public TrapezoidProfile.State elevGoalState = new TrapezoidProfile.State();
    public TrapezoidProfile.State elevSetpointState = new TrapezoidProfile.State(); 

    private final PIDController pidContrller = new PIDController(
        ElevatorConstants.kP,
        ElevatorConstants.kI,
        ElevatorConstants.kD);
    
    public Elevator() {

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        Slot0Configs elevatorPID = new Slot0Configs();
        
        elevatorPID.kP = ElevatorConstants.kP;
        elevatorPID.kI = ElevatorConstants.kI;
        elevatorPID.kD = ElevatorConstants.kD;
        elevatorPID.kG = ElevatorConstants.kG;

        ElevLeft.getConfigurator().apply(config.withSlot0(elevatorPID));
        ElevRight.getConfigurator().apply(config.withSlot0(elevatorPID));

        final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
        setSetpoint(elevTrapezoidProfile.calculate(0.02, elevSetpointState, elevGoalState));
        m_request.Position = elevSetpointState.position;
        m_request.Velocity = elevSetpointState.velocity;
        ElevLeft.setControl(m_request);
        ElevRight.setControl(m_request);

    }

    public void periodic() {

    }

    public TalonFX getElevLeft() {
        return ElevLeft;
    }

    public TalonFX getElevRight() {
        return ElevRight;
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

    public TrapezoidProfile.Constraints getConstraints() {
        return elevConstraints;
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