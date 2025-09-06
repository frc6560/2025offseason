package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

    private final TalonFX ElevLeft = new TalonFX(ElevatorConstants.ElevLeftCanID, "Canivore");
    private final TalonFX ElevRight = new TalonFX(ElevatorConstants.ElevRightCanID, "Canivore");

    private final DigitalInput topLimitSwitch = new DigitalInput(ElevatorConstants.TopLimitSwitchID);
    private final DigitalInput botLimitSwitch = new DigitalInput(ElevatorConstants.BotLimitSwitchID);

    private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(
            ElevatorConstants.kS, ElevatorConstants.kV, ElevatorConstants.kA);

    public final TrapezoidProfile.Constraints elevConstraints =
            new TrapezoidProfile.Constraints(ElevatorConstants.kMaxV, ElevatorConstants.kMaxA);

    public TrapezoidProfile.State elevGoalState = new TrapezoidProfile.State();
    public TrapezoidProfile.State elevSetpointState = new TrapezoidProfile.State();

    private final PIDController pidController = new PIDController(
            ElevatorConstants.kP,
            ElevatorConstants.kI,
            ElevatorConstants.kD
    );

    public enum WantedState {
        Stow,
        L2Ball,
        L3Ball,
        ShootBall,
        Idle,
        GroundBall,
    }

    private WantedState wantedState = WantedState.Idle;

    // Mechanism2d for SmartDashboard visualization
    private final Mechanism2d mechanism = new Mechanism2d(2, 5);
    private final MechanismRoot2d root = mechanism.getRoot("ElevatorBase", 1.0, 0);
    private final MechanismLigament2d elevatorLift =
            root.append(new MechanismLigament2d("ElevatorLift", 0.0, 90));

    // Shuffleboard tab for numeric + boolean properties
    private final ShuffleboardTab tab = Shuffleboard.getTab("Elevator");

    public Elevator() {
        // Keep mechanism in SmartDashboard

        // TalonFX motor configuration
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        Slot0Configs elevatorPID = new Slot0Configs();
        elevatorPID.kP = ElevatorConstants.kP;
        elevatorPID.kI = ElevatorConstants.kI;
        elevatorPID.kD = ElevatorConstants.kD;
        elevatorPID.kG = ElevatorConstants.kG;

        ElevLeft.getConfigurator().apply(config.withSlot0(elevatorPID));
        ElevRight.getConfigurator().apply(config.withSlot0(elevatorPID));

        elevSetpointState = new TrapezoidProfile.State(getElevatorHeight(), 0);

        // Register this subsystem in Shuffleboard for AdvantageScope to see Sendable properties
        tab.add(this);
    }

    @Override
    public void periodic() {
        double elevatorHeight = getElevatorHeight();

        // Step trapezoid profile
        elevSetpointState = new TrapezoidProfile(elevConstraints)
                .calculate(0.02, elevSetpointState, elevGoalState);

        // Visualization
        elevatorLift.setLength(elevatorHeight);
        elevatorLift.setAngle(90);

        SmartDashboard.putData("ElevatorMechanism", mechanism);

        System.out.println(elevatorHeight);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Elevator");

        // Numeric properties
        builder.addDoubleProperty("Height", this::getElevatorHeight, null);
        builder.addDoubleProperty("Setpoint", () -> elevSetpointState.position, null);
        builder.addDoubleProperty("Goal", () -> elevGoalState.position, null);

        // Boolean buttons to change WantedState
        builder.addBooleanProperty("GoToStow", () -> false, val -> setWantedState(WantedState.Stow));
        builder.addBooleanProperty("GoToL2Ball", () -> false, val -> setWantedState(WantedState.L2Ball));
        builder.addBooleanProperty("GoToL3Ball", () -> false, val -> setWantedState(WantedState.L3Ball));
        builder.addBooleanProperty("GoToShootBall", () -> false, val -> setWantedState(WantedState.ShootBall));
        builder.addBooleanProperty("GoToGroundBall", () -> false, val -> setWantedState(WantedState.GroundBall));
    }

    // Motor getters
    public TalonFX getElevLeft() { return ElevLeft; }
    public TalonFX getElevRight() { return ElevRight; }

    // Goal/setpoint setters
    public void setSetpoint(TrapezoidProfile.State nextSetpoint) { elevSetpointState = nextSetpoint; }
    public TrapezoidProfile.State getSetpoint() { return elevSetpointState; }
    public void setGoal(double goalState) { elevGoalState = new TrapezoidProfile.State(goalState, 0); }
    public TrapezoidProfile.State getGoal() { return elevGoalState; }
    public double getGoalValue() { return elevGoalState.position; }

    // Stop motors
    public void stopElev() {
        ElevLeft.stopMotor();
        ElevRight.stopMotor();
    }

    // Limit switches
    public boolean getLimitSwitchTop() { return topLimitSwitch.get(); }
    public boolean getLimitSwitchBot() { return botLimitSwitch.get(); }

    // Elevator height in rotations
    public double getElevatorHeight() { return ElevLeft.getPosition().getValueAsDouble(); }

    // WantedState setter
    public void setWantedState(WantedState state) { wantedState = state; }
}
