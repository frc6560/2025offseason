package frc.robot.subsystems;
// CTRE imports
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.configs.CANcoderConfiguration;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
// WPILib (Subsystem & Utilities) imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableInstance.NetworkMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.ArmFeedforward;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;


public class Arm extends SubsystemBase{
    
    private TalonFX armMotor;
    private DutyCycleEncoder absoluteEncoder;

    private final PIDController pid = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
    private final ArmFeedforward feedforward = new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV, ArmConstants.kA);

  
    private final TrapezoidProfile.Constraints constraints =
        new TrapezoidProfile.Constraints(ArmConstants.kMaxV, ArmConstants.kMaxA);
    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    private final NetworkTable ntTable = NetworkTableInstance.getDefault().getTable("Arm");
    private final NetworkTableEntry ntAngle = ntTable.getEntry("Angle");
    private final NetworkTableEntry ntPosition = ntTable.getEntry("Arm position");
    private final NetworkTableEntry ntTargetPos = ntTable.getEntry("Target angle");

    public enum State {
        STOW,
        PICKUP,
        REEF,
        GROUND,
        PROCESSOR,
        IN_MOTION
    }

    public Arm() {
      armMotor = new TalonFX(ArmConstants.MOTOR_ID);
      TalonFXConfiguration config = new TalonFXConfiguration();
      config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      armMotor.getConfigurator().apply(config);

    // PID settings
    pid.setTolerance(1.0); // 1 degree tolerance
    }

    public void setGoal(double goalStateDeg) {
      goal = new TrapezoidProfile.State(Math.toRadians(goalStateDeg), 0);
  }
  

    public TrapezoidProfile.State getGoal() {
        return goal;
      }

    public void setSetpoint(TrapezoidProfile.State nextSetpoint) {
        setpoint = nextSetpoint;
      }

    public TrapezoidProfile.State getSetpoint() {
        return setpoint;
      }

    public TrapezoidProfile.Constraints getConstraints() {
        return constraints;
      }

    public double getArmAngleDeg() {
        // Rotor position comes in *rotations of the motor shaft*
        double motorRotations = armMotor.getRotorPosition().getValueAsDouble();
        ;
      
        // Convert motor rotations → arm rotations using gear ratio
        double armRotations = motorRotations / ArmConstants.MOTOR_GEAR_RATIO;
      
        // Convert to degrees
        return armRotations * 360.0;
      }

    public State getState(){
        // Determine the current state of the arm based on its position
        double padding = 1.5;
        double angle = getArmAngleDeg();
        
        if (Math.abs(angle - ArmConstants.STOW_POSITION_DEG) < padding) {
            return State.STOW;
        } else if (Math.abs(angle - ArmConstants.PICKUP_POSITION_DEG) < padding) {
            return State.PICKUP;
        } else if (Math.abs(angle - ArmConstants.REEF_POSITION_DEG) < padding) {
            return State.REEF;
        } else if (Math.abs(angle - ArmConstants.GROUND_PICKUP_DEG) < padding) {
            return State.GROUND;
        } else if (Math.abs(angle - ArmConstants.PROCESSOR_DEG) < padding) {
            return State.PROCESSOR;
        } else {
            return State.IN_MOTION; // If no specific state is matched, return IN_MOTION
           
        }
    }

      @Override
public void periodic() {
  // Generate new profile step (constraints already stored in class)
  TrapezoidProfile profile = new TrapezoidProfile(constraints);

  // Advance profile by 20ms
  setpoint = profile.calculate(0.02, setpoint, goal);

  double currentAngleRad = Math.toRadians(getArmAngleDeg());

  // PID + Feedforward
  double pidOutput = pid.calculate(currentAngleRad, setpoint.position);
  double ffOutput = feedforward.calculate(setpoint.position, setpoint.velocity);

  double volts = pidOutput + ffOutput;

  // Apply to motor
  armMotor.setControl(new VoltageOut(volts));

  ntAngle.setDouble(getArmAngleDeg());
  ntPosition.setDouble(getArmAngleDeg());
  ntTargetPos.setDouble(Math.toDegrees(setpoint.position));
}

      


}
