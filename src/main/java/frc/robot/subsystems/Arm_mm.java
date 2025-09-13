

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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;



public class Arm_mm extends SubsystemBase{
    
    private double simAngleDeg = 90; // start straight up
    private double simTime = 0.0;
    private int currentTargetIndex = 0;
    private double stateStartTime = 0.0;
    private boolean isMoving = true;
    private final double[] targetPositions = {
      ArmConstants.STOW_POSITION_DEG,      // 0.0°
      ArmConstants.PICKUP_POSITION_DEG,    // 121.0°
      ArmConstants.REEF_POSITION_DEG,      // 11.0°
      ArmConstants.GROUND_PICKUP_DEG,      // -31.0°
      ArmConstants.PROCESSOR_DEG           // 0.0°
  };


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

    // Mechanism2d visualization
    private final Mechanism2d mech = new Mechanism2d(60, 60);
    private final MechanismRoot2d root = mech.getRoot("ArmRoot", 30, 5); // pivot at bottom
    private final MechanismLigament2d armLigament = 
        root.append(new MechanismLigament2d("Arm", 20, 90)); // length=20, angle=90°
    


    public enum State {
        STOW,
        PICKUP,
        REEF,
        GROUND,
        PROCESSOR,
        IN_MOTION
    }
    public Arm_mm() {
      armMotor = new TalonFX(ArmConstants.MOTOR_ID);
      TalonFXConfiguration config = new TalonFXConfiguration();
      config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      armMotor.getConfigurator().apply(config);
  
      absoluteEncoder = new DutyCycleEncoder(ArmConstants.ABS_ENCODER_DIO_PORT);
  
      pid.setTolerance(1.0);
  
      // Set arm ligament length in pixels (scaled from meters)
      armLigament.setLength(ArmConstants.ARM_LENGTH_METERS * 50);
  
      // Initialize at STOW
      armLigament.setAngle(ArmConstants.STOW_POSITION_DEG);
  
      SmartDashboard.putData("ArmMech2d", mech);
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
        if (RobotBase.isSimulation()) {
            return simAngleDeg;
        } else {
            // Use .get() which returns 0–1 of a rotation
            double angle = absoluteEncoder.get() * 360.0;
    
            // Apply offset so stow = 0°
            angle -= ArmConstants.ABS_ENCODER_OFFSET_DEG;
    
            // Flip if reversed
            if (ArmConstants.ABS_ENCODER_REVERSED) {
                angle = -angle;
            }
    
            // Normalize to [-180, 180]
            angle = Math.IEEEremainder(angle, 360.0);
    
            return angle;
        }
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
  armLigament.setAngle(getArmAngleDeg());

  

  if (RobotBase.isSimulation()) {
    // Increment time (assuming 20ms periodic calls)
    simTime += 0.02;
    
    double currentTarget = targetPositions[currentTargetIndex];
    double timeSinceStateStart = simTime - stateStartTime;
    
    if (isMoving) {
        // Move towards the current target
        double error = currentTarget - simAngleDeg;
        
        // Check if we're close enough to the target (within 2 degrees)
        if (Math.abs(error) < 2.0) {
            // We've reached the target, start the 3-second wait
            isMoving = false;
            stateStartTime = simTime;
            simAngleDeg = currentTarget; // Snap to exact position
            System.out.println("Reached target: " + currentTarget + "°. Waiting for 3 seconds.");
            
        } else {
            // Move 15% of the way to target each cycle (faster movement)
            simAngleDeg += error * 0.15;
        }
    } else {
        // We're waiting at the current position
        if (timeSinceStateStart >= 3.0) {
            // 3 seconds have passed, move to next target
            currentTargetIndex = (currentTargetIndex + 1) % targetPositions.length;
            isMoving = true;
            stateStartTime = simTime;
            System.out.println("Moving to next target: " + targetPositions[currentTargetIndex] + "°");
            
        }
        // Keep the arm at the current target position while waiting
        simAngleDeg = currentTarget;
    }
    
    // Debug output every 50 cycles (~1 second)
    if ((int)(simTime * 50) % 50 == 0) {
        System.out.println("Sim angle: " + String.format("%.1f", simAngleDeg) + 
                            " | Target: " + String.format("%.1f", currentTarget) + 
                            " | Moving: " + isMoving + 
                            " | Time at state: " + String.format("%.1f", timeSinceStateStart) + "s");
    }
}

// Add debug output
SmartDashboard.putString("Arm State", getState().toString());
SmartDashboard.putNumber("Current Angle", getArmAngleDeg());
SmartDashboard.putNumber("Target Angle", Math.toDegrees(setpoint.position));
}







}