package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntake extends SubsystemBase {
    private CANSparkMax leftIntakeMotor;
    private CANSparkMax rightIntakeMotor;
    
    /** Creates a new CoralIntake. */
    public CoralIntake() {
        // Initialize both motors for the roller intake
        leftIntakeMotor = new CANSparkMax(Constants.LEFT_INTAKE_MOTOR, MotorType.kBrushless);
        rightIntakeMotor = new CANSparkMax(Constants.RIGHT_INTAKE_MOTOR, MotorType.kBrushless);
        
        // Factory reset both motors
        leftIntakeMotor.restoreFactoryDefaults();
        rightIntakeMotor.restoreFactoryDefaults();
        
        // Set brake mode to hold position when stopped
        leftIntakeMotor.setIdleMode(IdleMode.kBrake);
        rightIntakeMotor.setIdleMode(IdleMode.kBrake);
        
        // Set current limits to protect motors
        leftIntakeMotor.setSmartCurrentLimit(25);
        rightIntakeMotor.setSmartCurrentLimit(25);
        
        // Make right motor follow left motor but inverted 
        // (since they face each other in the roller setup)
        rightIntakeMotor.follow(leftIntakeMotor, true);
    }
    
    /**
     * Set the speed of both intake motors
     * @param speed Speed from -1.0 to 1.0 (positive = intake, negative = outtake)
     */
    public void setSpeed(double speed) {
        leftIntakeMotor.set(speed);
        // Right motor automatically follows due to the follow() setup
    }
    
    /**
     * Stop the intake motors
     */
    public void stop() {
        setSpeed(0.0);
    }
    
    /**
     * Get the current speed of the intake
     */
    public double getSpeed() {
        return leftIntakeMotor.get();
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}