
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


import com.ctre.phoenix6.hardware.CANrange; 



import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
// import com.robot.Constants; 

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableInstance.NetworkMode;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import static com.team6560.frc2025.utility.NetworkTable.NtValueDisplay.ntDispTab;
import static edu.wpi.first.units.Units.Degree;

public class BallGrabber extends SubsystemBase {
    private final TalonFX grabberMotor; 

    private final CANrange grabberMotorRange; // Range for TalonFX IDs

    // encoder stuff
  private CANcoder m_relativeEncoder;
  private double initialEncoderPos;
  private TalonFXConfiguration fxConfig;
    


    private static final double INTAKE_SPEED = -0.3;
    private static final double OUTTAKE_SPEED = 0.7; 

    private static final double MAX_CURRENT_RUNNING = 30; 
    private static final double BALL_DETECTION_DISTANCE = 20; //  threshold to detect if a ball is present
    



//   private final NetworkTable ntTable = NetworkTableInstance.getDefault().getTable("BallGrabber");
//   private final NetworkTableEntry ntCurrent = ntTable.getEntry("Ball Grabber Current");
//   private final NetworkTableEntry ntVoltage = ntTable.getEntry("Ball Grabber Voltage");
//   private final NetworkTableEntry ntSpeed = ntTable.getEntry("Ball Grabber Speed");
//   private final NetworkTableEntry ntDutyCycle = ntTable.getEntry("Ball Grabber Velocity");

    public BallGrabber() {
       this.grabberMotor = new TalonFX(1, "grabberMotorRange");
       this.grabberMotorRange = new CANrange(0); 
       

    }


public void periodic(){

    var range = grabberMotorRange.getDistance().getValueAsDouble();
    if((range < BALL_DETECTION_DISTANCE && grabberMotor.getStatorCurrent().getValueAsDouble() < MAX_CURRENT_RUNNING)){
        grabberMotor.set(-0.1);
        SmartDashboard.putBoolean("Ball Detected", true);
        System.out.println("ball detect");
    } else if (range > BALL_DETECTION_DISTANCE) {
        SmartDashboard.putBoolean("Ball Detected", false);
        grabberMotor.set(0.1);
    }
    else{
        grabberMotor.set(0.1); 
    }
    
}
public void runIntakeOuttake(){
 var range = grabberMotorRange.getDistance().getValueAsDouble();
    if((range < BALL_DETECTION_DISTANCE && grabberMotor.getStatorCurrent().getValueAsDouble() < MAX_CURRENT_RUNNING)){
        grabberMotor.set(OUTTAKE_SPEED);
    } else{
        grabberMotor.set(INTAKE_SPEED);
    }
}



public void stop(){
    grabberMotor.set(0);
}

public double getMotorVelocity(){
    return grabberMotor.get();
}

public double getDutyCycle() {
    return grabberMotor.get(); 
}
}
