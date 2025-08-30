package frc.robot.subsystems;


import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PipeGrabber extends SubsystemBase {
    
    private SparkFlex grabberMotor;

    private static final int GRABBER_MOTOR_ID = 17;

    private static final double INTAKE_SPEED = 0.5;
    private static final double OUTTAKE_SPEED = -0.3;

    public PipeGrabber() {
        this.grabberMotor = new SparkFlex(GRABBER_MOTOR_ID, MotorType.kBrushless);
    
    }

    public void runIntake(){
        
        grabberMotor.set(INTAKE_SPEED);
        // return;
        // if (!hasGamePiece()){
        //     grabberMotor.set(0.5);
        //     // grabberMotor.set(INTAKE_SPEED);
        // } else {
        //     grabberMotor.set(0.0);
        // }
    }

    public void runGrabberOuttake(){
        grabberMotor.set(OUTTAKE_SPEED);
        // System.out.println("Worked");
    }

    public void stop() {
        grabberMotor.set(0);
    }

    public double getDutyCycle() {
        return grabberMotor.get();
    }

    public boolean hasGamePiece() {
        // return this.grabberMotor.getReverseLimitSwitch().isPressed();                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             ().isPressed();
        // with caleb inversion
        return false;
    }
    
    public double getOutputCurrent() {
        return grabberMotor.getOutputCurrent();
    }
}