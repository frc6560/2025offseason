
package com.team6560.frc2025.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.MotorController; 
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANrange; 

import static com.team6560.frc2025.utility.NetworkTable.NtValueDisplay.ntDispTab; 


import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.team6560.frc2025.Constants;

public class BallGrabber extends SubsystemBase {
    private TalonFX grabberMotor; 
    

    private static final int GRABBER_MOTOR_ID = 25;
    private static final double INTAKE_SPEED = -0.3;
    private static final double OUTTAKE_SPEED = 0.7; 

    private static final double MAX_CURRENT_RUNNING = 30; 
    private static final double GAMEPEICE_CURRENT = 20; //  threshold to detect if a ball is present
    public BallGrabber() {
        this.grabberMotor = new TalonFX(GRABBER_MOTOR_ID, MotorType.kBrushless);
        ntDispTab("Ball Grabber")
            .add("Ball Grabber Current", () -> grabberMotor.getOutputCurrent())
            .add("Ball Grabber Voltage", () -> grabberMotor.getBusVoltage())
            .add("Ball Grabber Speed", () -> grabberMotor.get())
            .add("Ball Grabber Duty Cycle", () -> getMotorVelocity());


}


public void periodic(){
    if((grabberMotor.getOutputCurrent() > GAMEPEICE_CURRENT && grabberMotor.getOutputCurrent() < MAX_CURRENT_RUNNING)){
        grabberMotor.set(-0.1);
        SmartDashboard.putBoolean("Ball Detected", true);
    } else if (grabberMotor.getOutputCurrent() < GAMEPEICE_CURRENT) {
        SmartDashboard.putBoolean("Ball Detected", false);
        grabberMotor.set(0.1);
    }
    else{
        grabberMotor.set(0.1); 
    }
}
public void runIntakeOuttake(){
    if((grabberMotor.getOutputCurrent() > GAMEPEICE_CURRENT && grabberMotor.getOutputCurrent() < MAX_CURRENT_RUNNING)){
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
