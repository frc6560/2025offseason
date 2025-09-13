package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;

/**
 * A "fake Xbox" using a normal keyboard through the Driver Station in simulation.
 * Each number key (1–5 on your Mac keyboard) maps to an arm preset.
 */
public class keyboardControls {
    private final Joystick keyboard;

    public keyboardControls(int port) {
        keyboard = new Joystick(port); // port must match Driver Station USB slot
    }

    // map number keys on keyboard to arm states
    public boolean goToReef() {
        return keyboard.getRawButton(1); // press "1"
    }

    public boolean goToPickup() {
        return keyboard.getRawButton(2); // press "2"
    }

    public boolean goToGround() {
        return keyboard.getRawButton(3); // press "3"
    }

    public boolean goToProcessor() {
        return keyboard.getRawButton(4); // press "4"
    }

    public boolean goToStow() {
        return keyboard.getRawButton(5); // press "5"
    }
}