package org.firstinspires.ftc.teamcode.Subsystems.Arm;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class Gripper extends SubsystemBase {
    //Grip1 is the top servo
    //Grip2 is the bottom servo
    //Flip is the servo which flips the mechanism
    public SimpleServo grip1, grip2, flip;
    private boolean flip_active;
    int gripPosition = 0;
    int releasePosition = 1;
    int flipUp = 1;
    int flipDown = 0;
    public void Grip1(HardwareMap hardwareMap) {
        this.grip1 = new SimpleServo(hardwareMap, "grip1", 0.0, 180.0);
    }
    public void Grip2(HardwareMap hardwareMap) {
        this.grip2 = new SimpleServo(hardwareMap, "grip2", 0.0, 180.0);
    }
    public void Flip(HardwareMap hardwareMap) {
        this.flip = new SimpleServo(hardwareMap, "flip", 0.0, 180.0);
        flip_active = true;
    }
    public void toggleFlip() {
        flip_active = !flip_active;
    }
    public boolean activeFlip() {
        return flip_active;
    }
    public Command grip1Grip() {
        grip1.setPosition(gripPosition);
        return null;
    }
    public Command grip1Release() {
        grip1.setPosition(releasePosition);
        return null;
    }
    public Command grip2Grip() {
        grip1.setPosition(gripPosition);
        return null;
    }
    public Command grip2Release() {
        grip1.setPosition(releasePosition);
        return null;
    }
    public Command flipUp() {
        flip.setPosition(flipUp);
        return null;
    }
    public Command flipDown() {
        flip.setPosition(flipDown);
        return null;
    }
}