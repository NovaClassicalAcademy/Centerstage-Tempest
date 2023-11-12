package org.firstinspires.ftc.teamcode.Subsystems.Robot;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.Servo;

public class Gripper extends SubsystemBase {
    private final Servo grip1;
    public static double gripPosition = 0.5;
    public static double releasePosition = 1;

    public Gripper(Servo grip1) {
        this.grip1 = grip1;
    }

    public Command grip1Grip() {
        return new InstantCommand(() -> {
            grip1.setPosition(gripPosition);
        }, this).andThen(
                new WaitCommand(500)
        );
    }

    public Command grip1Release() {
        return new InstantCommand(() -> {
            grip1.setPosition(releasePosition);
        });
    }
}