/*package org.firstinspires.ftc.teamcode.TeleOp;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.START;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.RIGHT_TRIGGER;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Disabled
@Config
@TeleOp
public class Teleop extends BaseOpMode { //Creating Instances and certain inputs

    @Override
    public void initialize() { //Creating Objects
        super.initialize();

        gb1(LEFT_BUMPER).whileHeld(
                drive.slowMode(gamepadEx1::getLeftX, gamepadEx1::getLeftY, gamepadEx1::getRightX)
        );

        gb1(START).toggleWhenPressed(
                drive.fieldCentric(gamepadEx1::getLeftX, gamepadEx1::getLeftY, gamepadEx1::getRightX, imu::getHeading),
                drive.robotCentric(gamepadEx1::getLeftX, gamepadEx1::getLeftY, gamepadEx1::getRightX)
        );

        gb1(START).and(gb1(RIGHT_TRIGGER)).toggleWhenActive(
                new InstantCommand(), true
        );

        gb2(LEFT_TRIGGER).whenActive(
                GripperSubsystem.grip1Grip()
        );

        gb2(LEFT_BUMPER).whenPressed(
                GripperSubsystem.grip1Release()
        );

        double liftMotorPower = gamepadEx2.getLeftY();
        double elbowMotorPower = gamepadEx2.getRightY();

        LiftSubsystem.liftMovement(liftMotorPower);
        ElbowSubsystem.ElbowMovement(elbowMotorPower);

        register(drive, LiftSubsystem, GripperSubsystem, ElbowSubsystem);
        drive.setDefaultCommand(drive.robotCentric(gamepadEx1::getLeftX, gamepadEx1::getLeftY, gamepadEx1::getRightX));
    }

    @Override
    public void run() {
        super.run();

    }
}
*/

