package org.firstinspires.ftc.teamcode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Arm.Elbow;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Gripper;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Lift;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
public class Teleop extends OpMode { //Creating Instances and certain inputs
    private MecanumDrive drive;
    private GamepadEx driverOp;
    private GamepadEx scoreOP;
    private Elbow ElbowClass;
    private Lift LiftClass;
    public Gripper GripperClass;
    public double driveSpeedMultiplier = (-0.5);
    public double pressedR = scoreOP.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
    public double pressedL = scoreOP.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
    @Override
    public void init() { //Creating Objects

        LiftClass = new Lift(hardwareMap);
        ElbowClass = new Elbow(hardwareMap);
        GripperClass = new Gripper();

        Motor frontLeft = new Motor(hardwareMap, "fl");
        Motor frontRight = new Motor(hardwareMap, "fr");
        Motor backLeft = new Motor(hardwareMap, "bl");
        Motor backRight = new Motor(hardwareMap, "br");

        drive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
        driverOp = new GamepadEx(gamepad1);
        scoreOP = new GamepadEx(gamepad2);
    }
    @Override
    public void loop() {
        //Lift Slide Movement (Left_Stick_Y - Gamepad 2)
        LiftClass.liftMovement();

        //Elbow Joint Movement Up (Dpad_Up - Gamepad 2)
        scoreOP.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(ElbowClass.servoScore());

        //Elbow Joint Movement Down (Dpad_Down - Gamepad 2)
        scoreOP.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(ElbowClass.servoGround());

        //Gripper 1 Close (Right Trigger - Gamepad 2)
        if(pressedR > 0.9) {
            GripperClass.grip1Grip();
            //need to add bool to keep it pressed
        }

        //Gripper 2 Close (Left Trigger - Gamepad 2)
        if(pressedL > 0.9) {
            GripperClass.grip2Grip();
            //need to add bool to keep it pressed
        }

        //Gripper 1 Open (Right Bumper - Gamepad 2)
        scoreOP.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(GripperClass.grip1Release());
                //switch boolean condition

        //Gripper 2 Open (Left Bumper - Gamepad 2)
        scoreOP.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(GripperClass.grip2Release());
                //switch boolean condition

        //Flipper Up/Down (A Toggle - Gamepad 2)
        scoreOP.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new ConditionalCommand(
                        new InstantCommand(GripperClass::flipUp, GripperClass),
                        new InstantCommand(GripperClass::flipDown, GripperClass),
                        () -> {
                            GripperClass.toggleFlip();
                            return GripperClass.activeFlip();
                        }));

        //Drivetrain Movement (DO NOT EDIT)
        drive.driveRobotCentric(
                driverOp.getLeftX() * driveSpeedMultiplier,
                driverOp.getLeftY() * driveSpeedMultiplier,
                driverOp.getRightX() * driveSpeedMultiplier
        );
    }
}