package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utilities.GamepadTrigger;
import org.firstinspires.ftc.teamcode.Subsystems.Robot.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Robot.Elbow;
import org.firstinspires.ftc.teamcode.Subsystems.Robot.Gripper;
import org.firstinspires.ftc.teamcode.Subsystems.Robot.Lift;
import org.firstinspires.ftc.teamcode.Utilities.TriggerGamepadEx;

public class BaseOpMode extends CommandOpMode {
    protected MotorEx frontLeft, frontRight, backLeft, backRight, elbow;
    protected DcMotorEx lift;
    protected Servo grip1;
    protected Lift LiftSubsystem;
    protected Gripper GripperSubsystem;
    protected Elbow ElbowSubsystem;
    protected MecanumDriveSubsystem drive;
    protected GamepadEx gamepadEx1;
    protected GamepadEx gamepadEx2;
    protected TriggerGamepadEx triggerGamepadEx1;
    protected TriggerGamepadEx triggerGamepadEx2;
    protected RevIMU imu;
    protected double fx = 513.241;
    protected double fy = 513.241;
    protected double cx = 327.961;
    protected double cy = 242.265;

    //(513.241, 513.241, 327.961, 242.265)
    @Override
    public void initialize() {
        tad("Mode", "Starting initialization");
        telemetry.update();
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        initHardware();
        setupHardware();

        imu = new RevIMU(hardwareMap);
        imu.init();

        drive = new MecanumDriveSubsystem(frontLeft, frontRight, backLeft, backRight, imu);

        triggerGamepadEx1 = new TriggerGamepadEx(gamepad1, gamepadEx1);
        triggerGamepadEx2 = new TriggerGamepadEx(gamepad2, gamepadEx2);

        LiftSubsystem = new Lift(lift);
        ElbowSubsystem = new Elbow(elbow);
        GripperSubsystem = new Gripper(grip1);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        tad("Mode", "Done initializing");

        telemetry.update();
    }
    protected void initHardware() {
        try {
            frontLeft = new MotorEx(hardwareMap, "fl");
            frontRight = new MotorEx(hardwareMap, "fr");
            backLeft = new MotorEx(hardwareMap, "bl");
            backRight = new MotorEx(hardwareMap, "br");
            lift = hardwareMap.get(DcMotorEx.class, "lift");
            elbow = new MotorEx(hardwareMap, "elbow");
            grip1 = hardwareMap.get(Servo.class, "grip1");
        }
        catch(Exception e) {
            tad("ERROR", "Motor init failed");
        }
    }
    protected void setupHardware() {
        frontLeft.setInverted(true);
        backLeft.setInverted(true);
        lift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }
    public GamepadButton gb1(GamepadKeys.Button button) {
        return  gamepadEx1.getGamepadButton(button);
    }
    public GamepadButton gb2(GamepadKeys.Button button) {
        return  gamepadEx2.getGamepadButton(button);
    }
    public GamepadTrigger gb1(GamepadKeys.Trigger trigger) {
        return  triggerGamepadEx1.getGamepadTrigger(trigger);
    }
    public GamepadTrigger gb2(GamepadKeys.Trigger trigger) {
        return  triggerGamepadEx1.getGamepadTrigger(trigger);
    }
    // telemetry add data = tad
    protected void tad(String caption, Object value) {
        telemetry.addData(caption, value);
    }
    protected void tal(String caption) {
        telemetry.addLine(caption);
    }
}