package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Size;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.PIDConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;


@TeleOp
public class autonomousBlue extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {



        BNO055IMU imu;

        DcMotor frontLeft = hardwareMap.dcMotor.get("fl");
        DcMotor backLeft = hardwareMap.dcMotor.get("bl");
        DcMotor frontRight = hardwareMap.dcMotor.get("fr");
        DcMotor backRight = hardwareMap.dcMotor.get("br");
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        Servo intakeServo = hardwareMap.servo.get("intakeServo");
        //CRServo hopUpServo = hardwareMap.crservo.get("hopUpServo");
        AnalogInput analogInput = hardwareMap.get(AnalogInput.class, "myanaloginput");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        double refrenceAngle = Math.toRadians(0);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        float intakeMotorPower = gamepad1.left_trigger;
        float outtakeMotorPower = gamepad1.right_trigger;
        double position = analogInput.getVoltage() / 3.3 * 360;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        telemetry.setAutoClear(true);
        telemetry.update();
        float driveSpeed = 1;

        int[] portalList = VisionPortal.makeMultiPortalView(4, VisionPortal.MultiPortalLayout.HORIZONTAL);

        AprilTagProcessor tagProcessor1 = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setLensIntrinsics(513.241, 513.241, 327.961, 242.265)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        AprilTagProcessor tagProcessor2 = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setLensIntrinsics(513.241, 513.241, 327.961, 242.265)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        AprilTagProcessor tagProcessor3 = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setLensIntrinsics(513.241, 513.241, 327.961, 242.265)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        AprilTagProcessor tagProcessor4 = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setLensIntrinsics(513.241, 513.241, 327.961, 242.265)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        VisionPortal visionPortal1 = new VisionPortal.Builder()
                .addProcessor(tagProcessor1)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .setLiveViewContainerId(portalList[0])
                .build();

        VisionPortal visionPortal2 = new VisionPortal.Builder()
                .addProcessor(tagProcessor2)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .setLiveViewContainerId(portalList[1])
                .build();

        VisionPortal visionPortal3 = new VisionPortal.Builder()
                .addProcessor(tagProcessor3)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 3"))
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .setLiveViewContainerId(portalList[2])
                .build();

        VisionPortal visionPortal4 = new VisionPortal.Builder()
                .addProcessor(tagProcessor4)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 4"))
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .setLiveViewContainerId(portalList[3])
                .build();

        while (visionPortal1.getCameraState() != VisionPortal.CameraState.STREAMING) {
        }

        ExposureControl exposure = visionPortal1.getCameraControl(ExposureControl.class);
        exposure.setMode(ExposureControl.Mode.Manual);
        exposure.setExposure(15, TimeUnit.MILLISECONDS);

        GainControl gain = visionPortal1.getCameraControl(GainControl.class);
        gain.setGain(255);

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {

            frontLeft.setPower(frontLeftPower * driveSpeed);
            backLeft.setPower(backLeftPower * driveSpeed);
            frontRight.setPower(frontRightPower * driveSpeed);
            backRight.setPower(backRightPower * driveSpeed);

            if (gamepad1.left_trigger > 0) {
                intakeMotor.setPower(-intakeMotorPower);
                //hopUpServo.setPower(intakeMotorPower);

            } else if (gamepad1.right_trigger > 0) {
                intakeMotor.setPower(outtakeMotorPower);
                //hopUpServo.setPower(outtakeMotorPower);
            } else {
                intakeMotor.setPower(0);
                //hopUpServo.setPower(0);
            }
            if (gamepad1.dpad_up) {
                intakeServo.setPosition(0.45);
            } else if (gamepad1.dpad_down) {
                intakeServo.setPosition(0.53);
            }
            if (tagProcessor1.getDetections().size() > 0) {
                AprilTagDetection tag = tagProcessor1.getDetections().get(0);
            }
            telemetry.update();

        }
    }

    public double PIDControl(double refrence, double state) {
        double integralSum = 0;
        double Kp = PIDConstants.Kp;
        double Ki = PIDConstants.Ki;
        double Kd = PIDConstants.Kd;
        Drivetrain drivetrain = new Drivetrain();
        ElapsedTime timer = new ElapsedTime();
        double lastError = 0;
        double error = angleWrap(refrence - state);
        telemetry.addData("Error: ", error);
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / (timer.seconds());
        lastError = error;
        timer.reset();
        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
    }

    public double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }
        return radians;
    }
}