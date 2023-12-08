package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

import android.util.Size;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;


@Autonomous(name = "tag")
public class lineAT extends LinearOpMode {
    int angleSensitivity = 2;
    int strafeSensitivity = 6;
    boolean Angled;
    boolean Centered;
    boolean DistanceAway;
    boolean aligned = false;

    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    public void SetDistance(int distance) {
        DcMotor frontLeft = hardwareMap.dcMotor.get("fl");
        DcMotor backLeft = hardwareMap.dcMotor.get("bl");
        DcMotor frontRight = hardwareMap.dcMotor.get("fr");
        DcMotor backRight = hardwareMap.dcMotor.get("br");
        if (distance < 11) {
            float Dpower = 0.1f;
            frontLeft.setPower(-Dpower);
            frontRight.setPower(-Dpower);
            backLeft.setPower(-Dpower);
            backRight.setPower(-Dpower);
        } else if (distance > 12) {
            float Dpower = 0.1f;
            frontLeft.setPower(Dpower);
            frontRight.setPower(Dpower);
            backLeft.setPower(Dpower);
            backRight.setPower(Dpower);
        } else {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            DistanceAway = true;
        }
    }

    public void SetAngle(int AngleError) {
        DcMotor frontLeft = hardwareMap.dcMotor.get("fl");
        DcMotor backLeft = hardwareMap.dcMotor.get("bl");
        DcMotor frontRight = hardwareMap.dcMotor.get("fr");
        DcMotor backRight = hardwareMap.dcMotor.get("br");

        if (AngleError > angleSensitivity) {
            float Spower = 0.1f;
            frontLeft.setPower(-Spower);
            frontRight.setPower(Spower);
            backLeft.setPower(-Spower);
            backRight.setPower(Spower);
        } else if (AngleError < -angleSensitivity) {
            float Spower = 0.1f;
            frontLeft.setPower(Spower);
            frontRight.setPower(-Spower);
            backLeft.setPower(Spower);
            backRight.setPower(-Spower);
        } else {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            Angled = true;
        }
    }

    public void Center(float StrafeError) {
        DcMotor frontLeft = hardwareMap.dcMotor.get("fl");
        DcMotor backLeft = hardwareMap.dcMotor.get("bl");
        DcMotor frontRight = hardwareMap.dcMotor.get("fr");
        DcMotor backRight = hardwareMap.dcMotor.get("br");
        if (StrafeError < (320 - strafeSensitivity)) {
            float StPower = 0.2f;
            frontLeft.setPower(-StPower);
            frontRight.setPower(StPower);
            backLeft.setPower(StPower);
            backRight.setPower(-StPower);
        } else if (StrafeError > (320 + strafeSensitivity)) {
            float StPower = 0.2f;
            frontLeft.setPower(StPower);
            frontRight.setPower(-StPower);
            backLeft.setPower(-StPower);
            backRight.setPower(StPower);
        } else {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            Centered = true;
            aligned = true;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontLeft = hardwareMap.dcMotor.get("fl");
        DcMotor backLeft = hardwareMap.dcMotor.get("bl");
        DcMotor frontRight = hardwareMap.dcMotor.get("fr");
        DcMotor backRight = hardwareMap.dcMotor.get("br");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);

        backLeft.setDirection(DcMotor.Direction.REVERSE);





        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {

                autoAlign();

                //telemetry.addData("center", tag.center);
                //telemetry.addData("dist from tag: ", distance);
                telemetry.addData("Centered", Centered);
                telemetry.addData("Angled", Angled);
                telemetry.addData("align", aligned);
                //telemetry.addData("yaw", AngleError);
                //telemetry.addData("strafeError", StrafeError);
                //telemetry.addData("fps", myFPS);
                //telemetry.update();
        }
    }

    public void autoAlign() {
        AprilTagProcessor tagProcessor1 = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(513.241, 513.241, 327.961, 242.265)
                .setOutputUnits(INCH, AngleUnit.DEGREES)
                .build();

        VisionPortal visionPortal1 = new VisionPortal.Builder()
                .addProcessor(tagProcessor1)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .build();

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);

        while (visionPortal1.getCameraState() != VisionPortal.CameraState.STREAMING) {
        }

        ExposureControl exposure = visionPortal1.getCameraControl(ExposureControl.class);
        exposure.setMode(ExposureControl.Mode.Manual);
        exposure.setExposure(30, TimeUnit.MILLISECONDS);

        GainControl gain = visionPortal1.getCameraControl(GainControl.class);
        gain.setGain(255);


        AprilTagDetection targetTag = null;

        if (tagProcessor1.getDetections().size() > 0) {

            for(AprilTagDetection tag: tagProcessor1.getDetections()) {
                if (tag.metadata.id == 3) {
                    targetTag = tag;
                    break;
                }
            }
                if (targetTag != null) {
                    float myFPS = visionPortal1.getFps();
                    int AngleError = (int) targetTag.ftcPose.yaw;
                    int distance = (int) targetTag.ftcPose.range;
                    float StrafeError = (float) targetTag.center.x;

                    if (!Angled) {
                        SetAngle(AngleError);
                    }
                    if (!Centered) {
                        Center(StrafeError);
                    }
                    imu.resetYaw();

                    visionPortal1.stopLiveView();
            }
        }
    }
}
