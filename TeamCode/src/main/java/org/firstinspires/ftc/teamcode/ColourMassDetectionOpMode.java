package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Utilities.ColourMassDetectionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

@Autonomous
public class ColourMassDetectionOpMode extends OpMode {
	private VisionPortal visionPortal;
	private ColourMassDetectionProcessor colourMassDetectionProcessor;
	@Override
	public void init() {

		//our red range
		Scalar lower = new Scalar(0, 100, 100);
		Scalar upper = new Scalar(30, 255, 255);

		//our blue range
		//Scalar lower = new Scalar(120, 50, 50);
		//Scalar upper = new Scalar(125, 255, 255);

		//initial red
		//Scalar lower = new Scalar(150, 100, 100);
		//Scalar upper = new Scalar(180, 255, 255);
		double minArea = 100;
		
		colourMassDetectionProcessor = new ColourMassDetectionProcessor(
				lower,
				upper,
				() -> minArea,
				() -> 213, // the left dividing line
				() -> 426 // the right dividing line
		);
		visionPortal = new VisionPortal.Builder()
				.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // the camera on your robot is named "Webcam 1"
				.addProcessor(colourMassDetectionProcessor)
				.build();
	}
	@Override
	public void init_loop() {
		telemetry.addData("Currently Recorded Position", colourMassDetectionProcessor.getRecordedPropPosition());
		telemetry.addData("Camera State", visionPortal.getCameraState());
		telemetry.addData("Currently Detected Mass Center", "x: " + colourMassDetectionProcessor.getLargestContourX() + ", y: " + colourMassDetectionProcessor.getLargestContourY());
		telemetry.addData("Currently Detected Mass Area", colourMassDetectionProcessor.getLargestContourArea());
	}
	@Override
	public void start() {
		if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
			visionPortal.stopLiveView();
			visionPortal.stopStreaming();
		}

		ColourMassDetectionProcessor.PropPositions recordedPropPosition = colourMassDetectionProcessor.getRecordedPropPosition();

		if (recordedPropPosition == ColourMassDetectionProcessor.PropPositions.UNFOUND) {
			recordedPropPosition = ColourMassDetectionProcessor.PropPositions.MIDDLE;
		}
		

		switch (recordedPropPosition) {
			case LEFT:

				break;
			case MIDDLE:

				break;
			case RIGHT:

				break;
		}
	}
	@Override
	public void loop() {
	
	}
	@Override
	public void stop() {
		colourMassDetectionProcessor.close();
		visionPortal.close();
	}
}