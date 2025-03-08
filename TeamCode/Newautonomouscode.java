package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.Tfod;

@Autonomous(name = "Newautonomouscode (Blocks to Java)")
public class Newautonomouscode extends LinearOpMode {

  private VuforiaCurrentGame vuforiaPOWERPLAY;
  private Tfod tfod;
  private DcMotor backLeftMotor;
  private Servo ClawAsServo;
  private CRServo WinchAsCRServo;
  private DcMotor frontLeftMotorAsDcMotor;
  private DcMotor backRightMotorAsDcMotor;
  private DcMotor frontRightMotorAsDcMotor;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    float MaxConfidence;
    int SignalSleevePosition;
    List<Recognition> recognitions;
    int index;
    Recognition recognition;
    double Motor_compensation;

    vuforiaPOWERPLAY = new VuforiaCurrentGame();
    tfod = new Tfod();
    backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
    ClawAsServo = hardwareMap.get(Servo.class, "ClawAsServo");
    WinchAsCRServo = hardwareMap.get(CRServo.class, "WinchAsCRServo");
    frontLeftMotorAsDcMotor = hardwareMap.get(DcMotor.class, "frontLeftMotorAsDcMotor");
    backRightMotorAsDcMotor = hardwareMap.get(DcMotor.class, "backRightMotorAsDcMotor");
    frontRightMotorAsDcMotor = hardwareMap.get(DcMotor.class, "frontRightMotorAsDcMotor");

    // Sample TFOD Op Mode using a Custom Model
    // Set initial variables
    MaxConfidence = 0;
    SignalSleevePosition = 0;
    // Initialize Vuforia to provide TFOD with camera
    // images.
    // The following block uses the device's back camera.
    // The following block uses a webcam.
    vuforiaPOWERPLAY.initialize(
        "", // vuforiaLicenseKey
        hardwareMap.get(WebcamName.class, "Webcam 1"), // cameraName
        "", // webcamCalibrationFilename
        false, // useExtendedTracking
        false, // enableCameraMonitoring
        VuforiaLocalizer.Parameters.CameraMonitorFeedback.NONE, // cameraMonitorFeedback
        0, // dx
        0, // dy
        0, // dz
        AxesOrder.XZY, // axesOrder
        90, // firstAngle
        90, // secondAngle
        0, // thirdAngle
        true); // useCompetitionFieldTargetLocations
    // Initialize TFOD before waitForStart.
    // Use the Manage page to upload your custom model.
    // In the next block, replace
    // YourCustomModel.tflite with the name of your
    // custom model.
    // Enable following block to zoom in on target.
    telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
    telemetry.addData(">", "Press Play to start");
    telemetry.update();
    // Wait for start command from Driver Station.
    waitForStart();
    resetRuntime();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (getRuntime() < 3) {
        // Get a list of recognitions from TFOD.
        recognitions = tfod.getRecognitions();
        // If list is empty, inform the user. Otherwise, go through list and display info for each recognition.
        if (JavaUtil.listLength(recognitions) == 0) {
          telemetry.addData("TFOD", "No items detected.");
        } else {
          index = 0;
          // Iterate through list and call a function to display info for each recognized object.
          for (Recognition recognition_item : recognitions) {
            recognition = recognition_item;
            // Display info.
            displayInfo(index);
            if (recognition.getConfidence() > MaxConfidence) {
              telemetry.addData("Recognized new highest confidence at", index);
              SignalSleevePosition = recognition.getLabel();
              MaxConfidence = recognition.getConfidence();
            }
            // Increment index.
            index = index + 1;
          }
          telemetry.addData("Position", SignalSleevePosition);
          telemetry.addData("Confidence", MaxConfidence);
          telemetry.addData("Runtime", getRuntime());
          telemetry.update();
        }
      }
    }
    sleep(1000);
    // set direction maually
    // Deactivate TFOD.
    tfod.deactivate();
    // Reverse the right side motors
    // Reverse left motors if you are using NeveRests
    backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
    backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    // CLAW begins fully open = 0.5
    // this should reduce the speed of the chainless motors
    Motor_compensation = 0.7;
    if (opModeIsActive()) {
      // close claw
      ClawAsServo.setPosition(1);
      sleep(1500);
      // move side to side
      if (SignalSleevePosition == "leaf1") {
        // raise arm
        WinchAsCRServo.setPower(1);
        sleep(1200);
        WinchAsCRServo.setPower(0);
        // leaf is move left
        Motor_compensation = 0.55;
        frontLeftMotorAsDcMotor.setPower(-1 * Motor_compensation);
        backRightMotorAsDcMotor.setPower(-1 * Motor_compensation);
        backLeftMotor.setPower(1);
        frontRightMotorAsDcMotor.setPower(1);
        telemetry.addData("direction", "left");
        telemetry.update();
        sleep(1000);
        frontLeftMotorAsDcMotor.setPower(0 * Motor_compensation);
        backRightMotorAsDcMotor.setPower(0 * Motor_compensation);
        backLeftMotor.setPower(0);
        frontRightMotorAsDcMotor.setPower(0);
        // lower arm
        WinchAsCRServo.setPower(-1);
        sleep(1200);
        WinchAsCRServo.setPower(0);
      } else if (SignalSleevePosition == "rose2") {
        // Should be ring3 - CHANGE BACK IF WE FIX IMAGE RECOGNITION
        // raise arm
        WinchAsCRServo.setPower(1);
        sleep(1200);
        WinchAsCRServo.setPower(0);
        // ring detected move right
        Motor_compensation = 0.55;
        frontLeftMotorAsDcMotor.setPower(1 * Motor_compensation);
        backRightMotorAsDcMotor.setPower(1 * Motor_compensation);
        backLeftMotor.setPower(-1);
        frontRightMotorAsDcMotor.setPower(-1);
        telemetry.addData("direction", "right");
        telemetry.update();
        sleep(700);
        frontLeftMotorAsDcMotor.setPower(0 * Motor_compensation);
        backRightMotorAsDcMotor.setPower(0 * Motor_compensation);
        backLeftMotor.setPower(0);
        frontRightMotorAsDcMotor.setPower(0);
        // lower arm
        WinchAsCRServo.setPower(-1);
        sleep(1200);
        WinchAsCRServo.setPower(0);
      } else {
        // Should be rose2 - CHANGE BACK IF WE FIX IMAGE RECOGNITION
        telemetry.addData("direction", "forward");
        telemetry.update();
      }
      // move forward 1 second
      Motor_compensation = 0.7;
      frontLeftMotorAsDcMotor.setPower(-1 * Motor_compensation);
      backRightMotorAsDcMotor.setPower(-1 * Motor_compensation);
      backLeftMotor.setPower(-1);
      frontRightMotorAsDcMotor.setPower(-1);
      sleep(800);
      frontLeftMotorAsDcMotor.setPower(0 * Motor_compensation);
      backRightMotorAsDcMotor.setPower(0 * Motor_compensation);
      backLeftMotor.setPower(0);
      frontRightMotorAsDcMotor.setPower(0);
    }

    vuforiaPOWERPLAY.close();
    tfod.close();
  }

  /**
   * Describe this function...
   */
  private void displayInfo(int i) {
  }
}
