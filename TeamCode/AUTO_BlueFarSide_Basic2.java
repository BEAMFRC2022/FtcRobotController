package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "AUTO_BlueFarSide_Basic2 (Blocks to Java)")
public class AUTO_BlueFarSide_Basic2 extends LinearOpMode {

  private DcMotor Shoulder;
  private DcMotor elbow;
  private CRServo frontLeftMotor;
  private DcMotor frontRghtMotor;
  private DcMotor backLeftMotor;
  private CRServo backRightMotor;
  private CRServo Gripper;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    int Target_Shoulder;
    int Target_Elbow;
    double Drive_power;

    Shoulder = hardwareMap.get(DcMotor.class, "Shoulder");
    elbow = hardwareMap.get(DcMotor.class, "elbow");
    frontLeftMotor = hardwareMap.get(CRServo.class, "frontLeftMotor");
    frontRghtMotor = hardwareMap.get(DcMotor.class, "frontRghtMotor");
    backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
    backRightMotor = hardwareMap.get(CRServo.class, "backRightMotor");
    Gripper = hardwareMap.get(CRServo.class, "Gripper");

    Target_Shoulder = 0;
    Target_Elbow = 0;
    // Reset Shoulder and Elbow decoder - both must be in physical 0 position
    Shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    // Configure Drive motors
    frontLeftMotor.setDirection(CRServo.Direction.REVERSE);
    frontRghtMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    // Reverse the right side motors
    frontRghtMotor.setDirection(DcMotor.Direction.REVERSE);
    // Set Shoulder and Elbow to run to position
    Shoulder.setTargetPosition(Target_Shoulder);
    Shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    ((DcMotorEx) Shoulder).setVelocity(250);
    elbow.setTargetPosition(Target_Elbow);
    elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    ((DcMotorEx) elbow).setVelocity(200);
    waitForStart();
    if (opModeIsActive()) {
      sleep(500);
      // Move Elbow to travel position
      Target_Elbow = 100;
      elbow.setTargetPosition(Target_Elbow);
      sleep(750);
      // Drive Forward for 500 ms to center spike
      Drive_power = -0.5;
      frontLeftMotor.setPower(Drive_power);
      backLeftMotor.setPower(Drive_power);
      frontRghtMotor.setPower(Drive_power);
      backRightMotor.setPower(Drive_power);
      sleep(1400);
      frontLeftMotor.setPower(0);
      backLeftMotor.setPower(0);
      frontRghtMotor.setPower(0);
      backRightMotor.setPower(0);
      sleep(500);
      telemetry.addData("Position FB", elbow.getCurrentPosition());
      telemetry.update();
      // Drop Arm to release first Pixel
      Target_Shoulder = 0;
      Target_Elbow = 0;
      Shoulder.setTargetPosition(Target_Shoulder);
      elbow.setTargetPosition(Target_Elbow);
      sleep(1000);
      // Release one pixel
      Gripper.setPower(0.75);
      // 1100 to release 1 pixel
      sleep(1100);
      Gripper.setPower(0);
      sleep(500);
      // Move Arm to low scoring position
      Target_Shoulder = 0;
      Target_Elbow = 100;
      Shoulder.setTargetPosition(Target_Shoulder);
      elbow.setTargetPosition(Target_Elbow);
      sleep(3000);
      // Move back a little
      Drive_power = 0.5;
      frontLeftMotor.setPower(Drive_power);
      backLeftMotor.setPower(Drive_power);
      frontRghtMotor.setPower(Drive_power);
      backRightMotor.setPower(Drive_power);
      sleep(250);
      // Rotate right
      Drive_power = -0.5;
      frontLeftMotor.setPower(Drive_power);
      backLeftMotor.setPower(Drive_power);
      frontRghtMotor.setPower(Drive_power * -1);
      backRightMotor.setPower(Drive_power * -1);
      sleep(650);
      // Move toward backdrop in reverse mode from far side
      Drive_power = 0.5;
      frontLeftMotor.setPower(Drive_power);
      backLeftMotor.setPower(Drive_power);
      frontRghtMotor.setPower(Drive_power);
      backRightMotor.setPower(Drive_power);
      sleep(2000);
      // Move the arm into scoring position
      ((DcMotorEx) Shoulder).setVelocity(250);
      Target_Shoulder = 600;
      Target_Elbow = 600;
      Shoulder.setTargetPosition(Target_Shoulder);
      elbow.setTargetPosition(Target_Elbow);
      sleep(1900);
      // Stop robot
      Drive_power = 0;
      frontLeftMotor.setPower(Drive_power);
      backLeftMotor.setPower(Drive_power);
      frontRghtMotor.setPower(Drive_power);
      backRightMotor.setPower(Drive_power);
      sleep(1000);
      // Release yellow pixel on the backdrop
      Gripper.setPower(0.75);
      // 1100 to release 1 pixel
      sleep(1100);
      Gripper.setPower(0);
      sleep(1000);
      // Back away from the backdrop and move to the left
      Drive_power = -0.5;
      frontLeftMotor.setPower(Drive_power);
      backLeftMotor.setPower(Drive_power);
      frontRghtMotor.setPower(Drive_power);
      backRightMotor.setPower(Drive_power);
      sleep(300);
      // Move arm back to 0 position
      frontLeftMotor.setPower(0);
      backLeftMotor.setPower(0);
      frontRghtMotor.setPower(0);
      backRightMotor.setPower(0);
      Target_Elbow = 0;
      Target_Shoulder = 0;
      Shoulder.setTargetPosition(Target_Shoulder);
      elbow.setTargetPosition(Target_Elbow);
      sleep(1000);
      // Crab walk to Backstage
      Drive_power = 0.5;
      backLeftMotor.setPower(Drive_power);
      frontRghtMotor.setPower(Drive_power);
      sleep(1000);
      backRightMotor.setPower(Drive_power * -1);
      frontLeftMotor.setPower(Drive_power * -1);
      sleep(1000);
      frontLeftMotor.setPower(0);
      backLeftMotor.setPower(0);
      frontRghtMotor.setPower(0);
      backRightMotor.setPower(0);
      sleep(5000);
    }
  }
}
