package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "AUTO_LeftRed_Basic2 (Blocks to Java)")
public class AUTO_LeftRed_Basic2 extends LinearOpMode {

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
    double Drive_power;
    int Target_Elbow;

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
      elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      sleep(500);
      // Move Elbow to travel position
      Target_Shoulder = 0;
      Target_Elbow = 100;
      Shoulder.setTargetPosition(Target_Shoulder);
      elbow.setTargetPosition(Target_Elbow);
      telemetry.addData("Position SP", elbow.getTargetPosition());
      telemetry.addData("Position FB", elbow.getCurrentPosition());
      telemetry.addData("Power FB", elbow.getPower());
      telemetry.update();
      sleep(750);
      // Drive Forward for 500 ms to center spike
      Drive_power = -0.5;
      frontLeftMotor.setPower(Drive_power);
      backLeftMotor.setPower(Drive_power);
      frontRghtMotor.setPower(Drive_power);
      backRightMotor.setPower(Drive_power);
      sleep(700);
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
      sleep(3000);
      // Release one pixel
      Gripper.setPower(1);
      sleep(250);
      // Move Elbow to travel position
      Target_Shoulder = 0;
      Target_Elbow = 100;
      // Rotate left
      Drive_power = -0.5;
      frontLeftMotor.setPower(Drive_power * -1);
      backLeftMotor.setPower(Drive_power * -1);
      frontRghtMotor.setPower(Drive_power);
      backRightMotor.setPower(Drive_power);
      sleep(500);
      // Move forward
      Drive_power = -0.75;
      frontLeftMotor.setPower(Drive_power);
      backLeftMotor.setPower(Drive_power);
      frontRghtMotor.setPower(Drive_power);
      backRightMotor.setPower(Drive_power);
      sleep(500);
      // Rotate right
      Drive_power = -0.75;
      frontLeftMotor.setPower(Drive_power);
      backLeftMotor.setPower(Drive_power);
      frontRghtMotor.setPower(Drive_power * -1);
      backRightMotor.setPower(Drive_power * -1);
      sleep(350);
      // Drive straight to  finish zone
      Drive_power = -0.75;
      frontLeftMotor.setPower(Drive_power);
      backLeftMotor.setPower(Drive_power);
      frontRghtMotor.setPower(Drive_power);
      backRightMotor.setPower(Drive_power);
      sleep(2000);
      frontLeftMotor.setPower(0);
      backLeftMotor.setPower(0);
      frontRghtMotor.setPower(0);
      backRightMotor.setPower(0);
      sleep(50);
    }
  }
}
