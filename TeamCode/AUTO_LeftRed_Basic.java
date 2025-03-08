package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "AUTO_LeftRed_Basic (Blocks to Java)")
public class AUTO_LeftRed_Basic extends LinearOpMode {

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

    Shoulder = hardwareMap.get(DcMotor.class, "Shoulder");
    elbow = hardwareMap.get(DcMotor.class, "elbow");
    frontLeftMotor = hardwareMap.get(CRServo.class, "frontLeftMotor");
    frontRghtMotor = hardwareMap.get(DcMotor.class, "frontRghtMotor");
    backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
    backRightMotor = hardwareMap.get(CRServo.class, "backRightMotor");
    Gripper = hardwareMap.get(CRServo.class, "Gripper");

    Target_Shoulder = 0;
    Target_Elbow = 100;
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
    ((DcMotorEx) Shoulder).setVelocity(400);
    Shoulder.setTargetPosition(Target_Shoulder);
    Shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Shoulder.setTargetPosition(Target_Shoulder);
    ((DcMotorEx) elbow).setVelocity(200);
    elbow.setTargetPosition(Target_Elbow);
    elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    elbow.setTargetPosition(Target_Elbow);
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
      sleep(9000);
    }
  }
}
