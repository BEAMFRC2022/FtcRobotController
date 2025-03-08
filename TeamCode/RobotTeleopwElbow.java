package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "RobotTeleopwElbow (Blocks to Java)")
public class RobotTeleopwElbow extends LinearOpMode {

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
    boolean ButtonY;
    boolean ButtonB;
    boolean ButtonA;
    boolean ClawOpen;
    double Motor_compensation;
    float y;
    float x;
    float rx;
    double denominator;
    boolean ButtonX;

    Shoulder = hardwareMap.get(DcMotor.class, "Shoulder");
    elbow = hardwareMap.get(DcMotor.class, "elbow");
    frontLeftMotor = hardwareMap.get(CRServo.class, "frontLeftMotor");
    frontRghtMotor = hardwareMap.get(DcMotor.class, "frontRghtMotor");
    backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
    backRightMotor = hardwareMap.get(CRServo.class, "backRightMotor");
    Gripper = hardwareMap.get(CRServo.class, "Gripper");

    Shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    ButtonY = false;
    ButtonB = false;
    ButtonA = false;
    ButtonX = false;
    waitForStart();
    if (opModeIsActive()) {
      Shoulder.setTargetPosition(0);
      Shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      ((DcMotorEx) Shoulder).setVelocity(400);
      elbow.setTargetPosition(0);
      elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      ((DcMotorEx) elbow).setVelocity(100);
      // Reverse the right side motors
      // Reverse left motors if you are using NeveRests
      frontLeftMotor.setDirection(CRServo.Direction.REVERSE);
      frontRghtMotor.setDirection(DcMotor.Direction.REVERSE);
      frontRghtMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      // CLAW begins fully open (original = 0.5)
      ClawOpen = true;
      // this should reduce the speed of the chainless motors
      Motor_compensation = 0.7;
      while (opModeIsActive()) {
        // Remember, this is reversed!
        y = gamepad1.left_stick_y;
        x = gamepad1.left_stick_x * -1;
        // Counteract imperfect strafing
        rx = gamepad1.right_stick_x * -1;
        // Denominator is the largest motor power
        // (absolute value) or 1.
        // This ensures all the powers maintain
        // the same ratio, but only when at least one is
        // out of the range [-1, 1].
        denominator = JavaUtil.maxOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(y), Math.abs(x), Math.abs(rx))), 1));
        // Make sure your ID's match your configuration
        frontLeftMotor.setPower(((y + x + rx) / denominator) * Motor_compensation);
        backLeftMotor.setPower((((y - x) + rx) / denominator) * 1);
        frontRghtMotor.setPower((((y - x) - rx) / denominator) * 1);
        backRightMotor.setPower((((y + x) - rx) / denominator) * Motor_compensation);
        if (gamepad2.a) {
          ButtonA = true;
          ButtonB = false;
          ButtonX = false;
          ButtonY = false;
        } else if (gamepad2.b) {
          ButtonB = true;
          ButtonA = false;
          ButtonX = false;
          ButtonY = false;
        } else if (gamepad2.y) {
          ButtonY = true;
          ButtonA = false;
          ButtonB = false;
          ButtonX = false;
        } else if (gamepad2.x) {
          ButtonX = true;
          ButtonA = false;
          ButtonB = false;
          ButtonY = false;
        } else {
        }
        if (ButtonA) {
          Shoulder.setTargetPosition(0);
          elbow.setTargetPosition(0);
          telemetry.addData("velocity", ((DcMotorEx) Shoulder).getVelocity());
          telemetry.addData("position", Shoulder.getCurrentPosition());
          telemetry.addData("target", !Shoulder.isBusy());
          telemetry.update();
          telemetry.addData("velocity", ((DcMotorEx) elbow).getVelocity());
          telemetry.addData("position", elbow.getCurrentPosition());
          telemetry.addData("target", !elbow.isBusy());
          telemetry.update();
        }
        if (ButtonB) {
          Shoulder.setTargetPosition(200);
          telemetry.addData("velocity", ((DcMotorEx) Shoulder).getVelocity());
          telemetry.addData("position", Shoulder.getCurrentPosition());
          telemetry.addData("target", !Shoulder.isBusy());
          elbow.setTargetPosition(100);
          telemetry.addData("velocity", ((DcMotorEx) elbow).getVelocity());
          telemetry.addData("position", elbow.getCurrentPosition());
          telemetry.addData("target", !elbow.isBusy());
        }
        if (ButtonY) {
          Shoulder.setTargetPosition(380);
          telemetry.addData("velocity", ((DcMotorEx) Shoulder).getVelocity());
          telemetry.addData("position", Shoulder.getCurrentPosition());
          telemetry.addData("target", !Shoulder.isBusy());
          elbow.setTargetPosition(150);
          telemetry.addData("velocity", ((DcMotorEx) elbow).getVelocity());
          telemetry.addData("position", elbow.getCurrentPosition());
          telemetry.addData("target", !elbow.isBusy());
        }
        if (ButtonX) {
          Shoulder.setTargetPosition(20);
          telemetry.addData("velocity", ((DcMotorEx) Shoulder).getVelocity());
          telemetry.addData("position", Shoulder.getCurrentPosition());
          telemetry.addData("target", !Shoulder.isBusy());
          elbow.setTargetPosition(20);
          telemetry.addData("velocity", ((DcMotorEx) elbow).getVelocity());
          telemetry.addData("position", elbow.getCurrentPosition());
          telemetry.addData("target", !elbow.isBusy());
        }
        if (gamepad2.dpad_up) {
          Gripper.setPower(1);
        } else if (gamepad2.dpad_down) {
          Gripper.setPower(-1);
        } else {
          Gripper.setPower(0);
        }
        telemetry.update();
        // Put loop blocks here.
        telemetry.update();
      }
    }
  }
}
