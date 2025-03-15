package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "AUTO_DriveByTime_specimen_only (Blocks to Java)")
public class AUTO_DriveByTime_specimen_only extends LinearOpMode {

  private DcMotor backleftMotor;
  private DcMotor backrightmotor;
  private DcMotor frontleftmotor;
  private DcMotor frontrightmotor;
  private DistanceSensor DistanceSide;
  private DistanceSensor DistanceFront;
  private DcMotor extender;
  private DcMotor Shoulder;

  ElapsedTime runtime;

  /**
   * Function to perform a relative move, based on encoder counts.
   * Encoders are not reset as the move is based on the current position.
   * Move will stop if any of three conditions occur:
   * 1) Move gets to the desired position
   * 2) Move runs out of time
   * 3) Driver stops the OpMode running.
   */
  private void drive(int DriveTime, double DrivePwr, int Wait) {
    if (opModeIsActive()) {
      runtime.reset();
      while (opModeIsActive() && runtime.milliseconds() < DriveTime) {
        backleftMotor.setPower(DrivePwr);
        backrightmotor.setPower(DrivePwr);
        frontleftmotor.setPower(DrivePwr);
        frontrightmotor.setPower(DrivePwr);
        // Stop all motion.
      }
      backleftMotor.setPower(0);
      backrightmotor.setPower(0);
      frontleftmotor.setPower(0);
      frontrightmotor.setPower(0);
      // Optional pause after each move.
      sleep(Wait);
    }
  }

  /**
   * Describe this function...
   */
  private void strafe(double DrivePwr, int DriveTime, int Wait) {
    if (opModeIsActive()) {
      runtime.reset();
      while (opModeIsActive() && runtime.milliseconds() < DriveTime) {
        backleftMotor.setPower(-1 * DrivePwr);
        backrightmotor.setPower(DrivePwr);
        frontleftmotor.setPower(DrivePwr);
        frontrightmotor.setPower(-1 * DrivePwr);
        // Stop all motion.
      }
      backleftMotor.setPower(0);
      backrightmotor.setPower(0);
      frontleftmotor.setPower(0);
      frontrightmotor.setPower(0);
      // Optional pause after each move.
      sleep(Wait);
    }
  }

  /**
   * Describe this function...
   */
  private void strafing_distance(
      // TODO: Enter the type for argument named final_dist
      UNKNOWN_TYPE final_dist,
      double power, long Wait) {
    if (opModeIsActive()) {
      runtime.reset();
      while (opModeIsActive() && DistanceSide.getDistance(DistanceUnit.INCH) < final_dist) {
        backleftMotor.setPower(-1 * power);
        backrightmotor.setPower(power);
        frontleftmotor.setPower(power);
        frontrightmotor.setPower(-1 * power);
        // Stop all motion.
        telemetry.addData("distance from wall", DistanceSide.getDistance(DistanceUnit.INCH));
        telemetry.update();
      }
      backleftMotor.setPower(0);
      backrightmotor.setPower(0);
      frontleftmotor.setPower(0);
      frontrightmotor.setPower(0);
      // Optional pause after each move.
      sleep(Wait);
    }
  }

  /**
   * Describe this function...
   */
  private void forward_distance2(
      // TODO: Enter the type for argument named final_dist
      UNKNOWN_TYPE final_dist,
      double power, long Wait) {
    if (opModeIsActive()) {
      runtime.reset();
      while (opModeIsActive() && DistanceFront.getDistance(DistanceUnit.INCH) < final_dist) {
        backleftMotor.setPower(power);
        backrightmotor.setPower(power);
        frontleftmotor.setPower(power);
        frontrightmotor.setPower(power);
        // Stop all motion.
        telemetry.addData("distance from wall", DistanceFront.getDistance(DistanceUnit.INCH));
        telemetry.update();
      }
      backleftMotor.setPower(0);
      backrightmotor.setPower(0);
      frontleftmotor.setPower(0);
      frontrightmotor.setPower(0);
      // Optional pause after each move.
      sleep(Wait);
    }
  }

  /**
   * This OpMode illustrates the concept of driving a path based on time.
   *
   * This OpMode assumes that you do not have encoders on the
   * wheels, otherwise you would use RobotAutoDriveByEncoder.
   *
   * The desired path in this example is:
   * - Drive forward for 3 seconds
   * - Spin right for 1.3 seconds
   * - Drive backward for 1 second
   */
  @Override
  public void runOpMode() {
    double FORWARD_SPEED;
    double TURN_SPEED;
    int Target_Shoulder;
    int Target_Extender;

    backleftMotor = hardwareMap.get(DcMotor.class, "backleftMotor");
    backrightmotor = hardwareMap.get(DcMotor.class, "backrightmotor");
    frontleftmotor = hardwareMap.get(DcMotor.class, "frontleftmotor");
    frontrightmotor = hardwareMap.get(DcMotor.class, "frontrightmotor");
    DistanceSide = hardwareMap.get(DistanceSensor.class, "DistanceSide");
    DistanceFront = hardwareMap.get(DistanceSensor.class, "DistanceFront");
    extender = hardwareMap.get(DcMotor.class, "extender");
    Shoulder = hardwareMap.get(DcMotor.class, "Shoulder");

    // Initialize variables.
    runtime = new ElapsedTime();
    FORWARD_SPEED = 0.6;
    TURN_SPEED = 0.5;
    // To drive forward, most robots need the motor on one side to be reversed, because the
    // axles point in opposite directions. When run, this OpMode should start both motors driving
    // forward. Adjust these motor directions based on your first test drive.
    // Note: The settings here assume direct drive on left and right wheels.
    // Gear reduction or 90 degree drives may require direction flips.
    backleftMotor.setDirection(DcMotor.Direction.REVERSE);
    frontleftmotor.setDirection(DcMotor.Direction.REVERSE);
    backrightmotor.setDirection(DcMotor.Direction.FORWARD);
    frontrightmotor.setDirection(DcMotor.Direction.FORWARD);
    backleftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    backrightmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    frontleftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    frontrightmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    // Make all motors run in active brake mode
    backrightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    frontrightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    frontleftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backleftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    Shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    Shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Shoulder.setTargetPosition(0);
    extender.setTargetPosition(0);
    Shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    ((DcMotorEx) Shoulder).setVelocity(500);
    ((DcMotorEx) extender).setVelocity(1000);
    // Send telemetry message to signify robot waiting.
    telemetry.addData("Status", "Ready to run");
    telemetry.update();
    // Wait for the game to start (driver presses START).
    waitForStart();
    if (opModeIsActive()) {
      drive(250, 0.6, 250);
      // Step 1: Raise Shoulder to 290
      Target_Shoulder = 300;
      Shoulder.setTargetPosition(Target_Shoulder);
      sleep(2000);
      // Step 2: Drive Forward 550
      drive(1500, 0.4, 250);
      // Step 3: Extend Arm
      Target_Extender = -1050;
      extender.setTargetPosition(Target_Extender);
      sleep(2000);
      // Step 4: Raise Shoulder
      Target_Shoulder = 360;
      Shoulder.setTargetPosition(Target_Shoulder);
      sleep(1000);
      // Step 5: Retract Arm to Remove Specimen
      Target_Extender = 50;
      extender.setTargetPosition(Target_Extender);
      sleep(2000);
      // Step 6: Back Up
      drive(1000, -0.6, 250);
      // Step 7: Lower Shoulder to 0 Position
      Target_Shoulder = -30;
      Shoulder.setTargetPosition(Target_Shoulder);
      sleep(2000);
      // Step 8: Strafe right and back to park (Observation Zone)
      strafe(0.6, 2700, 250);
      drive(250, -0.6, 0);
    }
  }

  /**
   * Describe this function...
   */
  private void pivot(
      // TODO: Enter the type for argument named DriveTime
      UNKNOWN_TYPE DriveTime,
      double DrivePwr, long Wait) {
    if (opModeIsActive()) {
      runtime.reset();
      while (opModeIsActive() && runtime.milliseconds() < DriveTime) {
        frontleftmotor.setPower(DrivePwr);
        frontrightmotor.setPower(DrivePwr * -1);
        backleftMotor.setPower(DrivePwr);
        backrightmotor.setPower(DrivePwr * -1);
      }
      // Stop all motion.
      backleftMotor.setPower(0);
      backrightmotor.setPower(0);
      frontleftmotor.setPower(0);
      frontrightmotor.setPower(0);
      // Optional pause after each move.
      sleep(Wait);
    }
  }

  /**
   * Describe this function...
   */
  private void pivotCounterCW(
      // TODO: Enter the type for argument named DriveTime
      UNKNOWN_TYPE DriveTime,
      double DrivePwr, long Wait) {
    if (opModeIsActive()) {
      runtime.reset();
      while (opModeIsActive() && runtime.milliseconds() < DriveTime) {
        frontleftmotor.setPower(DrivePwr * -1);
        frontrightmotor.setPower(DrivePwr);
        backleftMotor.setPower(DrivePwr * -1);
        backrightmotor.setPower(DrivePwr);
      }
      // Stop all motion.
      backleftMotor.setPower(0);
      backrightmotor.setPower(0);
      frontleftmotor.setPower(0);
      frontrightmotor.setPower(0);
      // Optional pause after each move.
      sleep(Wait);
    }
  }
}
