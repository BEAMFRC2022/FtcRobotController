package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "AUTO_HighBasket_2X_Gyro_NEW2 (Blocks to Java)")
public class AUTO_HighBasket_2X_Gyro_NEW2 extends LinearOpMode {

  private DcMotor frontleftmotor;
  private DcMotor frontrightmotor;
  private DcMotor backleftMotor;
  private DcMotor backrightmotor;
  private DistanceSensor DistanceFront;
  private DistanceSensor DistanceSide;
  private IMU imu;
  private DcMotor extender;
  private DcMotor Shoulder;
  private CRServo Intake;

  ElapsedTime runtime;
  YawPitchRollAngles robotOrientation;

  /**
   * Describe this function...
   */
  private void pivot(int DriveTime, double DrivePwr, int Wait) {
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
  private void forward_distance(double finaldist, double power, int wai) {
    if (opModeIsActive()) {
      runtime.reset();
      while (opModeIsActive() && DistanceFront.getDistance(DistanceUnit.INCH) > finaldist) {
        backleftMotor.setPower(power);
        backrightmotor.setPower(power);
        frontleftmotor.setPower(power);
        frontrightmotor.setPower(power);
        telemetry.addLine("foward_distance_function");
        telemetry.update();
        // Stop all motion.
      }
      backleftMotor.setPower(0);
      backrightmotor.setPower(0);
      frontleftmotor.setPower(0);
      frontrightmotor.setPower(0);
      for (int count = 0; count < 1; count++) {
        telemetry.addData("distance from wall", DistanceFront.getDistance(DistanceUnit.INCH));
        telemetry.update();
      }
      // Optional pause after each move.
      sleep(wai);
    }
  }

  /**
   * Describe this function...
   */
  private void strafing_distance(double final_dist, double power, int Wait) {
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
   * This function will pivot robot clock wise by the desired degree - Gyro CW rotation is negative angles
   */
  private void PivotClockWiseDeg(double Degree, double Pwr, long Wait) {
    if (opModeIsActive()) {
      // Rest Yaw (Z-axis) to 0 degree
      imu.resetYaw();
      robotOrientation = imu.getRobotYawPitchRollAngles();
      // Yaw will be negative in CW direction - need to multiply FB by -1
      while (opModeIsActive() && robotOrientation.getYaw(AngleUnit.DEGREES) * -1 < Degree) {
        frontleftmotor.setPower(Pwr);
        frontrightmotor.setPower(Pwr * -1);
        backleftMotor.setPower(Pwr);
        backrightmotor.setPower(Pwr * -1);
        robotOrientation = imu.getRobotYawPitchRollAngles();
        // Outputting Gyro values in Run mode
        telemetry.addData("Yaw- Rotation about Z", robotOrientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("Desired Rotation", Degree);
        telemetry.update();
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
    ImuOrientationOnRobot orientationOnRobot;
    int Target_Shoulder;
    int Target_Extender;

    frontleftmotor = hardwareMap.get(DcMotor.class, "frontleftmotor");
    frontrightmotor = hardwareMap.get(DcMotor.class, "frontrightmotor");
    backleftMotor = hardwareMap.get(DcMotor.class, "backleftMotor");
    backrightmotor = hardwareMap.get(DcMotor.class, "backrightmotor");
    DistanceFront = hardwareMap.get(DistanceSensor.class, "DistanceFront");
    DistanceSide = hardwareMap.get(DistanceSensor.class, "DistanceSide");
    imu = hardwareMap.get(IMU.class, "imu");
    extender = hardwareMap.get(DcMotor.class, "extender");
    Shoulder = hardwareMap.get(DcMotor.class, "Shoulder");
    Intake = hardwareMap.get(CRServo.class, "Intake");

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
    Shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Shoulder.setTargetPosition(0);
    extender.setTargetPosition(0);
    Shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    ((DcMotorEx) Shoulder).setVelocity(500);
    ((DcMotorEx) extender).setVelocity(1000);
    // Configuring the Gyro  - Put INitilaize section
    // Edit this to match your mounting configuration.
    orientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);
    // Now initialize the IMU with this mounting orientation.
    imu.initialize(new IMU.Parameters(orientationOnRobot));
    // Rest Yaw (Z-axis) to 0 degree
    imu.resetYaw();
    // Send telemetry message to signify robot waiting.
    telemetry.addData("Status", "Ready to run");
    telemetry.update();
    // Wait for the game to start (driver presses START).
    waitForStart();
    if (opModeIsActive()) {
      // Step through each leg of the path, ensuring that the OpMode has not been stopped along the way.
      Target_Shoulder = 565;
      Shoulder.setTargetPosition(Target_Shoulder);
      Target_Extender = -4000;
      extender.setTargetPosition(Target_Extender);
      sleep(3000);
      // Step 1:  Drive forward 24 inch (26" from wall) using Front  dist sens
      forward_distance(20.4, 0.35, 50);
      // Step 2:  Lifting arm to high scoring position and extend.
      // Outputting data on encoders
      telemetry.addData("Shoulder Position", Shoulder.getCurrentPosition());
      telemetry.addData("Extender Position", extender.getCurrentPosition());
      telemetry.update();
      // Step 3: Push sample out of bucket, trying to set servo to lower pwr to prevent it from shooting sample out
      Intake.setPower(0.75);
      sleep(2500);
      Intake.setPower(0);
      // Step 4:  Drive back a bit and retract extender
      drive(1200, -0.4, 150);
      // Step 4.5:  raise shoulder back to ~90 deg
      Target_Extender = 0;
      extender.setTargetPosition(Target_Extender);
      sleep(1000);
      Target_Shoulder = 200;
      Shoulder.setTargetPosition(Target_Shoulder);
      // Step 5:  Reset arm to starting position
      // Strafing to align with blocks, with a newer battery 2200 and for the old one 2300
      strafing_distance(12, 0.3, 0);
      strafing_distance(40.5, 0.5, 150);
      Intake.setPower(-1);
      extender.setTargetPosition(-2300);
      Shoulder.setTargetPosition(-50);
      sleep(2800);
      Shoulder.setTargetPosition(200);
      extender.setTargetPosition(-1000);
      Intake.setPower(0);
      sleep(800);
      // Moving the robot in front of basket
      strafe(-0.45, 1000, 250);
      // Rotate left to align w basket
      pivot(300, -0.5, 50);
      // Moving shoulder to high scoring position
      Shoulder.setTargetPosition(545);
      sleep(200);
      extender.setTargetPosition(-3700);
      sleep(1000);
      // Drive forward to high basket
      forward_distance(18, 0.35, 50);
      // Extracting sample into low basket
      Intake.setPower(0.75);
      sleep(2100);
      Intake.setPower(0);
      drive(1000, -0.45, 150);
      extender.setTargetPosition(0);
      sleep(2000);
      Shoulder.setTargetPosition(0);
      sleep(5000);
    }
  }

  /**
   * This function will pivot counter clock wise by the desired degree - Gyro CCW rotation is positive angles
   */
  private void PivotCounterClockwiseDeg(double Degree, double Pwr, long Wait) {
    if (opModeIsActive()) {
      // Rest Yaw (Z-axis) to 0 degree
      imu.resetYaw();
      robotOrientation = imu.getRobotYawPitchRollAngles();
      // Yaw will be positive in CW direction
      while (opModeIsActive() && robotOrientation.getYaw(AngleUnit.DEGREES) < Degree) {
        frontleftmotor.setPower(Pwr * -1);
        frontrightmotor.setPower(Pwr);
        backleftMotor.setPower(Pwr * -1);
        backrightmotor.setPower(Pwr);
        robotOrientation = imu.getRobotYawPitchRollAngles();
        // Outputting Gyro values in Run mode
        telemetry.addData("Yaw- Rotation about Z", robotOrientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("Desired Rotation", Degree);
        telemetry.update();
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
