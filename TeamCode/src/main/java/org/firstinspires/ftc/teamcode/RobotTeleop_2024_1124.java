package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "RobotTeleop_2024_1124 (Blocks to Java)")
public class RobotTeleop_2024_1124 extends LinearOpMode {

  private DcMotor extender;
  private DcMotor Shoulder;
  private DcMotor backrightmotor;
  private DcMotor frontrightmotor;
  private DcMotor frontleftmotor;
  private DcMotor backleftMotor;
  private IMU imu;
  private CRServo Intake;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    boolean ScoreHigh_Y;
    int Extender_Enable;
    boolean ButtonB;
    boolean ButtonA;
    boolean TravelMode_X;
    double Drive_power;
    int Manual_Increment;
    int Manual_Increment_ext;
    ImuOrientationOnRobot orientationOnRobot;
    double pivotfactor;
    double pivotthreshold;
    int Shoulder_threshhold;
    YawPitchRollAngles robotOrientation;
    int Target_Shoulder;
    double Shoulder_deg;
    double extender_max_dis;
    double extender_limit;
    float y;
    float x;
    double rx;
    double denominator;

    extender = hardwareMap.get(DcMotor.class, "extender");
    Shoulder = hardwareMap.get(DcMotor.class, "Shoulder");
    backrightmotor = hardwareMap.get(DcMotor.class, "backrightmotor");
    frontrightmotor = hardwareMap.get(DcMotor.class, "frontrightmotor");
    frontleftmotor = hardwareMap.get(DcMotor.class, "frontleftmotor");
    backleftMotor = hardwareMap.get(DcMotor.class, "backleftMotor");
    imu = hardwareMap.get(IMU.class, "imu");
    Intake = hardwareMap.get(CRServo.class, "Intake");

    ScoreHigh_Y = false;
    ButtonB = false;
    ButtonA = false;
    TravelMode_X = false;
    Drive_power = 1;
    // Increment for position target when using manual arm mode
    Manual_Increment = 10;
    Manual_Increment_ext = 5;
    // Set arm motors to run to position
    extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Shoulder.setTargetPosition(0);
    Shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    ((DcMotorEx) Shoulder).setVelocity(300);
    // Set extender motors to run with a power command
    // Run with Encoder mode results in very limited  max motor speed - use run with out encoder
    extender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    // Reverse the right side motors
    backrightmotor.setDirection(DcMotor.Direction.REVERSE);
    frontrightmotor.setDirection(DcMotor.Direction.REVERSE);
    backrightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    frontrightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    frontleftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backleftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    // Configuring the Gyro
    // Edit this to match your mounting configuration.
    orientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);
    // Now initialize the IMU with this mounting orientation.
    imu.initialize(new IMU.Parameters(orientationOnRobot));
    // this should reduce the speed of the pivot control when the joystick is slightly moved
    // pivotfactor= reduction in power, pivot threshold = when to enable this slow function
    pivotfactor = 0.4;
    // pivot threshold = when to enable this slow function
    pivotthreshold = 0.8;
    // Limit how far extender can go before we need to disable it to stay under 42in
    Extender_Enable = 1;
    // 375 counts on the shoulder is close to the 45deg, assuming its a fully down during Init
    Shoulder_threshhold = 375;
    Target_Shoulder = 0;
    waitForStart();
    if (opModeIsActive()) {
      while (opModeIsActive()) {
        if (gamepad1.x) {
          extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
          extender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
          sleep(100);
          Target_Shoulder = 0;
        }
        // Outputting Gyro values
        robotOrientation = imu.getRobotYawPitchRollAngles();
        if (gamepad1.x) {
          imu.resetYaw();
        }
        // Calculating shoulder angle-degrees
        Shoulder_deg = 0.152 * Shoulder.getCurrentPosition() - 6.23;
        // Calculating maximum extender distance-inches
        extender_max_dis = 38 / Math.cos(Shoulder_deg / 180 * Math.PI) - 18.5;
        extender_limit = -140 * extender_max_dis - 35.2;
        // To allow the extender to not be limited when the Arm is at lifted ~45 deg or more

        y = gamepad1.left_stick_y;
        // Remember, this is reversed!
        x = gamepad1.left_stick_x * -1;
        // This will implement the pivot slowing feature that helps to better control robot at slow pivots commands
        if (Math.abs(gamepad1.right_stick_x) > pivotthreshold) {
          rx = gamepad1.right_stick_x * -1;
        } else {
          rx = gamepad1.right_stick_x * pivotfactor * -1;
        }
        // Denominator is the largest motor power
        // (absolute value) or 1.
        // This ensures all the powers maintain
        // the same ratio, but only when at least one is
        // out of the range [-1, 1].
        denominator = JavaUtil.maxOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(y), Math.abs(x), Math.abs(rx))), 1));
        // This function slows down the drive speed
        // Gamepad 1 A = Fast Y = Slow
        if (gamepad1.a) {
          Drive_power = 1;
        } else if (gamepad1.y) {
          // 0.5 seems like a good level
          Drive_power = 0.5;
        } else {
        }
        // Reads the input for arm position
        // Make sure your ID's match your configuration
        frontleftmotor.setPower(((y + x + rx) / denominator) * 1 * Drive_power);
        backleftMotor.setPower((((y - x) + rx) / denominator) * Drive_power);
        frontrightmotor.setPower((((y - x) - rx) / denominator) * Drive_power);
        backrightmotor.setPower((((y + x) - rx) / denominator) * 1 * Drive_power);
        if (gamepad2.a) {
          ButtonA = true;
          ButtonB = false;
          TravelMode_X = false;
          ScoreHigh_Y = false;
        } else if (gamepad2.b) {
          ButtonB = true;
          ButtonA = false;
          TravelMode_X = false;
          ScoreHigh_Y = false;
        } else if (gamepad2.y) {
          ScoreHigh_Y = true;
          ButtonA = false;
          ButtonB = false;
          TravelMode_X = false;
        } else if (gamepad2.x) {
          TravelMode_X = true;
          ButtonA = false;
          ButtonB = false;
          ScoreHigh_Y = false;
        } else {
        }
        if (ButtonA) {
          // Puts arm into pickup postion
          Target_Shoulder = 0;
          Shoulder.setTargetPosition(Target_Shoulder);
        }
        if (ButtonB) {
          // Low Scoring Position 1
          Target_Shoulder = 470;
          Shoulder.setTargetPosition(Target_Shoulder);
        }
        if (ScoreHigh_Y) {
          // High scoring position 2
          Target_Shoulder = 530;
          Shoulder.setTargetPosition(Target_Shoulder);
        }
        if (TravelMode_X) {
          // Arm in traveling mode
          Target_Shoulder = 60;
          Shoulder.setTargetPosition(Target_Shoulder);
        }
        // Disable extender if shoulder is < 45 deg, and extende < extender limit (needs to be < as the feedback is negative)
        if (Shoulder.getCurrentPosition() > Shoulder_threshhold || extender.getCurrentPosition() > extender_limit) {
          Extender_Enable = 1;
        } else if (Shoulder.getCurrentPosition() <= Shoulder_threshhold && extender.getCurrentPosition() < extender_limit) {
          Extender_Enable = 0;
        } else {
        }
        // Intake control - Dpad down pushes sample out
        // Intake control - Dpad up pulls sample in + extends arm slowly
        if (gamepad2.dpad_down) {
          Intake.setPower(0.8);
        } else if (gamepad2.dpad_up) {
          // Allow extender to retarct, but not extend if Extender Enable = 0
          extender.setPower(-0.3 * Extender_Enable);
          Intake.setPower(-1);
        } else {
          // Always allow extender to retract, but not extend if Extender Enable = 0
          if (gamepad2.left_stick_y >= 0) {
            extender.setPower(gamepad2.left_stick_y);
          } else {
            extender.setPower(gamepad2.left_stick_y * Extender_Enable);
          }
          Intake.setPower(0);
        }
        //
        // Shoulder manual control
        if (gamepad2.right_trigger == 1) {
          // Right trigger to manually lower shoulder
          Target_Shoulder += Manual_Increment * -1;
          Shoulder.setTargetPosition(Target_Shoulder);
          ButtonA = false;
          ButtonB = false;
          TravelMode_X = false;
          ScoreHigh_Y = false;
        } else if (gamepad2.right_bumper) {
          // Right bumper to manually raise shoulder
          Target_Shoulder += Manual_Increment;
          Shoulder.setTargetPosition(Target_Shoulder);
          ButtonA = false;
          ButtonB = false;
          TravelMode_X = false;
          ScoreHigh_Y = false;
        } else {
        }
        //
        telemetry.addData("Shoulder degree", Shoulder_deg);
        telemetry.addData("Extender distance", extender_max_dis);
        telemetry.addData("Extender limit", extender_limit);
        telemetry.addData("Shoulder Position SP", Target_Shoulder);
        telemetry.addData("Shoulder Position", Shoulder.getCurrentPosition());
        telemetry.addData("Extender position", extender.getCurrentPosition());
        telemetry.addData("Extender Enable", Extender_Enable);
        telemetry.addData("Extender Power", gamepad2.left_stick_y * Extender_Enable);
      }
    }
    extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    extender.setTargetPosition(0);
    extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
  }
}
