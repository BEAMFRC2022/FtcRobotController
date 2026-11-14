package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "BioBuzz Teleop Drive Only", group = "TeleOp")
public class BioBuzzTeleopDriveOnly extends LinearOpMode {

    public static final String VERSION = "1.01";

    double speed_fast = 0.8;
    double speed_slow = 0.5;
    
    double SAFE_DRIVE_SPEED   = speed_fast;
    double SAFE_STRAFE_SPEED  = speed_fast;
    double SAFE_YAW_SPEED     = speed_fast;
    double HEADING_HOLD_TIME  = 10.0;

    ElapsedTime stopTime = new ElapsedTime();
    boolean autoHeading  = false;

    // Instance of SimplifiedOdometryRobot helper
    SimplifiedOdometryRobot robot = new SimplifiedOdometryRobot(this);

    @Override
    public void runOpMode() {
        // Initialize drive hardware & telemetry
        robot.initialize(true);

        // Wait for driver to press start
        while (opModeInInit()) {
            telemetry.addData("Version", VERSION);
            telemetry.addData(">", "Touch Play to drive BioBuzz (Drive Only)");
            telemetry.addData("Drive Motors", robot.frontleftmotor != null ? "Connected" : "NOT FOUND (check config)");

            robot.readSensors();
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            robot.readSensors();

            // Reset gyro heading on gamepad1 options + share
            if (gamepad1.options && gamepad1.share) {
                robot.resetOdometry();
            }

            // DRIVING SECTION ----------------------------------------------------------------------
            // Inverted drive and strafe signs to match physical motor geometry while keeping turning correct
            double drive  =  gamepad1.left_stick_y * SAFE_DRIVE_SPEED;
            double strafe =  gamepad1.left_stick_x * SAFE_STRAFE_SPEED;
            double yaw    = -gamepad1.right_stick_x * SAFE_YAW_SPEED;

            // Orthogonal DPAD movements
            if (gamepad1.dpad_left) {
                strafe = -SAFE_DRIVE_SPEED / 2.0;
            } else if (gamepad1.dpad_right) {
                strafe = SAFE_DRIVE_SPEED / 2.0;
            } else if (gamepad1.dpad_up) {
                drive = -SAFE_DRIVE_SPEED / 2.0;
            } else if (gamepad1.dpad_down) {
                drive = SAFE_STRAFE_SPEED / 2.0;
            }

            // Heading Hold Logic
            if (Math.abs(yaw) > 0.05) {
                autoHeading = false;
            } else {
                if (!autoHeading && Math.abs(robot.getTurnRate()) < 2.0) {
                    robot.yawController.reset(robot.getHeading());
                    autoHeading = true;
                }
            }

            if (autoHeading) {
                yaw = robot.yawController.getOutput(robot.getHeading());
            }
            
            // Speed toggles
            if (gamepad1.y) {
                SAFE_DRIVE_SPEED = speed_slow;
                SAFE_STRAFE_SPEED = speed_slow;
                SAFE_YAW_SPEED = speed_slow;
            } else if (gamepad1.a) {
                SAFE_DRIVE_SPEED = speed_fast;
                SAFE_STRAFE_SPEED = speed_fast;
                SAFE_YAW_SPEED = speed_fast;
            }

            // Drive the Mecanum wheels
            robot.moveRobot(drive, strafe, yaw);

            // Gyro drift reset timer
            if ((drive == 0) && (strafe == 0) && (yaw == 0)) {
                if (stopTime.time() > HEADING_HOLD_TIME) {
                    robot.yawController.reset(robot.getHeading());
                }
            } else {
                stopTime.reset();
            }

            // Drive Telemetry
            telemetry.addData("Version", VERSION);
            telemetry.addData("Drive Motors (FL, BL, FR, BR)", "%.2f, %.2f, %.2f, %.2f", 
                    robot.frontLeftPower, robot.backLeftPower, robot.frontRightPower, robot.backRightPower);
            telemetry.addData("Drive Sticks (Y, X, Yaw)", "%.2f, %.2f, %.2f", drive, strafe, yaw);
            telemetry.update();
        }
    }
}
