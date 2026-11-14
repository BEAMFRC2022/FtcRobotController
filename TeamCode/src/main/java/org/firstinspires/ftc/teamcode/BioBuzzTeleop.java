/* Created by Phil Malone. 2023.
    This class illustrates my simplified Odometry Strategy.
    It implements basic straight line motions but with heading and drift controls to limit drift.
    See the readme for a link to a video tutorial explaining the operation and limitations of the code.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="BioBuzz Teleop", group = "Mr. Phil")
public class BioBuzzTeleop extends LinearOpMode
{
    double speed_fast = 0.8;
    double speed_slow = 0.5;
    double intake_power = 1.0;
    
    double SAFE_DRIVE_SPEED   = speed_fast;
    double SAFE_STRAFE_SPEED  = speed_fast;
    double SAFE_YAW_SPEED     = speed_fast;
    double HEADING_HOLD_TIME  = 10.0;

    // local parameters
    ElapsedTime stopTime = new ElapsedTime();
    boolean autoHeading  = false;

    // get an instance of the "Robot" class.
    SimplifiedOdometryRobot robot = new SimplifiedOdometryRobot(this);

    // Shooter hardware and variables
    private DcMotor shooterright;
    private DcMotor shooterleft;
    private double shooterPower = 0.5;
    private boolean wasDpadUp = false;
    private boolean wasDpadDown = false;

    @Override public void runOpMode()
    {
        // Initialize the drive hardware & Turn on telemetry
        robot.initialize(true);

        // Safe initialization of shooter motors (prevents NullPointerException crashes)
        try {
            shooterright = hardwareMap.get(DcMotor.class, "shooterright");
        } catch (Exception e) {
            try {
                shooterright = hardwareMap.get(DcMotor.class, "shooterrightAsDcMotor");
            } catch (Exception ex) {
                shooterright = null;
            }
        }

        try {
            shooterleft = hardwareMap.get(DcMotor.class, "shooterleft");
        } catch (Exception e) {
            try {
                shooterleft = hardwareMap.get(DcMotor.class, "shooterleftAsDcMotor");
            } catch (Exception ex) {
                shooterleft = null;
            }
        }

        if (shooterright != null) {
            shooterright.setDirection(DcMotor.Direction.REVERSE);
            shooterright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (shooterleft != null) {
            shooterleft.setDirection(DcMotor.Direction.FORWARD);
            shooterleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Wait for driver to press start
        while(opModeInInit()) {
            telemetry.addData(">", "Touch Play to drive");
            telemetry.addData("Drive Motors", robot.frontleftmotor != null ? "Connected" : "NOT FOUND (check config)");
            telemetry.addData("Shooter Right Motor", shooterright != null ? "Connected" : "NOT FOUND (check config)");
            telemetry.addData("Shooter Left Motor", shooterleft != null ? "Connected" : "NOT FOUND (check config)");
            telemetry.addData("Intake Motor", robot.intake != null ? "Connected" : "NOT FOUND (check config)");

            // Read and display sensor data
            robot.readSensors();
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive())
        {
            robot.readSensors();

            // Allow the driver to reset the gyro by pressing both small gamepad buttons
            if(gamepad1.options && gamepad1.share){
                robot.resetOdometry();
            }

            // DRIVING SECTION ----------------------------------------------------------------------
            double drive  = -gamepad1.left_stick_y * SAFE_DRIVE_SPEED;
            double strafe = -gamepad1.left_stick_x * SAFE_STRAFE_SPEED;
            double yaw    = -gamepad1.right_stick_x * SAFE_YAW_SPEED;

            // Orthogonal DPAD movements
            if (gamepad1.dpad_left) {
                strafe = SAFE_DRIVE_SPEED / 2.0;
            } else if (gamepad1.dpad_right) {
                strafe = -SAFE_DRIVE_SPEED / 2.0;
            } else if (gamepad1.dpad_up) {
                drive = SAFE_DRIVE_SPEED / 2.0;
            } else if (gamepad1.dpad_down) {
                drive = -SAFE_STRAFE_SPEED / 2.0;
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
            if(gamepad1.y) {
                SAFE_DRIVE_SPEED = speed_slow;
                SAFE_STRAFE_SPEED = speed_slow;
                SAFE_YAW_SPEED = speed_slow;
            } else if(gamepad1.a) {
                SAFE_DRIVE_SPEED = speed_fast;
                SAFE_STRAFE_SPEED = speed_fast;
                SAFE_YAW_SPEED = speed_fast;
            }

            // Drive the wheels
            robot.moveRobot(drive, strafe, yaw);

            // Gyro drift reset timer
            if ((drive == 0) && (strafe == 0) && (yaw == 0)) {
                if (stopTime.time() > HEADING_HOLD_TIME) {
                    robot.yawController.reset(robot.getHeading());
                }
            } else {
                stopTime.reset();
            }
            
            // OPERATOR SECTION --------------------------------------------------------------------
            
            // Safe Intake control
            if (robot.intake != null) {
                if (gamepad2.right_bumper) {
                    robot.intake.setPower(intake_power);
                } else if (gamepad2.left_bumper) {
                    robot.intake.setPower(-intake_power);
                } else {
                    robot.intake.setPower(0);
                }
            }
            
            // Safe Shooter control (Gamepad 2 D-pad Up/Down for power, Y for shooting)
            if (gamepad2.dpad_up && !wasDpadUp) {
                shooterPower = Math.min(1.0, shooterPower + 0.1);
            } else if (gamepad2.dpad_down && !wasDpadDown) {
                shooterPower = Math.max(0.0, shooterPower - 0.1);
            }
            wasDpadUp = gamepad2.dpad_up;
            wasDpadDown = gamepad2.dpad_down;

            if (gamepad2.y) {
                if (shooterleft != null) shooterleft.setPower(shooterPower);
                if (shooterright != null) shooterright.setPower(shooterPower);
            } else {
                if (shooterleft != null) shooterleft.setPower(0);
                if (shooterright != null) shooterright.setPower(0);
            }

            telemetry.addData("Shooter Power", "%.2f", shooterPower);
            telemetry.update();
        }
    }
}
