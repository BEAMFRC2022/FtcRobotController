/* Created by Phil Malone. 2023.
    This class illustrates my simplified Odometry Strategy.
    It implements basic straight line motions but with heading and drift controls to limit drift.
    See the readme for a link to a video tutorial explaining the operation and limitations of the code.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode illustrates a teleop OpMode for an Omni robot.
 * An external "Robot" class is used to manage all motor/sensor interfaces, and to assist driving functions.
 * The IMU gyro is used to stabilize the heading when the operator is not requesting a turn.
 */

@TeleOp(name="BioBuzz Teleop", group = "Mr. Phil")
public class BioBuzzTeleop extends LinearOpMode
{
    double speed_fast = 0.8;
    double speed_slow = 0.5;
    double intake_power = 1;
    
    double SAFE_DRIVE_SPEED   = speed_fast ; // Adjust this to your robot and your driver.  Slower usually means more accuracy.  Max value = 1.0
    double SAFE_STRAFE_SPEED  = speed_fast ; // Adjust this to your robot and your driver.  Slower usually means more accuracy.  Max value = 1.0
    double SAFE_YAW_SPEED     = speed_fast ; // Adjust this to your robot and your driver.  Slower usually means more accuracy.  Max value = 1.0  YAW means TURNING
    double HEADING_HOLD_TIME  = 10.0 ; // How long to hold heading once all driver input stops. (This Avoids effects of Gyro Drift)

    // local parameters
    ElapsedTime stopTime   = new ElapsedTime();  // Use for timeouts.
    boolean autoHeading    = false; // used to indicate when heading should be locked.

    // get an instance of the "Robot" class.
    SimplifiedOdometryRobot robot = new SimplifiedOdometryRobot(this);

    // Shooter hardware and variables
    private CRServo shooterright;
    private CRServo shooterleft;
    private double shooterPower = 0.5;
    private boolean wasDpadUp = false;
    private boolean wasDpadDown = false;

    @Override public void runOpMode()
    {
        // Initialize the drive hardware & Turn on telemetry
        robot.initialize(true);

        // Initialize shooter servos (supporting both naming conventions)
        try {
            shooterright = hardwareMap.get(CRServo.class, "shooterrightAsCRServo");
            shooterleft = hardwareMap.get(CRServo.class, "shooterleftAsCRServo");
        } catch (Exception e) {
            shooterright = hardwareMap.get(CRServo.class, "shooterright");
            shooterleft = hardwareMap.get(CRServo.class, "shooterleft");
        }
        shooterright.setDirection(CRServo.Direction.REVERSE);

        // Wait for driver to press start
        while(opModeInInit()) {
            telemetry.addData(">", "Touch Play to drive");

            // Read and display sensor data
            robot.readSensors();
            telemetry.update();
        };

        while (opModeIsActive())
        {
            robot.readSensors();

            // Allow the driver to reset the gyro by pressing both small gamepad buttons
            if(gamepad1.options && gamepad1.share){
                robot.resetOdometry();
            }

            // DRIVING SECTION ----------------------------------------------------------------------
            
            // read joystick values and scale according to limits set at top of this file
            double drive  = -gamepad1.left_stick_y * SAFE_DRIVE_SPEED;      //  Fwd/back on left stick
            double strafe = -gamepad1.left_stick_x * SAFE_STRAFE_SPEED;     //  Left/Right on left stick
            double yaw    = -gamepad1.right_stick_x * SAFE_YAW_SPEED;       //  Rotate on right stick

            //  OR... For special conditions, Use the DPAD to make pure orthoginal motions
            if (gamepad1.dpad_left) {
                strafe = SAFE_DRIVE_SPEED / 2.0;
            } else if (gamepad1.dpad_right) {
                strafe = -SAFE_DRIVE_SPEED / 2.0;
            } else if (gamepad1.dpad_up) {
                drive = SAFE_DRIVE_SPEED / 2.0;
            } else if (gamepad1.dpad_down) {
                drive = -SAFE_STRAFE_SPEED / 2.0;
            }

            // This is where we heep the robot heading locked so it doesn't turn while driving or strafing in a straight line.
            // Is the driver turning the robot, or should it hold its heading?
            if (Math.abs(yaw) > 0.05) {
                // driver is commanding robot to turn, so turn off auto heading.
                autoHeading = false;
            } else {
                // If we are not already locked, wait for robot to stop rotating (<2 deg per second) and then lock-in the current heading.
                if (!autoHeading && Math.abs(robot.getTurnRate()) < 2.0) {
                    robot.yawController.reset(robot.getHeading());  // Lock in the current heading
                    autoHeading = true;
                }
            }

            // If auto heading is on, override manual yaw with the value generated by the heading controller.
            if (autoHeading) {
                yaw = robot.yawController.getOutput(robot.getHeading());
            }
            
            // If A button is pressed, we want to reduce the speed.  If A button is pressed again, we want to go back to full speed
            if(gamepad1.y) {
                SAFE_DRIVE_SPEED = speed_slow;
                SAFE_STRAFE_SPEED = speed_slow;
                SAFE_YAW_SPEED = speed_slow;
            }
            else if(gamepad1.a) {
                SAFE_DRIVE_SPEED = speed_fast;
                SAFE_STRAFE_SPEED = speed_fast;
                SAFE_YAW_SPEED = speed_fast;
            }

            //  Drive the wheels based on the desired axis motions
            robot.moveRobot(drive, strafe, yaw);

            // If the robot has just been sitting here for a while, make heading setpoint track any gyro drift to prevent rotating.
            if ((drive == 0) && (strafe == 0) && (yaw == 0)) {
                if (stopTime.time() > HEADING_HOLD_TIME) {
                    robot.yawController.reset(robot.getHeading());  // just keep tracking the current heading
                }
            } else {
                stopTime.reset();
            }
            
            // OPERATOR SECTION --------------------------------------------------------------------
            
            // Run Intake (Gamepad 2 Bumpers)
            if(gamepad2.right_bumper){
                robot.intake.setPower(intake_power);
            }
            else if(gamepad2.left_bumper){
               robot.intake.setPower(-intake_power); 
            }
            else {
                robot.intake.setPower(0);
            }
            
            // Run Shooter (Gamepad 2 D-pad Up/Down for power, Y for shooting)
            if (gamepad2.dpad_up && !wasDpadUp) {
                shooterPower = Math.min(1.0, shooterPower + 0.1);
            } else if (gamepad2.dpad_down && !wasDpadDown) {
                shooterPower = Math.max(0.0, shooterPower - 0.1);
            }
            wasDpadUp = gamepad2.dpad_up;
            wasDpadDown = gamepad2.dpad_down;

            if (gamepad2.y) {
                shooterleft.setPower(shooterPower);
                shooterright.setPower(shooterPower);
            } else {
                shooterleft.setPower(0);
                shooterright.setPower(0);
            }

            telemetry.addData("Shooter Power", "%.2f", shooterPower);
            telemetry.update();
        }
    }
}
