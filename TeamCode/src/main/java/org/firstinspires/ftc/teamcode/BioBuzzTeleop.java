/* Created by Phil Malone. 2023.
    This class illustrates my simplified Odometry Strategy.
    It implements basic straight line motions but with heading and drift controls to limit drift.
    See the readme for a link to a video tutorial explaining the operation and limitations of the code.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name="BioBuzz Teleop", group = "Mr. Phil")
public class BioBuzzTeleop extends LinearOpMode
{
    public static final String VERSION = "2.02";

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

    // Limelight camera hardware
    private Limelight3A limelight;

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

        // Set telemetry transmission interval for smooth updates
        telemetry.setMsTransmissionInterval(11);

        // Wait for driver to press start while retrying Limelight boot-up
        while(opModeInInit()) {
            if (limelight == null) {
                try {
                    limelight = hardwareMap.get(Limelight3A.class, "limelight");
                    limelight.pipelineSwitch(0);
                    limelight.start();
                    telemetry.addData("Limelight Status", "Initialized Successfully!");
                } catch (Exception e) {
                    telemetry.addData("Limelight Status", "Waiting for Limelight (Check DS Config)...");
                }
            } else {
                LLStatus status = limelight.getStatus();
                if (status != null) {
                    telemetry.addData("Limelight Status", "Connected (FPS: %d, Temp: %.1f°C)", (int)status.getFps(), status.getTemp());
                } else {
                    telemetry.addData("Limelight Status", "Connected & Ready");
                }
            }

            telemetry.addData("Version", VERSION);
            telemetry.addData(">", "Touch Play to drive BioBuzz Teleop");
            telemetry.addData("Drive Motors", robot.frontleftmotor != null ? "Connected" : "NOT FOUND (check config)");
            telemetry.addData("Shooter Right Motor", shooterright != null ? "Connected" : "NOT FOUND (check config)");
            telemetry.addData("Shooter Left Motor", shooterleft != null ? "Connected" : "NOT FOUND (check config)");
            telemetry.addData("Intake Motor", robot.intake != null ? "Connected" : "NOT FOUND (check config)");

            // Read and display sensor data
            robot.readSensors();
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            robot.readSensors();

            // Allow the driver to reset the gyro by pressing both small gamepad buttons
            if(gamepad1.options && gamepad1.share){
                robot.resetOdometry();
            }

            // DRIVING SECTION ----------------------------------------------------------------------
            // Corrected strafe sign (-gamepad1.left_stick_x) so pushing left strafes left, pushing right strafes right
            double drive  =  gamepad1.left_stick_y * SAFE_DRIVE_SPEED;
            double strafe = -gamepad1.left_stick_x * SAFE_STRAFE_SPEED;
            double yaw    = -gamepad1.right_stick_x * SAFE_YAW_SPEED;

            // Orthogonal DPAD movements
            if (gamepad1.dpad_left) {
                strafe = SAFE_DRIVE_SPEED / 2.0;
            } else if (gamepad1.dpad_right) {
                strafe = -SAFE_DRIVE_SPEED / 2.0;
            } else if (gamepad1.dpad_up) {
                drive = -SAFE_DRIVE_SPEED / 2.0;
            } else if (gamepad1.dpad_down) {
                drive = SAFE_STRAFE_SPEED / 2.0;
            }

            // LIMELIGHT VISION NAVIGATION & AUTO-AIM
            LLResult result = null;
            boolean autoAiming = gamepad1.b;   // Hold Gamepad 1 B to auto-aim at target (Tags 38-41)
            boolean autoApproach = gamepad1.a; // Hold Gamepad 1 A to auto-approach target

            if (limelight != null) {
                result = limelight.getLatestResult();
            }

            String currentMode = "Manual Driving";

            if (result != null && result.isValid()) {
                double tx = result.getTx(); // Horizontal angle error (-29.8 to 29.8 deg)
                double ty = result.getTy(); // Vertical angle error
                Pose3D botpose = result.getBotpose();

                if (autoAiming) {
                    double Kp_turn = 0.02; // Smooth turning gain
                    yaw = Range.clip(-tx * Kp_turn, -0.4, 0.4); // Capped at 40% power to prevent overshoot
                    autoHeading = false;
                    currentMode = "Auto-Aiming (Holding B)";
                } else if (autoApproach) {
                    double targetTy = 9.6; // Calibrated for 1 foot distance from back wall
                    double Kp_drive = 0.012; // Reduced drive gain for precise alignment without overshooting
                    double Kp_turn  = 0.02;  // Reduced turn gain for precise alignment
                    
                    // Capped at 35% max power while Button A is held to prevent overshooting
                    drive = Range.clip((ty - targetTy) * Kp_drive, -0.35, 0.35);
                    yaw   = Range.clip(-tx * Kp_turn, -0.35, 0.35);
                    
                    autoHeading = false;
                    currentMode = "Auto-Approaching (Holding A - Smooth)";
                }

                telemetry.addData("Vision Target", "VALID (AprilTag Found)");
                telemetry.addData("TX (Heading Error)", "%.2f deg", tx);
                telemetry.addData("TY (Distance Error)", "%.2f deg", ty);
                if (botpose != null) {
                    telemetry.addData("Robot Field Position", "X: %.2f, Y: %.2f", botpose.getPosition().x, botpose.getPosition().y);
                }
            } else {
                if (limelight == null) {
                    telemetry.addData("Vision Status", "Limelight Not Configured");
                } else {
                    telemetry.addData("Vision Status", "Searching for Target (Tags 38-41)...");
                }
            }

            // Heading Hold Logic when not auto-aiming/turning
            if (!autoAiming && !autoApproach) {
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
            }

            // Speed toggles (Y / A buttons when not auto-aiming/approaching)
            if (!autoAiming && !autoApproach) {
                if (gamepad1.y) {
                    SAFE_DRIVE_SPEED = speed_slow;
                    SAFE_STRAFE_SPEED = speed_slow;
                    SAFE_YAW_SPEED = speed_slow;
                }
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

            // Driver Station Telemetry Output
            telemetry.addData("Version", VERSION);
            telemetry.addData("Drive Mode", currentMode);
            telemetry.addData("Shooter Power", "%.2f", shooterPower);
            telemetry.addData("Drive Motors (FL, BL, FR, BR)", "%.2f, %.2f, %.2f, %.2f", 
                    robot.frontLeftPower, robot.backLeftPower, robot.frontRightPower, robot.backRightPower);
            telemetry.addData("Drive Sticks (Y, X, Yaw)", "%.2f, %.2f, %.2f", drive, strafe, yaw);
            telemetry.update();
        }

        // Stop Limelight polling when OpMode ends
        if (limelight != null) {
            limelight.stop();
        }
    }
}
