package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name = "Nav With Limelight", group = "TeleOp")
public class navwithlimelight extends LinearOpMode {

    double speed_fast = 0.8;
    double speed_slow = 0.5;
    double intake_power = 1.0;
    
    double SAFE_DRIVE_SPEED   = speed_fast;
    double SAFE_STRAFE_SPEED  = speed_fast;
    double SAFE_YAW_SPEED     = speed_fast;
    double HEADING_HOLD_TIME  = 10.0;

    ElapsedTime stopTime = new ElapsedTime();
    boolean autoHeading = false;

    SimplifiedOdometryRobot robot = new SimplifiedOdometryRobot(this);

    // Shooter hardware and variables
    private CRServo shooterright;
    private CRServo shooterleft;
    private double shooterPower = 0.5;
    private boolean wasDpadUp = false;
    private boolean wasDpadDown = false;

    // Limelight hardware
    private Limelight3A limelight;

    @Override
    public void runOpMode() {
        // Initialize robot hardware
        robot.initialize(true);

        // Initialize shooter servos
        try {
            shooterright = hardwareMap.get(CRServo.class, "shooterrightAsCRServo");
            shooterleft = hardwareMap.get(CRServo.class, "shooterleftAsCRServo");
        } catch (Exception e) {
            shooterright = hardwareMap.get(CRServo.class, "shooterright");
            shooterleft = hardwareMap.get(CRServo.class, "shooterleft");
        }
        shooterright.setDirection(CRServo.Direction.REVERSE);

        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        // Wait for driver to press start
        while (opModeInInit()) {
            telemetry.addData(">", "Touch Play to drive with Limelight");
            robot.readSensors();
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            robot.readSensors();

            // Allow driver to reset odometry/gyro
            if (gamepad1.options && gamepad1.share) {
                robot.resetOdometry();
            }

            // DRIVING SECTION ----------------------------------------------------------------------
            double driveVal  = -gamepad1.left_stick_y * SAFE_DRIVE_SPEED;
            double strafeVal = -gamepad1.left_stick_x * SAFE_STRAFE_SPEED;
            double yawVal    = -gamepad1.right_stick_x * SAFE_YAW_SPEED;

            // Dpad manual override
            if (gamepad1.dpad_left) {
                strafeVal = SAFE_DRIVE_SPEED / 2.0;
            } else if (gamepad1.dpad_right) {
                strafeVal = -SAFE_DRIVE_SPEED / 2.0;
            } else if (gamepad1.dpad_up) {
                driveVal = SAFE_DRIVE_SPEED / 2.0;
            } else if (gamepad1.dpad_down) {
                driveVal = -SAFE_STRAFE_SPEED / 2.0;
            }

            // LIMELIGHT AUTO-AIM ASSIST (Hold Gamepad 1 B)
            LLResult result = limelight.getLatestResult();
            if (gamepad1.b && result != null && result.isValid()) {
                double tx = result.getTx(); // Horizontal angle error
                double Kp = 0.03; // Proportional turning gain
                yawVal = tx * Kp;
                autoHeading = false;
                telemetry.addData("Auto-Aim", "Locked! TX: %.2f", tx);
            } else {
                // Heading hold logic
                if (Math.abs(yawVal) > 0.05) {
                    autoHeading = false;
                } else {
                    if (!autoHeading && Math.abs(robot.getTurnRate()) < 2.0) {
                        robot.yawController.reset(robot.getHeading());
                        autoHeading = true;
                    }
                }

                if (autoHeading) {
                    yawVal = robot.yawController.getOutput(robot.getHeading());
                }
            }

            // Speed modifiers (A/Y buttons on Gamepad 1)
            if (gamepad1.y) {
                SAFE_DRIVE_SPEED = speed_slow;
                SAFE_STRAFE_SPEED = speed_slow;
                SAFE_YAW_SPEED = speed_slow;
            } else if (gamepad1.a) {
                SAFE_DRIVE_SPEED = speed_fast;
                SAFE_STRAFE_SPEED = speed_fast;
                SAFE_YAW_SPEED = speed_fast;
            }

            // Drive robot
            robot.moveRobot(driveVal, strafeVal, yawVal);

            // Heading hold timer
            if ((driveVal == 0) && (strafeVal == 0) && (yawVal == 0)) {
                if (stopTime.time() > HEADING_HOLD_TIME) {
                    robot.yawController.reset(robot.getHeading());
                }
            } else {
                stopTime.reset();
            }

            // OPERATOR SECTION --------------------------------------------------------------------
            
            // Intake control (Gamepad 2 Bumpers)
            if (gamepad2.right_bumper) {
                robot.intake.setPower(intake_power);
            } else if (gamepad2.left_bumper) {
                robot.intake.setPower(-intake_power);
            } else {
                robot.intake.setPower(0);
            }

            // Shooter power adjustment (Gamepad 2 D-pad Up/Down)
            if (gamepad2.dpad_up && !wasDpadUp) {
                shooterPower = Math.min(1.0, shooterPower + 0.1);
            } else if (gamepad2.dpad_down && !wasDpadDown) {
                shooterPower = Math.max(0.0, shooterPower - 0.1);
            }
            wasDpadUp = gamepad2.dpad_up;
            wasDpadDown = gamepad2.dpad_down;

            // Shoot (Gamepad 2 Y button)
            if (gamepad2.y) {
                shooterleft.setPower(shooterPower);
                shooterright.setPower(shooterPower);
            } else {
                shooterleft.setPower(0);
                shooterright.setPower(0);
            }

            // Telemetry
            telemetry.addData("Shooter Power", "%.2f", shooterPower);
            if (result != null && result.isValid()) {
                telemetry.addData("Limelight TX", "%.2f", result.getTx());
                telemetry.addData("Limelight TY", "%.2f", result.getTy());
                Pose3D botpose = result.getBotpose();
                if (botpose != null) {
                    telemetry.addData("Botpose X, Y", "%.2f, %.2f", botpose.getPosition().x, botpose.getPosition().y);
                }
            } else {
                telemetry.addData("Limelight", "No Target");
            }
            telemetry.update();
        }

        limelight.stop();
    }
}
