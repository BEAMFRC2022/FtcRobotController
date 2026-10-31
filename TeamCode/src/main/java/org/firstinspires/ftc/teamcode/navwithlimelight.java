package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name = "Nav With Limelight", group = "TeleOp")
public class navwithlimelight extends LinearOpMode {

    // Motor instances for 4 HD Hex motors (Mecanum drive)
    private DcMotor frontleftmotor;
    private DcMotor frontrightmotor;
    private DcMotor backleftmotor;
    private DcMotor backrightmotor;

    // Odometry / Robot helper
    private final SimplifiedOdometryRobot robot = new SimplifiedOdometryRobot(this);

    // Limelight camera instance
    private Limelight3A limelight;

    @Override
    public void runOpMode() {
        // Initialize motor hardware using the exact names provided:
        // Slot 0 = frontrightmotor
        // Slot 1 = backrightmotor
        // Slot 2 = backleftmotor
        // Slot 3 = frontleftmotor
        frontrightmotor = hardwareMap.get(DcMotor.class, "frontrightmotor");
        backrightmotor  = hardwareMap.get(DcMotor.class, "backrightmotor");
        backleftmotor   = hardwareMap.get(DcMotor.class, "backleftmotor");
        frontleftmotor  = hardwareMap.get(DcMotor.class, "frontleftmotor");

        // Initialize Odometry Robot helper
        robot.initialize(true);

        // Set motor directions for Mecanum drive
        frontleftmotor.setDirection(DcMotor.Direction.FORWARD);
        backleftmotor.setDirection(DcMotor.Direction.FORWARD);
        frontrightmotor.setDirection(DcMotor.Direction.REVERSE);
        backrightmotor.setDirection(DcMotor.Direction.REVERSE);

        // Set zero power behavior to BRAKE for crisp response
        frontleftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontrightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backrightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set telemetry transmission interval for smooth updates
        telemetry.setMsTransmissionInterval(11);

        // Initialization phase loop: Allows retrying until the Limelight finishes booting up
        while (opModeInInit()) {
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
                    telemetry.addData("Limelight", "FPS: %d, CPU: %.1f%%, Temp: %.1f°C", 
                            (int)status.getFps(), status.getCpu(), status.getTemp());
                } else {
                    telemetry.addData("Limelight Status", "Connected & Ready");
                }
            }

            telemetry.addData(">", "Touch Play to drive with Limelight");
            robot.readSensors();
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            robot.readSensors();

            // 1. Driver Joystick Inputs
            double axial   = -gamepad1.left_stick_y;  // Forward / Backward
            double lateral =  gamepad1.left_stick_x;  // Left / Right Strafe
            double yaw     =  gamepad1.right_stick_x; // Rotation

            // 2. Limelight Vision Navigation & Auto-Aim Assist
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
                    // Proportional control to center target horizontally
                    double Kp_turn = 0.03;
                    yaw = tx * Kp_turn;
                    currentMode = "Auto-Aiming (Holding B)";
                } else if (autoApproach) {
                    // Proportional control to drive towards target based on vertical offset (ty)
                    double targetTy = 0.0; 
                    double Kp_drive = 0.02;
                    axial = (ty - targetTy) * Kp_drive;
                    
                    double Kp_turn = 0.03;
                    yaw = tx * Kp_turn;
                    currentMode = "Auto-Approaching (Holding A)";
                }

                // Continuous Vision Telemetry
                telemetry.addData("Vision Target", "VALID (AprilTag Found)");
                telemetry.addData("TX (Heading Error)", "%.2f deg", tx);
                telemetry.addData("TY (Distance Error)", "%.2f deg", ty);
                if (botpose != null) {
                    telemetry.addData("Robot Field Position", "X: %.2f, Y: %.2f", botpose.getPosition().x, botpose.getPosition().y);
                }
            } else {
                if (limelight == null) {
                    telemetry.addData("Vision Status", "ERROR: Limelight not found in Hardware Map");
                } else {
                    telemetry.addData("Vision Status", "Searching for Target (Tags 38-41)...");
                }
            }

            // Continuous Camera Status Telemetry
            if (limelight != null) {
                LLStatus status = limelight.getStatus();
                if (status != null) {
                    telemetry.addData("Camera Status", "FPS: %d | CPU: %.1f%% | Temp: %.1f°C | Pipeline: %d (%s)",
                            (int)status.getFps(), status.getCpu(), status.getTemp(), 
                            status.getPipelineIndex(), status.getPipelineType());
                }
            }

            telemetry.addData("Drive Mode", currentMode);

            // 3. Calculate Mecanum Wheel Powers
            double denominator = Math.max(Math.abs(axial) + Math.abs(lateral) + Math.abs(yaw), 1.0);
            
            double frontLeftPower  = (axial + lateral + yaw) / denominator;
            double backLeftPower   = (axial - lateral + yaw) / denominator;
            double frontRightPower = (axial - lateral - yaw) / denominator;
            double backRightPower  = (axial + lateral - yaw) / denominator;

            // 4. Apply Power to Motors
            frontleftmotor.setPower(frontLeftPower);
            backleftmotor.setPower(backLeftPower);
            frontrightmotor.setPower(frontRightPower);
            backrightmotor.setPower(backRightPower);

            // Motor power telemetry
            telemetry.addData("Motors (FL, BL, FR, BR)", "%.2f, %.2f, %.2f, %.2f", 
                    frontLeftPower, backLeftPower, frontRightPower, backRightPower);
            telemetry.update();
        }

        // Stop Limelight polling when OpMode ends
        if (limelight != null) {
            limelight.stop();
        }
    }
}
