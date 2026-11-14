package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class SimplifiedOdometryRobot {
    private final LinearOpMode myOpMode;
    
    // Subsystem hardware
    public DcMotor intake;
    
    // Drive motors (4 HD Hex motors for Mecanum drive)
    public DcMotor frontleftmotor;
    public DcMotor frontrightmotor;
    public DcMotor backleftmotor;
    public DcMotor backrightmotor;

    // Current motor power levels
    public double frontLeftPower = 0;
    public double backLeftPower = 0;
    public double frontRightPower = 0;
    public double backRightPower = 0;

    public PIDController yawController = new PIDController(0, 0, 0);

    public SimplifiedOdometryRobot(LinearOpMode opMode) {
        this.myOpMode = opMode;
    }

    public void initialize(boolean telemetryOn) {
        // Initialize Intake Motor
        try {
            intake = myOpMode.hardwareMap.get(DcMotor.class, "intakeMotorAsDcMotor");
        } catch (Exception e) {
            try {
                intake = myOpMode.hardwareMap.get(DcMotor.class, "intake");
            } catch (Exception ex) {
                intake = null;
            }
        }

        // Initialize Drive Motors
        try {
            frontrightmotor = myOpMode.hardwareMap.get(DcMotor.class, "frontrightmotor");
            backrightmotor  = myOpMode.hardwareMap.get(DcMotor.class, "backrightmotor");
            backleftmotor   = myOpMode.hardwareMap.get(DcMotor.class, "backleftmotor");
            frontleftmotor  = myOpMode.hardwareMap.get(DcMotor.class, "frontleftmotor");

            frontleftmotor.setDirection(DcMotor.Direction.FORWARD);
            backleftmotor.setDirection(DcMotor.Direction.FORWARD);
            frontrightmotor.setDirection(DcMotor.Direction.REVERSE);
            backrightmotor.setDirection(DcMotor.Direction.REVERSE);

            frontleftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontrightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backleftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backrightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            // Safe fallback if drive motors are missing
        }
    }

    public void readSensors() {}
    public void resetOdometry() {}
    public double getTurnRate() { return 0.0; }
    public double getHeading() { return 0.0; }

    /**
     * Drives the robot's Mecanum wheels based on drive, strafe, and yaw command values.
     */
    public void moveRobot(double drive, double strafe, double yaw) {
        double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(yaw), 1.0);
        
        frontLeftPower  = (drive + strafe + yaw) / denominator;
        backLeftPower   = (drive - strafe + yaw) / denominator;
        frontRightPower = (drive - strafe - yaw) / denominator;
        backRightPower  = (drive + strafe - yaw) / denominator;

        if (frontleftmotor != null)  frontleftmotor.setPower(frontLeftPower);
        if (backleftmotor != null)   backleftmotor.setPower(backLeftPower);
        if (frontrightmotor != null) frontrightmotor.setPower(frontRightPower);
        if (backrightmotor != null)  backrightmotor.setPower(backRightPower);
    }

    public void drive(double distance, double power, double wait) {
        myOpMode.sleep((long)(wait * 1000));
    }

    public void turn(double degrees, double power, double wait) {
        myOpMode.sleep((long)(wait * 1000));
    }

    public static class PIDController {
        public PIDController(double p, double i, double d) {}
        public void reset(double setpoint) {}
        public double getOutput(double current) { return 0.0; }
    }
}
