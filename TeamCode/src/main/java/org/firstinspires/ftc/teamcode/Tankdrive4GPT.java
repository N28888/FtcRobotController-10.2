package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TankDrive4_GPT", group = "TeleOp4")
public class Tankdrive4GPT extends OpMode {

    // Drive motors
    private DcMotor leftfront, rightfront, leftback, rightback;

    // Slide motors
    private DcMotor slideLeft, slideRight;

    // Odometry encoders
    private DcMotor vertEncoder, horizEncoder;

    // Slide variables
    private final int SLIDE_MIN_TICKS = 0;
    private final int SLIDE_MAX_TICKS = 73000;
    private double slideTargetPosition = 0;
    private final double SLIDE_POWER = 1.0;

    // Odometry tracking
    private int startVertTicks = 0;
    private int startHorizTicks = 0;
    private boolean autoReturning = false;

    @Override
    public void init() {
        // Drive motors
        leftfront = hardwareMap.get(DcMotor.class, "leftfront");
        rightfront = hardwareMap.get(DcMotor.class, "rightfront");
        leftback = hardwareMap.get(DcMotor.class, "leftback");
        rightback = hardwareMap.get(DcMotor.class, "rightback");

        leftfront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftback.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor[] driveMotors = {leftfront, rightfront, leftback, rightback};
        for (DcMotor motor : driveMotors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Slide motors
        slideLeft = hardwareMap.get(DcMotor.class, "slideLeft");
        slideRight = hardwareMap.get(DcMotor.class, "slideRight");

        slideRight.setDirection(DcMotorSimple.Direction.REVERSE);

        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Odometry
        vertEncoder = hardwareMap.get(DcMotor.class, "vertEncoder");
        horizEncoder = hardwareMap.get(DcMotor.class, "horizEncoder");

        vertEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Capture start position
        startVertTicks = vertEncoder.getCurrentPosition();
        startHorizTicks = horizEncoder.getCurrentPosition();

        telemetry.addData("Status", "Ready (One Gamepad Mode)");
        telemetry.update();
    }

    @Override
    public void loop() {
        // --- Drive control ---
        double forward = -gamepad1.right_stick_y;
        double strafe = gamepad1.right_stick_x;
        double rotate = gamepad1.left_stick_x;

        double lf = forward + strafe + rotate;
        double rf = forward - strafe - rotate;
        double lb = forward - strafe + rotate;
        double rb = forward + strafe - rotate;

        double max = Math.max(Math.abs(lf), Math.max(Math.abs(rf), Math.max(Math.abs(lb), Math.abs(rb))));
        if (max > 1.0) {
            lf /= max;
            rf /= max;
            lb /= max;
            rb /= max;
        }

        leftfront.setPower(lf);
        rightfront.setPower(rf);
        leftback.setPower(lb);
        rightback.setPower(rb);

        boolean robotStopped = (Math.abs(forward) < 0.05 && Math.abs(strafe) < 0.05 && Math.abs(rotate) < 0.05);

        // --- Slide control (right stick Y) ---
        double manual = -gamepad1.left_stick_y;
        int currentPos = slideLeft.getCurrentPosition();
        if (Math.abs(manual) > 0.05) {
            slideTargetPosition += manual * 50;
            slideTargetPosition = Range.clip(slideTargetPosition, SLIDE_MIN_TICKS, SLIDE_MAX_TICKS);
        }

        slideLeft.setTargetPosition((int) slideTargetPosition);
        slideRight.setTargetPosition((int) slideTargetPosition);
        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideLeft.setPower(SLIDE_POWER);
        slideRight.setPower(SLIDE_POWER);

        // --- Auto return to start ---
        if (gamepad1.b) {
            autoReturning = true;
        }

        if (autoReturning && robotStopped) {
            int vertDiff = startVertTicks - vertEncoder.getCurrentPosition();
            int horizDiff = startHorizTicks - horizEncoder.getCurrentPosition();

            // Convert encoder differences to motor movement estimates (you can adjust scaling)
            double correctionForward = vertDiff * 0.0005;
            double correctionStrafe = horizDiff * 0.0005;

            // Drive backward toward start
            leftfront.setPower(correctionForward + correctionStrafe);
            rightfront.setPower(correctionForward - correctionStrafe);
            leftback.setPower(correctionForward - correctionStrafe);
            rightback.setPower(correctionForward + correctionStrafe);

            // If close enough, stop returning
            if (Math.abs(vertDiff) < 20 && Math.abs(horizDiff) < 20) {
                autoReturning = false;
                stopAllDrive();
            }
        }

        telemetry.addData("Slide Position", slideLeft.getCurrentPosition());
        telemetry.addData("Slide Target", slideTargetPosition);
        telemetry.addData("Auto-Returning", autoReturning);
        telemetry.addData("Odometry Vert", vertEncoder.getCurrentPosition());
        telemetry.addData("Odometry Horiz", horizEncoder.getCurrentPosition());
        telemetry.update();
    }

    private void stopAllDrive() {
        leftfront.setPower(0);
        rightfront.setPower(0);
        leftback.setPower(0);
        rightback.setPower(0);
    }
}