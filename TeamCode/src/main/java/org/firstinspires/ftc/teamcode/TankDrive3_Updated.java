package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TankDrive3_Updated", group = "TeleOp3")
public class TankDrive3_Updated extends OpMode {

    private DcMotor leftfrontMotor;
    private DcMotor rightfrontMotor;
    private DcMotor leftbackMotor;
    private DcMotor rightbackMotor;
    private DcMotorEx armMotor;
    private DcMotor slideMotor;
    private Servo wheelServo;
    private Servo topServo;

    private int armTargetPosition;
    private boolean isHoldingArm = false;

    private final double MANUAL_ARM_POWER = 0.6;
    private final double HOLDING_ARM_POWER = 0.5;

    @Override
    public void init() {
        leftfrontMotor = hardwareMap.get(DcMotor.class, "leftfront");
        rightfrontMotor = hardwareMap.get(DcMotor.class, "rightfront");
        leftbackMotor = hardwareMap.get(DcMotor.class, "leftback");
        rightbackMotor = hardwareMap.get(DcMotor.class, "rightback");
        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");

        wheelServo = hardwareMap.get(Servo.class, "wheelServo");
        topServo = hardwareMap.get(Servo.class, "topServo");

        try {
            armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armMotor.setDirection(DcMotor.Direction.FORWARD);
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armTargetPosition = 0;
            armMotor.setTargetPosition(armTargetPosition);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            isHoldingArm = true;
            armMotor.setPower(HOLDING_ARM_POWER);
        } catch (Exception e) {
            telemetry.addData("Error", "armMotor Init Failed: %s", e.getMessage());
            armMotor = null;
        }

        leftfrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftbackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightfrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightbackMotor.setDirection(DcMotor.Direction.FORWARD);

        leftfrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightfrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftbackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightbackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.addData(">", "Motor Directions MUST be Verified!");
        telemetry.update();
    }

    @Override
    public void loop() {
        double forward = gamepad1.right_stick_y;
        double pan = -gamepad1.right_stick_x;
        double rotate = gamepad1.left_stick_x;

        double slideExtend = gamepad1.right_trigger;
        double slideRetract = gamepad1.left_trigger;

        double leftFrontPower = forward + pan + rotate;
        double rightFrontPower = forward - pan - rotate;
        double leftBackPower = forward - pan + rotate;
        double rightBackPower = forward + pan - rotate;

        double slidePower = slideExtend - slideRetract;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        leftfrontMotor.setPower(leftFrontPower);
        rightfrontMotor.setPower(rightFrontPower);
        leftbackMotor.setPower(leftBackPower);
        rightbackMotor.setPower(rightBackPower);
        slideMotor.setPower(slidePower);

        if (armMotor != null) {
            boolean dpadUpPressed = gamepad1.dpad_up;
            boolean dpadDownPressed = gamepad1.dpad_down;

            if (dpadUpPressed || dpadDownPressed) {
                isHoldingArm = false;
                armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                if (dpadUpPressed) {
                    armMotor.setPower(-MANUAL_ARM_POWER);
                } else {
                    armMotor.setPower(MANUAL_ARM_POWER);
                }
            } else {
                if (!isHoldingArm) {
                    armTargetPosition = armMotor.getCurrentPosition();
                    armMotor.setTargetPosition(armTargetPosition);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(HOLDING_ARM_POWER);
                    isHoldingArm = true;
                } else {
                    if (armMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                        armMotor.setTargetPosition(armTargetPosition);
                        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }
                    armMotor.setPower(HOLDING_ARM_POWER);
                }
            }
        }
        double topServoPosition = 0.49;
        if(gamepad1.x){
            topServo.setPosition(0.81);
        } else if (gamepad1.y) {
            topServo.setPosition(0.49);
        } else if (gamepad1.b) {
            topServo.setPosition(0.17);
        }

        if(gamepad1.left_bumper){
            wheelServo.setPosition(1);
        } else if (gamepad1.right_bumper) {
            wheelServo.setPosition(0);
        }

        telemetry.addData("Input Y (Forward)", "%.2f", forward);
        telemetry.addData("Input X (Pan)", "%.2f", pan);
        telemetry.addData("Input R (Rotate)", "%.2f", rotate);
        telemetry.addData("Left Trigger", "%.2f", slideRetract);
        telemetry.addData("Right Trigger", "%.2f", slideExtend);
        telemetry.addData("Calculated LF Power", "%.2f", leftFrontPower);
        telemetry.addData("Calculated RF Power", "%.2f", rightFrontPower);
        telemetry.addData("Calculated LB Power", "%.2f", leftBackPower);
        telemetry.addData("Calculated RB Power", "%.2f", rightBackPower);
        telemetry.addData("Actual LF Power", "%.2f", leftfrontMotor.getPower());
        telemetry.addData("Actual RF Power", "%.2f", rightfrontMotor.getPower());
        telemetry.addData("Actual LB Power", "%.2f", leftbackMotor.getPower());
        telemetry.addData("Actual RB Power", "%.2f", rightbackMotor.getPower());
        telemetry.addData("Slide Power", "%.2f", slideMotor.getPower());
        telemetry.addData("Arm Power", armMotor != null ? String.format("%.2f", armMotor.getPower()) : "null");
        telemetry.update();
    }
}