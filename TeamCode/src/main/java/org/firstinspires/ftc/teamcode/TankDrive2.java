package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TankDrive2", group = "TeleOp")
public class TankDrive2 extends OpMode {

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor topMotor;
    private Servo frontServo;
    private Servo rotateServo;


    @Override
    public void init() {
        leftMotor = hardwareMap.get(DcMotor.class, "left");
        rightMotor = hardwareMap.get(DcMotor.class, "right");
        topMotor = hardwareMap.get(DcMotor.class, "top");
        frontServo = hardwareMap.get(Servo.class,"front");
        rotateServo = hardwareMap.get(Servo.class,"rotate");

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        // --- Drive Motors ---
        double drive = gamepad1.left_stick_y;
        double turn = -gamepad1.left_stick_x;

        double leftPower = drive - turn;
        double rightPower = drive + turn;

        double max = Math.max (Math. abs(leftPower), Math.abs(rightPower));
        if (max > 10){
            leftPower = leftPower / max;
            rightPower = rightPower / max;
        }

        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        // --- Top Motor Control ---
        double topPower = 0.1; // Default power is 0.1

        if (gamepad1.dpad_up) {
            topPower = -0.4;
        } else if (gamepad1.dpad_down) {
            topPower = 0.4;
        }
        // If neither is pressed, topPower remains 0.1 as set initially

        // Actually command the top motor
        topMotor.setPower(topPower);

        // Wheel Servo

        double frontPower = 0;

        if (gamepad1.left_bumper){
            frontPower = 0;
            frontServo.setPosition(frontPower);
        }
        else if(gamepad1.right_bumper){
            frontPower = 1;
            frontServo.setPosition(frontPower);
        }

        // Rotate Servo

        double rotatePower = 0.5;

        if(gamepad1.b){
            rotateServo.setPosition(0.81);
        } else if (gamepad1.y) {
            rotateServo.setPosition(0.5);
        } else if (gamepad1.x) {
            rotateServo.setPosition(0.17);
        }

        // --- Telemetry (Optional but Recommended) ---
        telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
        telemetry.addData("Right Stick Y", gamepad1.right_stick_y);
        telemetry.addData("Dpad Up", gamepad1.dpad_up);
        telemetry.addData("Dpad Down", gamepad1.dpad_down);
        telemetry.addData("Power", drive);
        telemetry.addData("Top Power", topPower);
        telemetry.update();
    }
}