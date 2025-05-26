package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TankDrive", group = "TeleOp")
public class TankDrive extends OpMode {

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor topMotor;


    @Override
    public void init() {
        leftMotor = hardwareMap.get(DcMotor.class, "left");
        rightMotor = hardwareMap.get(DcMotor.class, "right");
        topMotor = hardwareMap.get(DcMotor.class, "top");
    }

    @Override
    public void loop() {
        // --- Drive Motors ---
        double leftPower = -gamepad1.left_stick_y;
        double rightPower = -gamepad1.right_stick_y;

        leftMotor.setPower(leftPower);
        rightMotor.setPower(-rightPower);

        // --- Top Motor Control ---
        double topPower = 0.1; // Default power is 0.1

        if (gamepad1.dpad_up) {
            // If D-pad Up is pressed, set positive power (e.g., move up)
            topPower = 0.6; // Use a value between 0.0 and 1.0
        } else if (gamepad1.dpad_down) {
            // Else, if D-pad Down is pressed, set negative power (e.g., move down)
            topPower = -0.6; // Use a value between -1.0 and 0.0
        }
        // If neither is pressed, topPower remains 0.1 as set initially

        // Actually command the top motor
        topMotor.setPower(topPower);

        // --- Telemetry (Optional but Recommended) ---
        telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
        telemetry.addData("Right Stick Y", gamepad1.right_stick_y);
        telemetry.addData("Dpad Up", gamepad1.dpad_up);
        telemetry.addData("Dpad Down", gamepad1.dpad_down);
        telemetry.addData("Left Power", leftPower);
        telemetry.addData("Right Power", rightPower);
        telemetry.addData("Top Power", topPower);
        telemetry.update();
    }
}
