package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TankDrive_test", group = "TeleOp")
public class TankDrive_test extends OpMode {
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    @Override
    public void init() {
        leftDrive = hardwareMap.dcMotor.get("leftDrive");
        rightDrive = hardwareMap.dcMotor.get("rightDrive");
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        double leftPower = -gamepad1.left_stick_y;
        double rightPower = -gamepad1.right_stick_y;
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);

        telemetry.addData("leftPower", leftPower);
        telemetry.addData("rightPower", rightPower);
}

}
