package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;// Import DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TankDrive3_Mecanum", group = "TeleOp4")
public class TankDrive3_Mecanum extends OpMode {

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor rightbackMotor;
    private DcMotor leftbackMotor;

    @Override
    public void init() {
        leftMotor = hardwareMap.get(DcMotor.class, "left"); //1
        rightMotor = hardwareMap.get(DcMotor.class, "right"); //0
        rightbackMotor = hardwareMap.get(DcMotor.class, "rightback"); //3
        leftbackMotor = hardwareMap.get(DcMotor.class, "leftback"); //2

        // ***** 重要: 根据你的齿轮传动和电机实际旋转方向进行设置 *****
        // 下面的设置仅为示例，你需要根据你的测试结果修改
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        leftbackMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        rightbackMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        // 读取摇杆输入
        double y = -gamepad1.left_stick_y; // 前后移动
        double x = gamepad1.left_stick_x;  // 左右平移
        double rx = gamepad1.right_stick_x; // 旋转

        // 计算每个轮子的功率
        double leftFrontPower = y + x + rx;
        double leftBackPower = y - x + rx;
        double rightFrontPower = y - x - rx;
        double rightBackPower = y + x - rx;

        // 归一化，避免功率超过 ±1
        double max = Math.max(
                Math.max(Math.abs(leftFrontPower), Math.abs(leftBackPower)),
                Math.max(Math.abs(rightFrontPower), Math.abs(rightBackPower))
        );
        if (max > 1.0) {
            leftFrontPower /= max;
            leftBackPower /= max;
            rightFrontPower /= max;
            rightBackPower /= max;
        }

        // 设置电机功率
        leftMotor.setPower(leftFrontPower);
        leftbackMotor.setPower(leftBackPower);
        rightMotor.setPower(rightFrontPower);
        rightbackMotor.setPower(rightBackPower);
    }
}