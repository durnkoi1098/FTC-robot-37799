package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "alikhanCODJava")
public class alikhanCODJava extends OpMode {

    private DcMotor mator1left;   // алдыңғы сол
    private DcMotor matorLeft;    // артқы сол
    private DcMotor mator2right;  // алдыңғы оң
    private DcMotor matorRight;   // артқы оң

    // Алдымен 1.0 қалдырыңыз. Робот дұрыс жүргеннен кейін ғана өзгертіңіз
    private static final double STRAFE_GAIN = 1.0;
    private static final double TURN_GAIN = 1.0;
    // Бұрылу бағыты керісінше болса 1.0 -> -1.0 қылыңыз
    private static final double TURN_SIGN = 1.0;
    // Баяу режим (сол триггерді басып тұрғанда)
    private static final double SLOW_FACTOR = 0.4;
    // Стиктің өлі аймағы
    private static final double DEADZONE = 0.05;

    @Override
    public void init() {
        mator1left = hardwareMap.get(DcMotor.class, "mator1left");
        matorLeft = hardwareMap.get(DcMotor.class, "matorLeft");
        mator2right = hardwareMap.get(DcMotor.class, "mator2right");
        matorRight = hardwareMap.get(DcMotor.class, "matorRight");

        // Алға жүру жақсы болғандықтан бағыттар сол күйінде
        mator1left.setDirection(DcMotorSimple.Direction.REVERSE);
        matorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        mator2right.setDirection(DcMotorSimple.Direction.REVERSE);
        matorRight.setDirection(DcMotorSimple.Direction.FORWARD);

        DcMotor[] motors = {mator1left, matorLeft, mator2right, matorRight};
        for (DcMotor motor : motors) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        telemetry.addData("Статус", "Дайын!");
        telemetry.addData("Тест режимі", "BACK батырмасын басып тұрып A/B/X/Y");
        telemetry.update();
    }

    private double dz(double v) {
        return Math.abs(v) < DEADZONE ? 0 : v;
    }

    @Override
    public void loop() {
        // ---------- ТЕСТ РЕЖИМІ ----------
        // BACK басып тұрып: A = алдыңғы сол, B = алдыңғы оң,
        // X = артқы сол, Y = артқы оң. Әр дөңгелек АЛҒА айналуы керек.
        if (gamepad1.back) {
            mator1left.setPower(gamepad1.a ? 0.5 : 0);
            mator2right.setPower(gamepad1.b ? 0.5 : 0);
            matorLeft.setPower(gamepad1.x ? 0.5 : 0);
            matorRight.setPower(gamepad1.y ? 0.5 : 0);
            telemetry.addData("Режим", "ТЕСТ");
            telemetry.update();
            return;
        }

        // ---------- ҚАЛЫПТЫ ЖҮРІС ----------
        double forward = -dz(gamepad1.left_stick_y);
        double strafe = dz(gamepad1.left_stick_x) * STRAFE_GAIN;
        double turn = dz(gamepad1.right_stick_x) * TURN_GAIN * TURN_SIGN;

        double frontLeft = forward + strafe + turn;
        double backLeft = forward - strafe + turn;
        double frontRight = forward - strafe - turn;
        double backRight = forward + strafe - turn;

        double maxPower = Math.max(
                1.0,
                Math.max(
                        Math.max(Math.abs(frontLeft), Math.abs(backLeft)),
                        Math.max(Math.abs(frontRight), Math.abs(backRight))
                )
        );

        double speed = (gamepad1.left_trigger > 0.5) ? SLOW_FACTOR : 1.0;

        mator1left.setPower(frontLeft / maxPower * speed);
        matorLeft.setPower(backLeft / maxPower * speed);
        mator2right.setPower(frontRight / maxPower * speed);
        matorRight.setPower(backRight / maxPower * speed);

        telemetry.addData("Режим", "Жүріс");
        telemetry.addData("forward", "%.2f", forward);
        telemetry.addData("strafe", "%.2f", strafe);
        telemetry.addData("turn", "%.2f", turn);
        telemetry.addData("FL / BL", "%.2f / %.2f", frontLeft, backLeft);
        telemetry.addData("FR / BR", "%.2f / %.2f", frontRight, backRight);
        telemetry.addData("Баяу режим", speed < 1.0);
        telemetry.update();
    }

    @Override
    public void stop() {
        mator1left.setPower(0);
        matorLeft.setPower(0);
        mator2right.setPower(0);
        matorRight.setPower(0);
    }
}