package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@Autonomous
public class Tunagem extends LinearOpMode {

    DcMotor fe, fd, te, td;

    // ===== PID =====
    double kP = 0.006;
    double kI = 0.00000;
    double kD = 0.0006;

    double erro;
    double ultimoErro = 0;
    double integral = 0;

    // ===== CONSTANTES =====
    final double DIAMETRO_RODA_CM = 9.6;
    final double TICKS_POR_VOLTA = 537.7;
    final double CM_POR_VOLTA = Math.PI * DIAMETRO_RODA_CM;

    final double POWER_MIN = 0.15;
    final double POWER_MAX = 0.6;

    int cmParaTicks(double cm) {
        return (int) ((cm / CM_POR_VOLTA) * TICKS_POR_VOLTA);
    }

    int encoderMedio() {
        return (fe.getCurrentPosition()
                + fd.getCurrentPosition()
                + te.getCurrentPosition()
                + td.getCurrentPosition()) / 4;
    }

    @Override
    public void runOpMode() {

        fe = hardwareMap.get(DcMotor.class, "FE");
        fd = hardwareMap.get(DcMotor.class, "FD");
        te = hardwareMap.get(DcMotor.class, "TE");
        td = hardwareMap.get(DcMotor.class, "TD");

        fe.setDirection(DcMotorSimple.Direction.REVERSE);
        te.setDirection(DcMotorSimple.Direction.REVERSE);

        fe.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        te.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        td.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fe.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        te.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        td.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        // ===== 2 METROS =====
        int alvo = cmParaTicks(200);

        while (opModeIsActive()) {

            int atual = encoderMedio();
            erro = alvo - atual;

            // ===== CHEGOU =====
            if (Math.abs(erro) < 20) break;

            // ===== PID =====
            integral += erro;
            integral = Range.clip(integral, -3000, 3000); // anti-windup

            double derivada = erro - ultimoErro;

            double power =
                    (erro * kP) +
                            (integral * kI) +
                            (derivada * kD);

            // ===== POTÊNCIA MÍNIMA =====
            if (Math.abs(power) < POWER_MIN) {
                power = Math.signum(power) * POWER_MIN;
            }

            power = Range.clip(power, -POWER_MAX, POWER_MAX);

            fe.setPower(power);
            fd.setPower(power);
            te.setPower(power);
            td.setPower(power);

            ultimoErro = erro;

            // ===== TELEMETRIA =====
            telemetry.addData("Alvo (ticks)", alvo);
            telemetry.addData("Atual", atual);
            telemetry.addData("Erro", erro);
            telemetry.addData("P", erro * kP);
            telemetry.addData("I", integral * kI);
            telemetry.addData("D", derivada * kD);
            telemetry.addData("Power", power);
            telemetry.update();
        }

        // ===== PARA =====
        fe.setPower(0);
        fd.setPower(0);
        te.setPower(0);
        td.setPower(0);
    }
}
