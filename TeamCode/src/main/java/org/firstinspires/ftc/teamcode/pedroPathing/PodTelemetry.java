package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.pedropathing.geometry.Pose;
import com.pedropathing.follower.Follower;

@TeleOp(name = "ODOMETRIA PODS", group = "Teste")
public class PodTelemetry extends OpMode {

    Follower follower;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        // mesma pose inicial do auto
        follower.setStartingPose(new Pose(21.5, 124.5, Math.toRadians(143)));

        telemetry.addLine("Pronto para mover o robô");
        telemetry.update();
    }

    @Override
    public void loop() {

        // atualiza odometria
        follower.update();

        Pose pose = follower.getPose();

        telemetry.addLine("===== POSICAO DO ROBO =====");
        telemetry.addData("X (cm)", "%.2f", pose.getX());
        telemetry.addData("Y (cm)", "%.2f", pose.getY());
        telemetry.addData("Heading (graus)", "%.2f", Math.toDegrees(pose.getHeading()));
        telemetry.addData("Heading (rad)", "%.3f", pose.getHeading());

        telemetry.addLine("---------------------------");
        telemetry.addLine("Empurre o robo com a mao");

        telemetry.update();
    }
}