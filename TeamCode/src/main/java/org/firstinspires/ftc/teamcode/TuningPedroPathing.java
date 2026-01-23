package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.follower.Follower;

import java.lang.reflect.Method;
import java.util.Arrays;
import java.util.Comparator;

@Autonomous(name = "PedroPathing Method Inspector", group = "Debug")
public class TuningPedroPathing extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Iniciando PedroPathing Inspector...");
        telemetry.update();

        Follower follower = null;
        try {
            follower = Constants.createFollower(hardwareMap);
        } catch (Exception e) {
            telemetry.addLine("ERRO: não foi possivel criar Follower:");
            telemetry.addLine(e.toString());
            telemetry.update();
            waitForStart();
            return;
        }

        // ---- Lista de métodos de Follower
        Class<?> folClass = follower.getClass();
        Method[] folMethods = folClass.getMethods();
        Arrays.sort(folMethods, Comparator.comparing(Method::getName));

        telemetry.addLine("==== Métodos públicos de Follower (" + folMethods.length + ") ====");
        int count = 0;
        for (Method m : folMethods) {
            telemetry.addData(String.format("%02d", ++count), m.getName() + paramTypes(m));
            if (count % 6 == 0) telemetry.update();
        }
        telemetry.update();

        // ---- Tenta descobrir path builder / newPath
        telemetry.addLine("");
        telemetry.addLine("Procurando path builder methods (pathBuilder/newPath)...");
        telemetry.update();

        Method builderMethod = null;
        try {
            // tenta pathBuilder() sem args
            builderMethod = folClass.getMethod("pathBuilder");
        } catch (NoSuchMethodException e) {
            // ignora
        }
        if (builderMethod == null) {
            try {
                builderMethod = folClass.getMethod("pathBuilder", Class.class); // algumas versões pedem Pose class
            } catch (NoSuchMethodException e) {
                // ignora
            }
        }
        if (builderMethod == null) {
            try {
                builderMethod = folClass.getMethod("newPath");
            } catch (NoSuchMethodException e) {
                // ignora
            }
        }

        if (builderMethod == null) {
            telemetry.addLine("Nenhum método pathBuilder() ou newPath() encontrado na classe Follower.");
            telemetry.update();
        } else {
            telemetry.addLine("Encontrado builder: " + builderMethod.toString());
            telemetry.update();

            // invoca o builder e lista métodos do builder
            Object builderObj = null;
            try {
                // tenta invocar sem args
                builderObj = builderMethod.invoke(follower);
            } catch (Exception e) {
                // tentamos invocar com a pose atual (algumas versões exigem)
                try {
                    Method getPose = folClass.getMethod("getPose");
                    Object pose = getPose.invoke(follower);
                    builderObj = builderMethod.invoke(follower, pose.getClass());
                } catch (Exception ex) {
                    telemetry.addLine("Falha ao invocar builder: " + ex.toString());
                    telemetry.update();
                }
            }

            if (builderObj != null) {
                Class<?> builderClass = builderObj.getClass();
                Method[] bMethods = builderClass.getMethods();
                Arrays.sort(bMethods, Comparator.comparing(Method::getName));
                telemetry.addLine("==== Métodos do Builder (" + bMethods.length + ") ====");
                int i = 0;
                for (Method m : bMethods) {
                    telemetry.addData(String.format("%02d", ++i), m.getName() + paramTypes(m));
                    if (i % 6 == 0) telemetry.update();
                }
                telemetry.update();
            }
        }

        telemetry.addLine("");
        telemetry.addLine("=== FIM DO INSPECTOR ===");
        telemetry.addLine("Copie TODO o telemetry e cole na conversa.");
        telemetry.update();

        waitForStart(); // fica esperando, para você ler/pegar telemetry
    }

    private String paramTypes(Method m) {
        Class<?>[] ps = m.getParameterTypes();
        if (ps.length == 0) return "()";
        StringBuilder sb = new StringBuilder("(");
        for (int i = 0; i < ps.length; i++) {
            if (i > 0) sb.append(", ");
            sb.append(ps[i].getSimpleName());
        }
        sb.append(")");
        return sb.toString();
    }
}
