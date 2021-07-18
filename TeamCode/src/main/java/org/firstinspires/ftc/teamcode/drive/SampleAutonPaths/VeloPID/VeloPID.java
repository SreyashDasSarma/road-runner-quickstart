package org.firstinspires.ftc.teamcode.drive.SampleAutonPaths.VeloPID;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp
public class VeloPID extends LinearOpMode {
    public static PIDCoefficients MOTOR_VELO_PID = new PIDCoefficients(0.1, 0.1, 0.1);

    public static double kV = 1 / TuningController.rpmToTicksPerSecond(TuningController.MOTOR_MAX_RPM);
    public static double kA = 1;
    public static double kStatic = 1;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    private final ElapsedTime veloTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        // Change my id
        DcMotorEx myMotor1 = hardwareMap.get(DcMotorEx.class, "fw1");
        DcMotorEx myMotor2 = hardwareMap.get(DcMotorEx.class, "fw");
        Servo tilter = hardwareMap.get(Servo.class, "tilt");
        Servo slapper = hardwareMap.get(Servo.class, "mag");
        // Reverse as appropriate
        // myMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        // myMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        myMotor1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        myMotor2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        VelocityPIDF veloController = new VelocityPIDF(MOTOR_VELO_PID, kV, kA, kStatic);
        TuningController tuningController = new TuningController();

        double lastTargetVelo = 0.0;
        double lastKv = kV;
        double lastKa = kA;
        double lastKstatic = kStatic;

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addLine("Ready!");
        telemetry.update();
        telemetry.clearAll();

        waitForStart();
        tilter.setPosition(1);
        if (isStopRequested()) return;

        tuningController.start();
        veloTimer.reset();
        int count=-1;
        while (!isStopRequested() && opModeIsActive()) {
            double targetVelo = tuningController.update();

            veloController.setTargetVelocity(targetVelo);
            veloController.setTargetAcceleration((targetVelo - lastTargetVelo) / veloTimer.seconds());
            veloTimer.reset();

            lastTargetVelo = targetVelo;

            telemetry.addData("targetVelocity", targetVelo);

            double motorPos = myMotor2.getCurrentPosition();
            double motorVelo = myMotor2.getVelocity();

            double power = veloController.update(motorPos, motorVelo);
            myMotor1.setPower(power);
            myMotor2.setPower(power);

            if(lastKv != kV || lastKa != kA || lastKstatic != kStatic) {
                lastKv = kV;
                lastKa = kA;
                lastKstatic = kStatic;

                veloController = new VelocityPIDF(MOTOR_VELO_PID, kV, kA, kStatic);
            }
            if(motorVelo>1200&&count<3){
                //if(count>=0) {
                    slapper.setPosition(0.35);
                    sleep(100);
                    slapper.setPosition(0.5);
                    sleep(200);
                //}
                count=(count+1);
            }
            telemetry.addData("velocity", motorVelo);
            telemetry.addData("error", targetVelo - motorVelo);
            telemetry.addData("upperBound", TuningController.rpmToTicksPerSecond(TuningController.TESTING_MAX_SPEED * 1.15));
            telemetry.addData("lowerBound", 0);
            telemetry.update();
        }
    }
}
