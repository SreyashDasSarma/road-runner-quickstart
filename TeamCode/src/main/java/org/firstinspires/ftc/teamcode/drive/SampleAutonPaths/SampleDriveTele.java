package org.firstinspires.ftc.teamcode.drive.SampleAutonPaths;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

//import org.firstinspires.ftc.teamcode.drive.Hardware.Robot;
@TeleOp(name="TwoDrivers", group="Linear Opmode")
public class SampleDriveTele extends LinearOpMode implements Runnable{
    public static HardwareFile robot;
    public static double MOTOR_TICKS_PER_REV = 560;
    public static double MOTOR_MAX_RPM = 300;
    public static double MOTOR_GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed

    public static boolean RUN_USING_ENCODER = true;
    public static boolean DEFAULT_GAINS = false;

    public static double TESTING_SPEED = 0.9 * MOTOR_MAX_RPM;

    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(30, 0, 2, 15);

    private double lastKp = 0.0;
    private double lastKi = 0.0;
    private double lastKd = 0.0;
    private double lastKf = getMotorVelocityF();

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    private VoltageSensor batteryVoltageSensor;
    private void printVelocity(DcMotorEx motor, double target) {
        telemetry.addData("targetVelocity", rpmToTicksPerSecond(target));

        double motorVelo = motor.getVelocity();
        telemetry.addData("velocity", motorVelo);
        telemetry.addData("error", rpmToTicksPerSecond(target) - motorVelo);

        telemetry.addData("upperBound", rpmToTicksPerSecond(TESTING_SPEED) * 1.15);
        telemetry.addData("lowerBound", 0);
    }

    private void setVelocity(DcMotorEx motor, double power) {
        if(RUN_USING_ENCODER) {
            motor.setVelocity(rpmToTicksPerSecond(power));
            Log.i("mode", "setting velocity");
        }
        else {
            Log.i("mode", "setting power");
            motor.setPower(power / MOTOR_MAX_RPM);
        }
    }

    private void setPIDFCoefficients(DcMotorEx motor, PIDFCoefficients coefficients) {
        if(!RUN_USING_ENCODER) {
            Log.i("config", "skipping RUE");
            return;
        }

        if (!DEFAULT_GAINS) {
            Log.i("config", "setting custom gains");
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                    coefficients.p, coefficients.i, coefficients.d, coefficients.f * 12 / batteryVoltageSensor.getVoltage()
            ));
        } else {
            Log.i("config", "setting default gains");
        }
    }

    public static double rpmToTicksPerSecond(double rpm) {
        return rpm * MOTOR_TICKS_PER_REV / MOTOR_GEAR_RATIO / 60;
    }

    public static double getMotorVelocityF() {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 * 60.0 / (MOTOR_MAX_RPM * MOTOR_TICKS_PER_REV);
    }
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new HardwareFile(hardwareMap);
        robot.driveTrain=new SampleMecanumDrive(hardwareMap);
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        setPIDFCoefficients(robot.shooter, MOTOR_VELO_PID);
        waitForStart();
        guardpos=false;
        running=false;
        while (opModeIsActive()) {
            run();
        }
    }

    double lxMult = 1;
    double lyMult = 1;
    double rxMult = 1;
    public boolean running;
    public void run() {
        robot.driveTrain.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y * lyMult,
                        -gamepad1.left_stick_x * lxMult,
                        -gamepad1.right_stick_x * 0.92 * rxMult
                )
        );
        setMultiplier();
        if(gamepad2.right_bumper){
            robot.intakeL.setPower(-1);
            robot.intakeR.setPower(-1);
            robot.in1.setPower(1);
            robot.in2.setPower(1);
        }else{
            robot.intakeL.setPower(0);
            robot.intakeR.setPower(0);
            robot.in1.setPower(0);
            robot.in2.setPower(0);
        }
        if(gamepad2.y){
            robot.arm1.setPosition(0.93);
            robot.arm2.setPosition (0.07);
            sleep(500);
            robot.grabber.setPosition(0.63);
            robot.grabber2.setPosition(0.29);
        }else if(gamepad2.a){
            robot.grabber.setPosition(0.13);
            robot.grabber2.setPosition(0.83);
            sleep(500);
            robot. arm1.setPosition(0.1);
            robot.arm2.setPosition (0.88);
        }
        if(gamepad2.left_bumper){
            running=true;
            robot.leftIntakeHolder.setPosition(1);
            robot.rightIntakeHolder.setPosition(0);
            int i=0;
            while (!isStopRequested()&&i!=4) {
                setVelocity(robot.shooter, TESTING_SPEED);

                printVelocity(robot.shooter, TESTING_SPEED);

                if (lastKp != MOTOR_VELO_PID.p || lastKi != MOTOR_VELO_PID.i || lastKd != MOTOR_VELO_PID.d || lastKf != MOTOR_VELO_PID.f) {
                    setPIDFCoefficients(robot.shooter, MOTOR_VELO_PID);

                    lastKp = MOTOR_VELO_PID.p;
                    lastKi = MOTOR_VELO_PID.i;
                    lastKd = MOTOR_VELO_PID.d;
                    lastKf = MOTOR_VELO_PID.f;
                }
                if(rpmToTicksPerSecond(TESTING_SPEED) - robot.shooter.getVelocity()<1150){
                    robot.slapper.setPosition(0.35);
                    sleep(100);
                    robot.slapper.setPosition(0.5);
                    sleep(100);
                   
                    ++i;
                    telemetry.addData("I is", i);
                }
                telemetry.update();
            }
            robot.shooter.setVelocity(0);
            robot.magdown();
            running=false;
        }
        if(gamepad2.x){
            robot.leftIntakeHolder.setPosition(1);
            robot.rightIntakeHolder.setPosition(0);
            guardpos=false;
        }else if(gamepad2.b){
            robot.leftIntakeHolder.setPosition(0.5);
            robot.rightIntakeHolder.setPosition(0.5);
            guardpos=true;
        }
        telemetry.addData("Protect:",guardpos);
        telemetry.update();
        robot.driveTrain.update();
    }
    public static boolean guardpos;
    private void setMultiplier() {
        if (gamepad1.left_trigger >= 0.3) {
            lxMult = 0.5;
            rxMult = 0.5;
            lyMult = 0.5;
        } else {
            lxMult = 1;
            rxMult = 1;
            lyMult = 1;
        }
        if (gamepad1.right_bumper) {
            lxMult = -lxMult;
            lyMult = -lyMult;
        }
    }
}
