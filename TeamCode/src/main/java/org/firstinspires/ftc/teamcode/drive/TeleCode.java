package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="DriverControlJr", group="Linear Opmode")
public class TeleCode extends LinearOpMode implements Runnable{
    public static HardwareFile robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new HardwareFile(hardwareMap);
        robot.driveTrain=new SampleMecanumDrive(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            run();
        }
    }

    double lxMult = 1;
    double lyMult = 1;
    double rxMult = 1;
    public void run() {
        robot.driveTrain.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y * lyMult,
                        -gamepad1.left_stick_x * lxMult,
                        -gamepad1.right_stick_x * 0.92 * rxMult
                )
        );
        setMultiplier();
        if(gamepad2.right_bumper) {
            robot.intakeMotor.setPower(-1);
            robot.transportMotor.setPower(0.9);
        }else if(gamepad2.right_trigger==1){
            robot.intakeMotor.setPower(1);
            robot.transportMotor.setPower(-0.9);
        }else{
            robot.intakeMotor.setPower(0);
            robot.transportMotor.setPower(0);
        }
        if(gamepad2.a){
            //robot.armWobble.setTargetPosition(93);
           // sleep(500);
            robot.grabberWobble.setPosition(0.63);
        }else if(gamepad2.y){
            robot.grabberWobble.setPosition(0.13);
           // sleep(500);
           // robot.armWobble.setTargetPosition(1);
        }
        if(gamepad2.left_bumper){
            robot.shooter.setPower(1);
            sleep(500);
            for(int i=0;i<=3;++i){
                robot.slapper.setPosition(0.35);
                sleep(100);
                robot.slapper.setPosition(0.5);
                sleep(1500);
            }
            robot.shooter.setPower(0);
        }
        if(gamepad2.dpad_up){
            robot.shooter.setPower(0.4);
            sleep(500);
            for(int i=0;i<1;++i){
                sleep(2500);
                robot.slapper.setPosition(0.35);
                sleep(100);
                robot.slapper.setPosition(0.5);
                sleep(1500);
            }
            robot.shooter.setPower(0);
        }
        telemetry.update();
        robot.driveTrain.update();
    }
    private void setMultiplier() {
        if (gamepad1.left_trigger >= 0.3) {
            lxMult = 0.5;
            rxMult = 0.5;
            lyMult = 0.5;
        }
        else {
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
