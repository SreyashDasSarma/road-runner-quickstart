package org.firstinspires.ftc.teamcode.drive.SampleAutonPaths;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class HardwareFile {
    private LinearOpMode linearOpMode;
    public DcMotor intakeMotor, shooter;
    public Servo armWobble;
    public Servo grabberWobble, slapper, tilter, shooterflap;
    HardwareMap map;
    public SampleMecanumDrive driveTrain;
    public static Pose2d robotPose = new Pose2d();

    public HardwareFile(HardwareMap imported) {
        //robotPose = new Pose2d(x - 70.4725, y - 70.4725, robotOrientation);
        construct(imported);
    }
    private void construct(HardwareMap imported){
        map = imported;
        intakeMotor = map.get(DcMotor.class, "intakeR");
        shooter = map.get(DcMotor.class, "fw");
        armWobble = map.get(Servo.class, "wobbleArm2");
        grabberWobble = map.get(Servo.class, "wobbleGrabber2");
        slapper = map.get(Servo.class, "mag");
        tilter = map.get(Servo.class, "tilt");
        shooterflap = map.get(Servo.class, "flap");

    }
    public void wobbleArmUp() {
        armWobble(0.88);
    }
    public void wobbleArmDown() {
        armWobble (0.07);

    }
    public void wobbleArmVertical(){
        armWobble(0.5);
    }
    public void grab(){
        grabberWobble(0.83);
    }
    public void release(){
        grabberWobble(0.29);
    }
    public void intake(double intakeSpeed){
        intakeMotor(-intakeSpeed);
    }

    public void shooter(double shooterpower){
        shooter.setPower(shooterpower);
    }
    public void magup(){
        tilter.setPosition(1);
    }
    public void magdown(){
        tilter.setPosition(0.5);
    }

}
