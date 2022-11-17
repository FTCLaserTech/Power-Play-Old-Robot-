package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class TrajectoryBook
{
    SampleMecanumDrive drive;
    ExtraOpModeFunctions extras;

    public TrajectorySequence rightHighJunctionTwo;

    public TrajectorySequence redCarouselSpinDuck;
    public TrajectorySequence blueCarouselSpinDuck;

    public TrajectorySequence redWarehouseRightBlockDrop;
    public TrajectorySequence redWarehouseMiddleBlockDrop;
    public TrajectorySequence redWarehouseLeftBlockDrop;
    public TrajectorySequence blueWarehouseRightBlockDrop;
    public TrajectorySequence blueWarehouseMiddleBlockDrop;
    public TrajectorySequence blueWarehouseLeftBlockDrop;


    public TrajectorySequence redRightBlockCarousel;
    public TrajectorySequence redMiddleBlockCarousel;
    public TrajectorySequence redLeftBlockCarousel;
    public TrajectorySequence blueRightBlockCarousel;
    public TrajectorySequence blueMiddleBlockCarousel;
    public TrajectorySequence blueLeftBlockCarousel;


    public TrajectorySequence redParkInWarehouseFromCarousel;
    public TrajectorySequence redParkInStorageFromCarousel;
    public TrajectorySequence blueParkInWarehouseCarousel;
    public TrajectorySequence blueParkInStorageFromCarousel;

    public TrajectorySequence redParkInWarehouseFromWarehouse;
    public TrajectorySequence blueParkInWarehouseFromWarehouse;

    public TrajectorySequence redRightDuckBlock;

    public Trajectory blueWarehouseShift;
    public Trajectory redWarehouseShift;

    // These trajectories deliver the duck after it is picked up
    public TrajectorySequence redDuckStorageUnitRight;
    public TrajectorySequence redDuckStorageUnitMiddle;
    public TrajectorySequence redDuckStorageUnitLeft;

    public TrajectorySequence redDuckPark;
    public TrajectorySequence blueDuckStorageUnitRight;
    public TrajectorySequence blueDuckStorageUnitMiddle;
    public TrajectorySequence blueDuckStorageUnitLeft;
    public TrajectorySequence blueDuckPark;
    public TrajectorySequence redCapstoneWarehouse;
    public TrajectorySequence redCapstoneWarehouseShift;
    public TrajectorySequence blueCapstoneWarehouse;
    public TrajectorySequence blueCapstoneWarehouseShift;

    public TrajectoryBook (SampleMecanumDrive drivePass, ExtraOpModeFunctions extrasPass)
    {
        drive = drivePass;
        extras = extrasPass;
    }

    public void RightHighJunctionTwo(Pose2d pose)
    {
        rightHighJunctionTwo = drive.trajectorySequenceBuilder(pose)
                // Move towards carousel
                .lineToLinearHeading(new Pose2d(5, 19, Math.toRadians(0)))
                .waitSeconds(0.4)
                .lineToLinearHeading(new Pose2d(52, 19, Math.toRadians(0)))
                .waitSeconds(0.4)
                .lineToLinearHeading(new Pose2d(52, 11, Math.toRadians(-87)))
                .waitSeconds(0.4)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpen())
                .waitSeconds(0.4)
                .lineToLinearHeading(new Pose2d(52, -25, Math.toRadians(-87)))
                .waitSeconds(0.4)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawClose())
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(52, 11, Math.toRadians(-87)))
                .waitSeconds(0.4)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpen())
                .build();
    }
    //
    // Used in red carousel programs at the beginning to spin the duck
    public void RedCarouselSpinDuck(Pose2d pose)
    {
        redCarouselSpinDuck = drive.trajectorySequenceBuilder(pose)
                // Move towards carousel
                .lineToLinearHeading(
                        new Pose2d(8, 17.5, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(19, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                // Spin the carousel
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.leftWheelCCW())
                .waitSeconds(2.7)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.leftWheelOff())
                // Move 'vertically' to shipping hub along wall
                .lineToLinearHeading(new Pose2d(40, 17, Math.toRadians(90)))
                .build();
    }

    // Used in red carousel, freight in right, and parking in storage
    public void RedRightBlockCarousel(Pose2d pose)
    {
        redRightBlockCarousel = drive.trajectorySequenceBuilder(pose)
                // Move 'horizontally' to shipping hub to drop the freight
                .lineToLinearHeading(new Pose2d(40, -9, Math.toRadians(90)))
                // Bring up arm and drop freight
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armTopAuto())
                .waitSeconds(2.0)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpen())
                .waitSeconds(0.5)
                // Back away from shipping hub
                .lineToLinearHeading(new Pose2d(40, 2, Math.toRadians(90)))
                // Move arm to collect position
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armCollect())
                .build();
    }

    // Used in red carousel, freight in middle, and parking in storage
    public void RedMiddleBlockCarousel(Pose2d pose)
    {
        redMiddleBlockCarousel = drive.trajectorySequenceBuilder(pose)
                // Move to shipping hub
                .lineToLinearHeading(new Pose2d(40, 1, Math.toRadians(90)))
                // Bring up arm and drop freight
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armMid())
                .waitSeconds(2.0)
                // Move closer to the shipping hub
                .lineToLinearHeading(new Pose2d(40, 2, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(0., () -> extras.clawOpen())
                .waitSeconds(0.5)
                // Back up from shipping hub
                .lineToLinearHeading(new Pose2d(40, 2, Math.toRadians(90)))
                // Arm to collect position
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armCollect())
                .build();
    }

    // Used in red carousel, freight in left, and parking in storage
    public void RedLeftBlockCarousel(Pose2d pose)
    {
        redLeftBlockCarousel = drive.trajectorySequenceBuilder(pose)
                // Move to shipping hub
                //.turn(Math.toRadians(-180))
                // Bring up arm and drop freight
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armBottomAuto())
                .waitSeconds(2.0)
                .lineToLinearHeading(new Pose2d(40, -10, Math.toRadians(-90)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpen())
                .waitSeconds(0.5)
                // Back up from shipping hub
                .lineToLinearHeading(new Pose2d(40, 2, Math.toRadians(-90)))
                // Arm to collect position
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armCollect())
                .build();
    }

    public void RedParkInStorageFromCarousel (Pose2d pose)
    {
        redParkInStorageFromCarousel = drive.trajectorySequenceBuilder(pose)
                // Back up more from shipping hub
                .lineToLinearHeading(new Pose2d(40, 16, Math.toRadians(-90)))
                // Move to park in storage unit
                .lineToLinearHeading(new Pose2d(29.5, 21, Math.toRadians(-90)))
                .build();
    }

    public void RedParkInWarehouseFromCarousel(Pose2d pose)
    {
        redParkInWarehouseFromCarousel = drive.trajectorySequenceBuilder(pose)
                // Back up more from shipping hub
                .lineToLinearHeading(new Pose2d(40, 12, Math.toRadians(90)))
                // Move and park in warehouse
                .lineToLinearHeading(new Pose2d(-1, 12, Math.toRadians(90)))
                .waitSeconds(1.5)
                .lineToLinearHeading(new Pose2d(-3, -84, Math.toRadians(90)))
                .build();
    }

    public void BlueCarouselSpinDuck(Pose2d pose)
    {
        blueCarouselSpinDuck = drive.trajectorySequenceBuilder(pose)
                // Move towards carousel
                .lineToLinearHeading(
                        new Pose2d(8, -17.5, Math.toRadians(-90)),
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                // Spin the carousel
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.rightWheelCCW())
                .waitSeconds(2.7)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.rightWheelOff())
                // Move 'vertically' to shipping hub along wall
                .lineToLinearHeading(new Pose2d(40, -17, Math.toRadians(-90)))
                .build();
    }

    public void BlueRightBlockCarousel(Pose2d pose)
    {
        blueRightBlockCarousel = drive.trajectorySequenceBuilder(pose)
                // Move 'horizontally' to shipping hub to drop the freight
                .lineToLinearHeading(new Pose2d(40, 9, Math.toRadians(-90)))
                // Bring up arm and drop freight
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armTopAuto())
                .waitSeconds(2.0)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpen())
                .waitSeconds(0.5)
                // Back away from shipping hub
                .lineToLinearHeading(new Pose2d(40, -2, Math.toRadians(-90)))
                // Move arm to collect position
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armCollect())
                .build();
    }

    public void BlueMiddleBlockCarousel(Pose2d pose)
    {
        blueMiddleBlockCarousel = drive.trajectorySequenceBuilder(pose)
                // Move to shipping hub
                .lineToLinearHeading(new Pose2d(40, -1, Math.toRadians(-90)))
                // Bring up arm and drop freight
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armMid())
                .waitSeconds(2.0)
                .UNSTABLE_addTemporalMarkerOffset(0., () -> extras.clawOpen())
                .waitSeconds(0.5)
                // Back up from shipping hub
                .lineToLinearHeading(new Pose2d(40, -2, Math.toRadians(-90)))
                // Arm to collect position
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armCollect())
                .build();
    }

    public void BlueLeftBlockCarousel(Pose2d pose)
    {
        blueLeftBlockCarousel = drive.trajectorySequenceBuilder(pose)
                // Move to shipping hub
                //.turn(Math.toRadians(-180))
                // Bring up arm and drop freight
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armBottomAuto())
                .waitSeconds(2.0)
                .lineToLinearHeading(new Pose2d(40, 9, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpen())
                .waitSeconds(0.5)
                // Back up from shipping hub
                .lineToLinearHeading(new Pose2d(40, -2, Math.toRadians(90)))
                // Arm to collect position
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armCollect())
                .build();
    }

    public void BlueParkInWarehouseCarousel(Pose2d pose)
    {
        blueParkInWarehouseCarousel = drive.trajectorySequenceBuilder(pose)
                // Back up more from shipping hub
                .lineToLinearHeading(new Pose2d(40, -16, Math.toRadians(90)))
                // Move to park in storage unit
                .lineToLinearHeading(new Pose2d(29.5, -21, Math.toRadians(90)))
               .build();
    }

    public void BlueParkInStorageFromCarousel(Pose2d pose)
    {
        blueParkInStorageFromCarousel = drive.trajectorySequenceBuilder(pose)
                // Back up more from shipping hub
                .lineToLinearHeading(new Pose2d(40, -12, Math.toRadians(-90)))
                // Move and park in warehouse
                .lineToLinearHeading(new Pose2d(-1, -12, Math.toRadians(-90)))
                .waitSeconds(1.5)
                .lineToLinearHeading(new Pose2d(-3, 84, Math.toRadians(-90)))
               .build();
    }

    public void RedRightDuckBlock(Pose2d pose)
    {
        redRightDuckBlock = drive.trajectorySequenceBuilder(pose)
                // Grab Capstone
                // Arm to top position
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armTopAuto())
                .lineToLinearHeading(new Pose2d(40, -6.25, Math.toRadians(90)))
                .build();
    }

    public void RedWarehouseRightBlockDrop(Pose2d pose)
    {
        redWarehouseRightBlockDrop = drive.trajectorySequenceBuilder(pose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armTopAuto())
                        .lineToLinearHeading(
                        new Pose2d(12, 17, Math.toRadians(-180)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(19, 17, Math.toRadians(-180)))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpen())
                .waitSeconds(1.0)
                .lineToLinearHeading(new Pose2d(12, 17, Math.toRadians(-180)))
                .build();
    }

    public void RedWarehouseMiddleBlockDrop(Pose2d pose)
    {
        redWarehouseMiddleBlockDrop = drive.trajectorySequenceBuilder(pose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armMid())
                .lineToLinearHeading(
                        new Pose2d(12, 17, Math.toRadians(-180)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(16, 17, Math.toRadians(-180)))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpen())
                .waitSeconds(1.0)
                .lineToLinearHeading(new Pose2d(12, 17, Math.toRadians(-180)))
                .build();
    }

    public void RedWarehouseLeftBlockDrop(Pose2d pose)
    {
        redWarehouseLeftBlockDrop = drive.trajectorySequenceBuilder(pose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armBottomAuto())
                .lineToLinearHeading(
                        new Pose2d(12, 17, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(22, 17, Math.toRadians(0)))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpen())
                .waitSeconds(1.0)
                .lineToLinearHeading(new Pose2d(12, 17, Math.toRadians(0)))
                .build();
    }

    public void RedParkInWarehouseFromWarehouse(Pose2d pose)
    {
        redParkInWarehouseFromWarehouse = drive.trajectorySequenceBuilder(pose)
                .waitSeconds(1.0)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armCollect())
                .waitSeconds(3.0)
                .lineToLinearHeading(new Pose2d(-2, 9, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(-2, -28, Math.toRadians(-90)))
                .build();
    }

    public void BlueWarehouseRightBlockDrop(Pose2d pose)
    {
        blueWarehouseRightBlockDrop = drive.trajectorySequenceBuilder(pose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armTopAuto())
                .lineToLinearHeading(
                        new Pose2d(12, -17, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(19, -17, Math.toRadians(180)))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpen())
                .waitSeconds(1.0)
                .lineToLinearHeading(new Pose2d(12, -17, Math.toRadians(180)))
                .build();
    }

    public void BlueWarehouseMiddleBlockDrop(Pose2d pose)
    {
        blueWarehouseMiddleBlockDrop = drive.trajectorySequenceBuilder(pose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armMid())
                .lineToLinearHeading(
                        new Pose2d(12, -17, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(16, -17, Math.toRadians(180)))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpen())
                .waitSeconds(1.0)
                .lineToLinearHeading(new Pose2d(12, -17, Math.toRadians(180)))
                .build();
    }

    public void BlueWarehouseLeftBlockDrop(Pose2d pose)
    {
        blueWarehouseLeftBlockDrop = drive.trajectorySequenceBuilder(pose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armBottomAuto())
                .lineToLinearHeading(
                        new Pose2d(12, -17, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(22, -17, Math.toRadians(0)))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpen())
                .waitSeconds(1.0)
                .lineToLinearHeading(new Pose2d(12, -17, Math.toRadians(0)))
                .build();
    }

    public void BlueParkInWarehouseFromWarehouse(Pose2d pose)
    {
        blueParkInWarehouseFromWarehouse = drive.trajectorySequenceBuilder(pose)
                .waitSeconds(1.0)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armCollect())
                .waitSeconds(3.0)
                .lineToLinearHeading(new Pose2d(-2, -9, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-2, 28, Math.toRadians(90)))
                .build();
    }

    public void BlueWarehouseShift(Pose2d pose)
    {
        blueWarehouseShift = drive.trajectoryBuilder(pose)
                .lineToLinearHeading(new Pose2d(20, 28, Math.toRadians(90)))
                .build();
    }

    public void RedWarehouseShift(Pose2d pose)
    {
        redWarehouseShift = drive.trajectoryBuilder(pose)
                .lineToLinearHeading(new Pose2d(20, -28, Math.toRadians(-90)))
                .build();
    }

    public void RedDuckStorageUnitRight(Pose2d pose)
    {
        redDuckStorageUnitRight = drive.trajectorySequenceBuilder(pose)
                // Grab Capstone
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armBottomAuto())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.capstoneArmCollect())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.wristOpen())
                .lineToLinearHeading(new Pose2d(6, 0, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(6, -18, Math.toRadians(178)))
                .waitSeconds(0.2)
                .lineToLinearHeading(new Pose2d(11, -18, Math.toRadians(178)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.wristClose())
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armTopAuto())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.capstoneShoulderCarry())
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> extras.capstoneElbowCarry())

                // Drop off freight
                .lineToLinearHeading(new Pose2d(30, 0, Math.toRadians(135)))
                // Move closer to shipping hub
                .lineToLinearHeading(new Pose2d(39, -8, Math.toRadians(90))) // Y was -6.3 on 2/28
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpenDuck())
                .waitSeconds(0.3)
                // Arm to collect position
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armCollect())

                // GRAB DUCK
                .lineToLinearHeading(new Pose2d(10, 19, Math.toRadians(90)))
                .lineToLinearHeading(
                        new Pose2d(4.5, 19, Math.toRadians(90)), // X WAS 6.5 ON 3/5
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                // Spin the carousel
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.leftWheelCCW())
                .waitSeconds(2.7)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.leftWheelOff())

                // Duck Path
                .lineToLinearHeading(new Pose2d(8, -8, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(8, -4, Math.toRadians(120)))
                .lineToLinearHeading(new Pose2d(0.8, -5, Math.toRadians(130)))
                .lineToLinearHeading(new Pose2d(1.5, 17, Math.toRadians(130)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawClose())
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> extras.armTopAuto())
                .lineToLinearHeading(new Pose2d(41, -8, Math.toRadians(90))) // Y was -6.4 2/28
                .waitSeconds(0.4)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpen())
                .waitSeconds(0.3)
                .build();
    }
    public void RedRightStoragePark(Pose2d pose)
    {
        redDuckPark = drive.trajectorySequenceBuilder(pose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armCollect())
                .lineToLinearHeading(new Pose2d(25, 22.5, Math.toRadians(-90)))
                .build();
    }
    public void RedRightWarehouseParkNear(Pose2d pose)
    {
        redDuckPark = drive.trajectorySequenceBuilder(pose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armBottomAuto())
                .lineToLinearHeading(new Pose2d(-3, -7, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-4, -83, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armCollect())
                .build();
    }
    public void RedRightWarehouseParkFar(Pose2d pose)
    {
        redDuckPark = drive.trajectorySequenceBuilder(pose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armBottomAuto())
                .lineToLinearHeading(new Pose2d(15, -7, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(19, -90, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armCollect())
                .turn(Math.toRadians(-180))
                .build();
    }


    public void RedDuckStorageUnitMiddle(Pose2d pose)
    {
        redDuckStorageUnitMiddle = drive.trajectorySequenceBuilder(pose)
                // Grab Capstone
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armBottomAuto())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.capstoneArmCollect())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.wristOpen())
                .lineToLinearHeading(new Pose2d(6, 0, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(6, -10, Math.toRadians(178)))
                .waitSeconds(0.3)
                .lineToLinearHeading(new Pose2d(10, -10, Math.toRadians(178)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.wristClose())
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armMid())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.capstoneShoulderCarry())
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> extras.capstoneElbowCarry())

                // Drop off freight
                .lineToLinearHeading(new Pose2d(30, 0, Math.toRadians(135)))
                .lineToLinearHeading(new Pose2d(37, -2, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(37, -5.5, Math.toRadians(90)))
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpenDuck())
                .waitSeconds(0.3)
                // Back up from shipping hub
                .lineToLinearHeading(new Pose2d(37, 3, Math.toRadians(90)))
                // Arm to collect position
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armCollect())
                .waitSeconds(0.8)

                // GRAB DUCK
                .lineToLinearHeading(new Pose2d(9, 19, Math.toRadians(90)))
                .lineToLinearHeading(
                        new Pose2d(4.5, 19, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                // Spin the carousel
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.leftWheelCCW())
                .waitSeconds(2.7)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.leftWheelOff())

                // Duck Path
                .lineToLinearHeading(new Pose2d(8, -8, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(8, -4, Math.toRadians(120)))
                .lineToLinearHeading(new Pose2d(0, -5, Math.toRadians(130)))
                .lineToLinearHeading(new Pose2d(1, 16, Math.toRadians(130)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawClose())
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> extras.armTopAuto())
                .lineToLinearHeading(new Pose2d(40, -6.3, Math.toRadians(90)))
                .waitSeconds(0.4)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpen())
                .waitSeconds(0.3)
                .build();
    }
    public void RedMiddleStoragePark(Pose2d pose)
    {
        redDuckPark = drive.trajectorySequenceBuilder(pose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armCollect())
                .lineToLinearHeading(new Pose2d(25, 24, Math.toRadians(-90)))
                .build();
    }
    public void RedMiddleWarehouseParkNear(Pose2d pose)
    {
        redDuckPark = drive.trajectorySequenceBuilder(pose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armBottomAuto())
                .lineToLinearHeading(new Pose2d(-2, -7, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-3, -83, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armCollect())
                .build();
    }
    public void RedMiddleWarehouseParkFar(Pose2d pose)
    {
        redDuckPark = drive.trajectorySequenceBuilder(pose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armBottomAuto())
                .lineToLinearHeading(new Pose2d(14, -7, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(18, -90, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armCollect())
                .turn(Math.toRadians(-180))
                .build();
    }
    public void RedDuckStorageUnitLeft(Pose2d pose)
    {
        redDuckStorageUnitLeft = drive.trajectorySequenceBuilder(pose)
                // Grab Capstone
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armBottomAuto())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.capstoneArmCollect())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.wristOpen())
                .lineToLinearHeading(new Pose2d(6, 0, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(7, 0, Math.toRadians(178)))
                .waitSeconds(0.3)
                .lineToLinearHeading(new Pose2d(11, 0, Math.toRadians(178)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.wristClose())
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armBottomAuto())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.capstoneShoulderCarry())
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> extras.capstoneElbowCarry())

                // Drop off freight
                .lineToLinearHeading(new Pose2d(40, -10, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(40, -13, Math.toRadians(-90)))
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpenDuck())
                .waitSeconds(0.2)
                // Back up from shipping hub
                .lineToLinearHeading(new Pose2d(40, -6, Math.toRadians(-90)))
                // Arm to collect position
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armCollect())

                // GRAB DUCK
                .lineToLinearHeading(new Pose2d(10, 16.5, Math.toRadians(90)))
                .lineToLinearHeading(
                        new Pose2d(6, 16.5, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                // Spin the carousel
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.leftWheelCCW())
                .waitSeconds(2.7)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.leftWheelOff())

                // Duck Path
                .lineToLinearHeading(new Pose2d(8, -8, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(8, -4, Math.toRadians(120)))
                .lineToLinearHeading(new Pose2d(1.35, -5, Math.toRadians(130)))
                .lineToLinearHeading(new Pose2d(1.95, 16, Math.toRadians(130)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawClose())
                .waitSeconds(0.4)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> extras.armTopAuto())
                .lineToLinearHeading(new Pose2d(41, -9.1, Math.toRadians(90)))
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpen())
                .waitSeconds(0.25)
                .build();
    }
    public void RedLeftStoragePark(Pose2d pose)
    {
        redDuckPark = drive.trajectorySequenceBuilder(pose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armCollect())
                .lineToLinearHeading(new Pose2d(27, 22.5, Math.toRadians(-90)))
                .waitSeconds(1.0)
                .build();
    }
    public void RedLeftWarehouseParkNear(Pose2d pose)
    {
        redDuckPark = drive.trajectorySequenceBuilder(pose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armBottomAuto())
                .lineToLinearHeading(new Pose2d(-3, -7, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-4, -87, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armCollect())
                .build();
    }
    public void RedLeftWarehouseParkFar(Pose2d pose)
    {
        redDuckPark = drive.trajectorySequenceBuilder(pose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armBottomAuto())
                .lineToLinearHeading(new Pose2d(14, -7, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(20, -90, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armCollect())
                .turn(Math.toRadians(-180))
                .build();
    }
    public void BlueDuckStorageUnitRight(Pose2d pose)
    {
        blueDuckStorageUnitRight = drive.trajectorySequenceBuilder(pose)
                // Grab Capstone
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armBottomAuto())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.capstoneArmCollect())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.wristOpen())
                .lineToLinearHeading(new Pose2d(6, 0, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(6, -8.5, Math.toRadians(-178)))
                .waitSeconds(0.2)
                .lineToLinearHeading(new Pose2d(11, -8.5, Math.toRadians(-178)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.wristClose())
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armTopAuto())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.capstoneShoulderCarry())
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> extras.capstoneElbowCarry())

                // Drop off freight
                .lineToLinearHeading(new Pose2d(30, -3, Math.toRadians(-135)))
                // Move closer to shipping hub
                .lineToLinearHeading(new Pose2d(40, 7, Math.toRadians(-90))) // Y was 6 2/28
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpenDuck())
                .waitSeconds(0.3)
                // Arm to collect position
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armCollect())

                // GRAB DUCK
                .lineToLinearHeading(new Pose2d(10, -19, Math.toRadians(-90)))
                .lineToLinearHeading(
                        new Pose2d(3, -19, Math.toRadians(-90)), /// X was 5.8 on 3/5
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                // Spin the carousel
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.rightWheelCCW())
                .waitSeconds(2.7)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.rightWheelOff())

                // Duck Path
                .lineToLinearHeading(new Pose2d(8, 6, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(8, 4, Math.toRadians(-120)))
                .lineToLinearHeading(new Pose2d(1.6, 5, Math.toRadians(-130)))
                .lineToLinearHeading(new Pose2d(3.0, -19, Math.toRadians(-130)), // X was 2.5 on 2/28
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawClose())
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> extras.armTopAuto())
                .lineToLinearHeading(new Pose2d(41, 6.8, Math.toRadians(-90)))
                .waitSeconds(0.4)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpen())
                .waitSeconds(0.3)
                .build();
    }
    public void BlueRightStoragePark(Pose2d pose)
    {
        blueDuckPark = drive.trajectorySequenceBuilder(pose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armCollect())
                .lineToLinearHeading(new Pose2d(28, -25, Math.toRadians(90))) // X was 25 on 2/28
                .waitSeconds(1.0)
                .build();
    }
    public void BlueRightWarehouseParkNear(Pose2d pose)
    {
        blueDuckPark = drive.trajectorySequenceBuilder(pose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armBottomAuto())
                .lineToLinearHeading(new Pose2d(-3, 7, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(-4, 80, Math.toRadians(-90))) // Y was 87 on 3/1
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armCollect())
                .build();
    }
    public void BlueRightWarehouseParkFar(Pose2d pose)
    {
        blueDuckPark = drive.trajectorySequenceBuilder(pose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armBottomAuto())
                .lineToLinearHeading(new Pose2d(14, 7, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(18, 90, Math.toRadians(-90))) // x was 18 on 2/28
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armCollect())
                .turn(Math.toRadians(180))
                .build();
    }

    public void BlueDuckStorageUnitMiddle(Pose2d pose)
    {
        blueDuckStorageUnitMiddle = drive.trajectorySequenceBuilder(pose)
                // Grab Capstone
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armBottomAuto())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.capstoneArmCollect())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.wristOpen())
                .lineToLinearHeading(new Pose2d(6, 0, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(6, 1, Math.toRadians(-178)))
                .waitSeconds(0.3)
                .lineToLinearHeading(new Pose2d(10, 1, Math.toRadians(-178)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.wristClose())
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armMid())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.capstoneShoulderCarry())
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> extras.capstoneElbowCarry())

                // Drop off freight
                .lineToLinearHeading(new Pose2d(30, 0, Math.toRadians(-135)))
                .lineToLinearHeading(new Pose2d(38, 2, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(38, 5.5, Math.toRadians(-90)))
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpenDuck())
                .waitSeconds(0.3)
                // Back up from shipping hub
                .lineToLinearHeading(new Pose2d(37, -3, Math.toRadians(-90)))
                // Arm to collect position
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armCollect())
                .waitSeconds(0.8)

                // GO TO CAROUSEL
                .lineToLinearHeading(new Pose2d(11, -19, Math.toRadians(-90)))
                .lineToLinearHeading(
                        new Pose2d(3, -21, Math.toRadians(-90)), // X WAS 5.8 on 3/5, Y WAS -19
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                // Spin the carousel
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.rightWheelCCW())
                .waitSeconds(2.7)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.rightWheelOff())

                // GRAB THE DUCK
                .lineToLinearHeading(new Pose2d(8, 8, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(8, 4, Math.toRadians(-120)))
                .lineToLinearHeading(new Pose2d(1, 5, Math.toRadians(-130)))
                .lineToLinearHeading(new Pose2d(2, -16, Math.toRadians(-130)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawClose())
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> extras.armTopAuto())
                .lineToLinearHeading(new Pose2d(39, 6.3, Math.toRadians(-90))) // x was 40
                .waitSeconds(0.4)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpen())
                .waitSeconds(0.3)
                .build();
    }

    public void BlueMiddleStoragePark(Pose2d pose)
    {
        blueDuckPark = drive.trajectorySequenceBuilder(pose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armCollect())
                .lineToLinearHeading(new Pose2d(27, -24, Math.toRadians(90)))
                .waitSeconds(1.0)
                .build();
    }
    public void BlueMiddleWarehouseParkNear(Pose2d pose)
    {
        blueDuckPark = drive.trajectorySequenceBuilder(pose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armBottomAuto())
                .lineToLinearHeading(new Pose2d(-4, 7, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(-5, 80, Math.toRadians(-90))) // Y was 87 on 3/1
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armCollect())
                .build();
    }
    public void BlueMiddleWarehouseParkFar(Pose2d pose)
    {
        blueDuckPark = drive.trajectorySequenceBuilder(pose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armBottomAuto())
                .lineToLinearHeading(new Pose2d(14, 7, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(18, 90, Math.toRadians(-90)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armCollect())
                .turn(Math.toRadians(180))
                .build();
    }

    public void BlueDuckStorageUnitLeft(Pose2d pose)
    {
        blueDuckStorageUnitLeft = drive.trajectorySequenceBuilder(pose)
                // Grab Capstone
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armBottomAuto())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.capstoneArmCollect())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.wristOpen())
                .lineToLinearHeading(new Pose2d(6, 0, Math.toRadians(0))) // new traj on 3/1
                .lineToLinearHeading(new Pose2d(6, 10, Math.toRadians(-178)))
                .waitSeconds(0.8)
                .lineToLinearHeading(new Pose2d(11, 10, Math.toRadians(-178)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.wristClose())
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armBottomAuto())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.capstoneShoulderCarry())
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> extras.capstoneElbowCarry())

                // Drop off freight
                .lineToLinearHeading(new Pose2d(38.5, 9, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(38, 12.6, Math.toRadians(90))) // X was 38.5 and Y was 13 on 3/1
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpenDuck())
                .waitSeconds(0.3)
                // Back up from shipping hub
                .lineToLinearHeading(new Pose2d(40, 6, Math.toRadians(90)))
                // Arm to collect position
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armCollect())

                // GO TO CAROUSEL
                .lineToLinearHeading(new Pose2d(11, -17.5, Math.toRadians(-90))) // Y WAS -16.5 ON 3/5
                .lineToLinearHeading(
                        new Pose2d(5.8, -17.5, Math.toRadians(-90)), // x was 4.16 (?) on 3/1, Y WAS -16.5
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                // Spin the carousel
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.rightWheelCCW())
                .waitSeconds(2.7)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.rightWheelOff())

                // Duck Path
                .lineToLinearHeading(new Pose2d(8, 8, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(8, 4, Math.toRadians(-120)))
                .lineToLinearHeading(new Pose2d(1, 5, Math.toRadians(-130)))
                .lineToLinearHeading(new Pose2d(2, -16, Math.toRadians(-130)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawClose())
                .waitSeconds(1.0)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> extras.armTopAuto())
                .lineToLinearHeading(new Pose2d(38, 9, Math.toRadians(-90))) // was (39, 9) on 3/1
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpen())
                .waitSeconds(0.5)
                .build();
    }
    public void BlueLeftStoragePark(Pose2d pose)
    {
        blueDuckPark = drive.trajectorySequenceBuilder(pose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armCollect())
                .lineToLinearHeading(new Pose2d(25, -22.5, Math.toRadians(90)))
                .waitSeconds(1.0)
                .build();
    }
    public void BlueLeftWarehouseParkNear(Pose2d pose)
    {
        blueDuckPark = drive.trajectorySequenceBuilder(pose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armBottomAuto())
                .lineToLinearHeading(new Pose2d(-3, 7, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(-4, 80, Math.toRadians(-90)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armCollect())
                .build();
    }
    public void BlueLeftWarehouseParkFar(Pose2d pose)
    {
        blueDuckPark = drive.trajectorySequenceBuilder(pose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armBottomAuto())
                .lineToLinearHeading(new Pose2d(14, 7, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(14, 90, Math.toRadians(-90)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armCollect())
                .turn(Math.toRadians(180))
                .build();
    }


    public void RedWarehouseRight(Pose2d pose)
    {
        redCapstoneWarehouse = drive.trajectorySequenceBuilder(pose)
                // Grab Capstone
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armBottomAuto())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.capstoneArmCollect())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.wristOpen())
                .lineToLinearHeading(new Pose2d(6, 0, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(25, 10, Math.toRadians(90)))
                .waitSeconds(0.2)
                .lineToLinearHeading(new Pose2d(25, 4.5, Math.toRadians(90)))
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.wristClose())
                .waitSeconds(0.2)
                .lineToLinearHeading(new Pose2d(25, 6, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.capstoneArmCarry())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armTopAuto())
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(25, -2, Math.toRadians(-89)))
                .lineToLinearHeading(new Pose2d(42.5,0 , Math.toRadians(-90))) // y was -2 on 3/3
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpen())
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armCollect())
                .lineToLinearHeading(new Pose2d(0,-3 , Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(0,-32 , Math.toRadians(-90)))
                .build();
    }
    public void RedWarehouseMiddle(Pose2d pose)
    {
        redCapstoneWarehouse = drive.trajectorySequenceBuilder(pose)
                // Grab Capstone
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armBottomAuto())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.capstoneArmCollect())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.wristOpen())
                .lineToLinearHeading(new Pose2d(6, 0, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(6, 5, Math.toRadians(135)))
                .waitSeconds(0.2)
                .lineToLinearHeading(new Pose2d(10, 10, Math.toRadians(135)))
                .lineToLinearHeading(new Pose2d(14, 6, Math.toRadians(135)))
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.wristClose())
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armMid())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.capstoneShoulderCarry())
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> extras.capstoneElbowCarry())
                .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(24, 3, Math.toRadians(-135))) // was (23, 2) on 3/3
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpen())
                .waitSeconds(0.2)
                .lineToLinearHeading(new Pose2d(-5,-3 , Math.toRadians(-90)))
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armCollect())
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(-5,-29 , Math.toRadians(-90)))
                .build();

    }
    public void RedWarehouseLeft(Pose2d pose)
    {
        redCapstoneWarehouse = drive.trajectorySequenceBuilder(pose)
                // Grab Capstone
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armBottomAuto())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.capstoneArmCollect())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.wristOpen())
                .lineToLinearHeading(new Pose2d(6, 0, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(7, -1, Math.toRadians(178))) // Y was 0 on 3/3
                .waitSeconds(0.3)
                .lineToLinearHeading(new Pose2d(11, 0, Math.toRadians(178)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.wristClose())
                .waitSeconds(0.3)

                // Move to Shipping Hub and drop game piece
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.capstoneArmCarry())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armBottomAuto())
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(41, 0, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(41, 2, Math.toRadians(90))) // new movement on 3/3
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpen())
                .waitSeconds(0.2)

                // Park in Warehouse
                .lineToLinearHeading(new Pose2d(41, -6, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armCollect())
                .lineToLinearHeading(new Pose2d(-1, -4, Math.toRadians(-90)))
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(-5,-32 , Math.toRadians(-90))) // line added on 3/3

                .build();

    }

    public void RedWarehouseRightShift(Pose2d pose)
    {
        redCapstoneWarehouseShift = drive.trajectorySequenceBuilder(pose)
                // Shift over
                .lineToLinearHeading(new Pose2d(28, -29, Math.toRadians(-135)))
                .build();
    }

    public void RedWarehouseMiddleShift(Pose2d pose)
    {
        redCapstoneWarehouseShift = drive.trajectorySequenceBuilder(pose)
                // Shift over
                .lineToLinearHeading(new Pose2d(23, -29, Math.toRadians(-135)))
                .build();
    }

    public void RedWarehouseLeftShift(Pose2d pose)
    {
        redCapstoneWarehouseShift = drive.trajectorySequenceBuilder(pose)
                // Shift over
                .lineToLinearHeading(new Pose2d(23, -29, Math.toRadians(-135)))
                .build();
    }

    public void BlueWarehouseLeft(Pose2d pose)
    {
        blueCapstoneWarehouse = drive.trajectorySequenceBuilder(pose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.capstoneArmCollect())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.wristOpen())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armBottomAuto())
                .lineToLinearHeading(new Pose2d(6, 0, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(25, -15, Math.toRadians(-105)))
                .lineToLinearHeading(new Pose2d(28, -6.5, Math.toRadians(-105)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.wristClose())
                .waitSeconds(0.2)
                .lineToLinearHeading(new Pose2d(27, -8, Math.toRadians(-105)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.capstoneShoulderCarry())
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> extras.capstoneElbowCarry())
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(24, 0, Math.toRadians(-45)))
                .lineToLinearHeading(new Pose2d(30, -6, Math.toRadians(-45)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpen())

                // PARK IN WAREHOUSE
                .lineToLinearHeading(new Pose2d(24, 0, Math.toRadians(-45)))
                .lineToLinearHeading(new Pose2d(-1, 1, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-1, 29, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armCollect())
                .waitSeconds(1)
                .build();
    }

    public void BlueWarehouseMiddle(Pose2d pose)
    {
        blueCapstoneWarehouse = drive.trajectorySequenceBuilder(pose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armBottomAuto())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.capstoneArmCollect())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.wristOpen())
                .lineToLinearHeading(new Pose2d(6, 0, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(6, 1, Math.toRadians(-178)))
                .waitSeconds(0.3)
                .lineToLinearHeading(new Pose2d(10, 1, Math.toRadians(-178)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.wristClose())
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armMid())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.capstoneShoulderCarry())
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> extras.capstoneElbowCarry())
                .waitSeconds(1.5)

                // DROP GAME ELEMENT
                .lineToLinearHeading(new Pose2d(24, 0, Math.toRadians(135)))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpen())
                .waitSeconds(0.2)
                .lineToLinearHeading(new Pose2d(22, 2, Math.toRadians(135)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armCollect())

                .lineToLinearHeading(new Pose2d(-3, 1, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-3, 29, Math.toRadians(90)))
                .waitSeconds(1)

                .build();
    }


    public void BlueWarehouseRight(Pose2d pose)
    {
        blueCapstoneWarehouse = drive.trajectorySequenceBuilder(pose)
                // Grab Capstone
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armBottomAuto())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.capstoneArmCollect())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.wristOpen())
                .lineToLinearHeading(new Pose2d(6, 0, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(6, -8.5, Math.toRadians(-178)))
                .waitSeconds(0.2)
                .lineToLinearHeading(new Pose2d(12, -8.5, Math.toRadians(-178)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.wristClose())
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armTopAuto())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.capstoneShoulderCarry())
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> extras.capstoneElbowCarry())
                .waitSeconds(1.5)

                // DROP GAME ELEMENT
                .lineToLinearHeading(new Pose2d(27, -3, Math.toRadians(135)))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpen())
                .waitSeconds(0.2)
                .lineToLinearHeading(new Pose2d(22, 2, Math.toRadians(135)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.armCollect())

                // PARK IN WAREHOUSE
                .lineToLinearHeading(new Pose2d(-3, 1, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-3, 29, Math.toRadians(90)))
                .waitSeconds(1)
                .build();
    }

    public void BlueWarehouseLeftShift(Pose2d pose)
    {
        blueCapstoneWarehouseShift = drive.trajectorySequenceBuilder(pose)
                // Shift over
                .lineToLinearHeading(new Pose2d(27, 29, Math.toRadians(135)))
                .build();
    }

    public void BlueWarehouseMiddleShift(Pose2d pose)
    {
        blueCapstoneWarehouseShift = drive.trajectorySequenceBuilder(pose)
                .lineToLinearHeading(new Pose2d(25, 29, Math.toRadians(135)))
                .build();
    }

    public void BlueWarehouseRightShift(Pose2d pose)
    {
        blueCapstoneWarehouseShift = drive.trajectorySequenceBuilder(pose)
                // Shift over
                .lineToLinearHeading(new Pose2d(25, 29, Math.toRadians(135)))
                .build();
    }
}
