// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.LoggedTracer;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.util.Arrays;

public class Robot extends LoggedRobot {
    private Command autonomousCommand;
    private RobotContainer robotContainer;

    private static final boolean IS_PRACTICE = true;
    private static final String LOG_DIRECTORY = isSimulation() ? "/logs" : "/home/lvuser/logs";
    private static final long MIN_FREE_SPACE =
          IS_PRACTICE
                  ? 100000000
                  : // 100 MB
                  1000000000; // 1 GB

    @Override
    public void robotInit() {
        setupLogging();

        switch (Constants.ROBOT_MODE) {
            case REAL:
                // Running on a real robot, log to a USB stick ("/U/logs")
                Logger.addDataReceiver(new WPILOGWriter(LOG_DIRECTORY));
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case SIM:
                // Running a physics simulator, log to NT
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case REPLAY:
                // Replaying a log, set up replay source
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;
        }

        Logger.start();

        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        LoggedTracer.reset();

        try {
            Threads.setCurrentThreadPriority(true, 4);
            double commandSchedulerStart = Timer.getTimestamp();
            CommandScheduler.getInstance().run();
            double commandSchedulerEnd = Timer.getTimestamp();
            LoggedTracer.record("Commands");
            SmartDashboard.putNumber(
              "Logged Robot/Loop Cycle Time Milliseconds",
              (commandSchedulerEnd - commandSchedulerStart) * 1000.0);
            Threads.setCurrentThreadPriority(false, 0);
        } catch (Exception e) {
            SmartDashboard.putString("Logged Robot/Latest Error", e.getMessage());
        }
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
          autonomousCommand.schedule();
        }
    }

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
          autonomousCommand.cancel();
        }
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    private void setupLogging() {
        var directory = new File(LOG_DIRECTORY);
        if (!directory.exists()) {
            directory.mkdir();
        }

        if (isSimulation()) return; // Don't delete files while simulating
        if (directory.getFreeSpace() < MIN_FREE_SPACE) {
            System.out.println("ERROR: out of space!");
            var files = directory.listFiles();
            if (files == null) {
                System.out.println("ERROR: Cannot delete, Files are NULL!");
            } else {
                // Sorting the files by name will ensure that the oldest files are deleted first
                files = Arrays.stream(files).sorted().toArray(File[]::new);

                long bytesToDelete = MIN_FREE_SPACE - directory.getFreeSpace();

                for (File file : files) {
                    if (file.getName().endsWith(".wpilog")) {
                        try {
                            bytesToDelete -= Files.size(file.toPath());
                        } catch (IOException e) {
                            System.out.println("Failed to get size of file " + file.getName());
                            continue;
                        }

                        if (file.delete()) {
                            System.out.println("Deleted " + file.getName() + " to free up space");
                        } else {
                            System.out.println("Failed to delete " + file.getName());
                        }

                        if (bytesToDelete <= 0) {
                            break;
                        }
                    }
                }
            }
        }
    }
}
