// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.aprilTagVision;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import frc.robot.Constants.FieldConstants;

public class AprilTagVisionIONorthstar implements AprilTagVisionIO {
  private static final int cameraResolutionWidth = 1600;
  private static final int cameraResolutionHeight = 1200;
  private static final int cameraAutoExposure = 1;
  private static final int cameraExposure = 40;
  private static final int cameraGain = 25;

  private final DoubleArraySubscriber observationSubscriber;
  private final DoubleArraySubscriber demoObservationSubscriber;
  private final IntegerSubscriber fpsSubscriber;

  public AprilTagVisionIONorthstar(String instanceId, String cameraId) {
    var northstarTable = NetworkTableInstance.getDefault().getTable(instanceId);

    var configTable = northstarTable.getSubTable("config");
    configTable.getStringTopic("camera_id").publish().set(cameraId);
    configTable.getIntegerTopic("camera_resolution_width").publish().set(cameraResolutionWidth);
    configTable.getIntegerTopic("camera_resolution_height").publish().set(cameraResolutionHeight);
    configTable.getIntegerTopic("camera_auto_exposure").publish().set(cameraAutoExposure);
    configTable.getIntegerTopic("camera_exposure").publish().set(cameraExposure);
    configTable.getIntegerTopic("camera_gain").publish().set(cameraGain);
    configTable.getDoubleTopic("fiducial_size_m").publish().set(FieldConstants.kAprilTagWidth);
    try {
      configTable
          .getStringTopic("tag_layout")
          .publish()
          .set(new ObjectMapper().writeValueAsString(FieldConstants.kAprilTagLayout));
    } catch (JsonProcessingException e) {
      throw new RuntimeException("Failed to serialize AprilTag layout JSON for Northstar");
    }

    var outputTable = northstarTable.getSubTable("output");
    observationSubscriber =
        outputTable
            .getDoubleArrayTopic("observations")
            .subscribe(
                new double[] {}, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
    demoObservationSubscriber =
        outputTable
            .getDoubleArrayTopic("demo_observations")
            .subscribe(
                new double[] {}, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
    fpsSubscriber = outputTable.getIntegerTopic("fps").subscribe(0);
  }

  public void updateInputs(AprilTagVisionInputs inputs) {
    var queue = observationSubscriber.readQueue();
    inputs.timestamps = new double[queue.length];
    inputs.frames = new double[queue.length][];
    for (int i = 0; i < queue.length; i++) {
      inputs.timestamps[i] = queue[i].timestamp / 1000000.0;
      inputs.frames[i] = queue[i].value;
    }
    inputs.demoFrame = new double[] {};
    for (double[] demoFrame : demoObservationSubscriber.readQueueValues()) {
      inputs.demoFrame = demoFrame;
    }
    inputs.fps = fpsSubscriber.get();
  }
}
