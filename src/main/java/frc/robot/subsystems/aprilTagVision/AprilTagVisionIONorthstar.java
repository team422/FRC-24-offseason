// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.aprilTagVision;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import frc.robot.Constants.FieldConstants;

public class AprilTagVisionIONorthstar implements AprilTagVisionIO {
  private static final int cameraResolutionWidth = 1280;
  private static final int cameraResolutionHeight = 800;
  private static final int cameraAutoExposure = 1;
  private static final int cameraExposure = 30;
  private static final int cameraGain = 50;
  private static final int cameraBrightness = 0;
  private static final int cameraContrast = 32;
  private static final int cameraGamma = 100;

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
    configTable.getIntegerTopic("camera_brightness").publish().set(cameraBrightness);
    configTable.getIntegerTopic("camera_contrast").publish().set(cameraContrast);
    configTable.getIntegerTopic("camera_gamma").publish().set(cameraGamma);
    configTable.getDoubleTopic("fiducial_size_m").publish().set(FieldConstants.kAprilTagWidth);
    try {
      configTable
          .getStringTopic("tag_layout")
          .publish()
          // .set(new ObjectMapper().writeValueAsString(FieldConstants.kAprilTagLayout));
          .set(
              "{\"ID\":1,\"pose\":{\"translation\":{\"x\":16.697198,\"y\":0.65532,\"z\":1.4859},\"rotation\":{\"quaternion\":{\"W\":0.45399049973954686,\"X\":0.0,\"Y\":0.0,\"Z\":0.8910065241883679}}}},{\"ID\":2,\"pose\":{\"translation\":{\"x\":16.697198,\"y\":7.3964799999999995,\"z\":1.4859},\"rotation\":{\"quaternion\":{\"W\":-0.45399049973954675,\"X\":-0.0,\"Y\":0.0,\"Z\":0.8910065241883679}}}},{\"ID\":3,\"pose\":{\"translation\":{\"x\":11.560809999999998,\"y\":8.05561,\"z\":1.30175},\"rotation\":{\"quaternion\":{\"W\":-0.7071067811865475,\"X\":-0.0,\"Y\":0.0,\"Z\":0.7071067811865476}}}},{\"ID\":4,\"pose\":{\"translation\":{\"x\":9.276079999999999,\"y\":6.137656,\"z\":1.8679160000000001},\"rotation\":{\"quaternion\":{\"W\":0.9659258262890683,\"X\":0.0,\"Y\":0.25881904510252074,\"Z\":0.0}}}},{\"ID\":5,\"pose\":{\"translation\":{\"x\":9.276079999999999,\"y\":1.914906,\"z\":1.8679160000000001},\"rotation\":{\"quaternion\":{\"W\":0.9659258262890683,\"X\":0.0,\"Y\":0.25881904510252074,\"Z\":0.0}}}},{\"ID\":6,\"pose\":{\"translation\":{\"x\":13.474446,\"y\":3.3063179999999996,\"z\":0.308102},\"rotation\":{\"quaternion\":{\"W\":-0.8660254037844387,\"X\":-0.0,\"Y\":0.0,\"Z\":0.49999999999999994}}}},{\"ID\":7,\"pose\":{\"translation\":{\"x\":13.890498,\"y\":4.0259,\"z\":0.2125},\"rotation\":{\"quaternion\":{\"W\":1.0,\"X\":0.0,\"Y\":0.0,\"Z\":0.0}}}},{\"ID\":8,\"pose\":{\"translation\":{\"x\":13.474446,\"y\":4.745482,\"z\":0.308102},\"rotation\":{\"quaternion\":{\"W\":0.8660254037844387,\"X\":0.0,\"Y\":0.0,\"Z\":0.49999999999999994}}}},{\"ID\":9,\"pose\":{\"translation\":{\"x\":12.643358,\"y\":4.745482,\"z\":0.308102},\"rotation\":{\"quaternion\":{\"W\":0.5000000000000001,\"X\":0.0,\"Y\":0.0,\"Z\":0.8660254037844386}}}},{\"ID\":10,\"pose\":{\"translation\":{\"x\":12.227305999999999,\"y\":4.0259,\"z\":0.308102},\"rotation\":{\"quaternion\":{\"W\":6.123233995736766E-17,\"X\":0.0,\"Y\":0.0,\"Z\":1.0}}}},{\"ID\":11,\"pose\":{\"translation\":{\"x\":12.643358,\"y\":3.3063179999999996,\"z\":0.308102},\"rotation\":{\"quaternion\":{\"W\":-0.49999999999999983,\"X\":-0.0,\"Y\":0.0,\"Z\":0.8660254037844388}}}},{\"ID\":12,\"pose\":{\"translation\":{\"x\":0.851154,\"y\":0.65532,\"z\":1.4859},\"rotation\":{\"quaternion\":{\"W\":0.8910065241883679,\"X\":0.0,\"Y\":0.0,\"Z\":0.45399049973954675}}}},{\"ID\":13,\"pose\":{\"translation\":{\"x\":0.851154,\"y\":7.3964799999999995,\"z\":1.4859},\"rotation\":{\"quaternion\":{\"W\":-0.8910065241883678,\"X\":-0.0,\"Y\":0.0,\"Z\":0.45399049973954686}}}},{\"ID\":14,\"pose\":{\"translation\":{\"x\":8.272272,\"y\":6.137656,\"z\":1.8679160000000001},\"rotation\":{\"quaternion\":{\"W\":5.914589856893349E-17,\"X\":-0.25881904510252074,\"Y\":1.5848095757158825E-17,\"Z\":0.9659258262890683}}}},{\"ID\":15,\"pose\":{\"translation\":{\"x\":8.272272,\"y\":1.914906,\"z\":1.8679160000000001},\"rotation\":{\"quaternion\":{\"W\":5.914589856893349E-17,\"X\":-0.25881904510252074,\"Y\":1.5848095757158825E-17,\"Z\":0.9659258262890683}}}},{\"ID\":16,\"pose\":{\"translation\":{\"x\":5.9875419999999995,\"y\":-0.0038099999999999996,\"z\":1.30175},\"rotation\":{\"quaternion\":{\"W\":0.7071067811865476,\"X\":0.0,\"Y\":0.0,\"Z\":0.7071067811865476}}}},{\"ID\":17,\"pose\":{\"translation\":{\"x\":4.073905999999999,\"y\":3.3063179999999996,\"z\":0.308102},\"rotation\":{\"quaternion\":{\"W\":-0.49999999999999983,\"X\":-0.0,\"Y\":0.0,\"Z\":0.8660254037844388}}}},{\"ID\":18,\"pose\":{\"translation\":{\"x\":3.6576,\"y\":4.0259,\"z\":0.308102},\"rotation\":{\"quaternion\":{\"W\":6.123233995736766E-17,\"X\":0.0,\"Y\":0.0,\"Z\":1.0}}}},{\"ID\":19,\"pose\":{\"translation\":{\"x\":4.073905999999999,\"y\":4.745482,\"z\":0.308102},\"rotation\":{\"quaternion\":{\"W\":0.5000000000000001,\"X\":0.0,\"Y\":0.0,\"Z\":0.8660254037844386}}}},{\"ID\":20,\"pose\":{\"translation\":{\"x\":4.904739999999999,\"y\":4.745482,\"z\":0.308102},\"rotation\":{\"quaternion\":{\"W\":0.8660254037844387,\"X\":0.0,\"Y\":0.0,\"Z\":0.49999999999999994}}}}");
    } catch (Exception e) {
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
