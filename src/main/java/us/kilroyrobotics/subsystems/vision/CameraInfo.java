package us.kilroyrobotics.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.simulation.SimCameraProperties;

public record CameraInfo(
    String cameraName, Transform3d transformation, SimCameraProperties simCameraProperties) {}
