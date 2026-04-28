package frc.robot.util.Rebuilt.Zones;

import java.util.function.Supplier;

import org.opencv.core.Rect;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RectangleZone implements Zone {
    private final Rectangle2d rectangle;

    /**
     * Creates a rectangular zone with the given top left and bottom right corners.
     * @param topLeft the top left corner of the rectangle
     * @param bottomRight the bottom right corner of the rectangle
     */
    public RectangleZone(Rectangle2d rectangle) {
        this.rectangle = rectangle;
    }

    /**
     * Returns a Trigger that is active when the given translation is within the rectangle.
     * @param translation the translation to check
     * @return a Trigger that is active when the given translation is within the rectangle
     */
    
    @Override
    public Trigger contains(Supplier<Translation2d> translation) {
        return new Trigger(() -> rectangle.contains(translation.get()));
    }
    
}
