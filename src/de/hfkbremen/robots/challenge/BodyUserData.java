package de.hfkbremen.robots.challenge;

public class BodyUserData {

    public static enum ShapeType {

        TIRE,
        WALL,
        ROBOT,
        SENSOR,
        OBSTACLE_BALL
    };

    public Object reference;
    public final ShapeType shape_type;

    public BodyUserData(Object pReference, ShapeType pShapeType) {
        reference = pReference;
        shape_type = pShapeType;
    }
}
