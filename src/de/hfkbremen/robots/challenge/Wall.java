package de.hfkbremen.robots.challenge;

import de.hfkbremen.robots.challenge.FUD.ObstacleWallFUD;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.Fixture;
import org.jbox2d.dynamics.World;

import static de.hfkbremen.robots.challenge.BodyUserData.ShapeType.WALL;

public class Wall extends BodyUserData {

    // A boundary is a simple rectangle with x,y,width,and height
    private final float x;
    private final float y;
    private final float w;
    private final float h;

    // But we also have to make a body for box2d to know about it
    private final Body b;

    public Wall(World pWorld, float x_, float y_, float w_, float h_) {
        super(null, WALL);
        reference = this;

        x = x_;
        y = y_;
        w = w_;
        h = h_;

        // Define the polygon
        PolygonShape sd = new PolygonShape();
        // Figure out the box2d coordinates
        float box2dW = w / 2;
        float box2dH = h / 2;
//        float box2dW = pBox2d.scalarPixelsToWorld(w / 2);
//        float box2dH = pBox2d.scalarPixelsToWorld(h / 2);
        sd.setAsBox(box2dW, box2dH);

        // Create the body
        BodyDef bd = new BodyDef();
        bd.type = BodyType.STATIC;
//        bd.position.set(pBox2d.coordPixelsToWorld(x, y));
//        b = pBox2d.createBody(bd);

        bd.position.set(x, y);
        b = pWorld.createBody(bd);

        // Attached the shape to the body using a Fixture
        Fixture fixture = b.createFixture(sd, 1);
        fixture.setUserData(new ObstacleWallFUD());

        b.setUserData(this);
    }
}
