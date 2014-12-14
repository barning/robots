package de.hfkbremen.robots.challenge.environment;

import de.hfkbremen.robots.challenge.BodyUserData;
import de.hfkbremen.robots.challenge.FUD.ObstacleBallFUD;
import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.Fixture;
import org.jbox2d.dynamics.World;

public class EnvironmentMapBaelleBad extends EnvironmentMapEmpty {

    public EnvironmentMapBaelleBad(World pWorld, final int pBalls) {
        super(pWorld, new Vec2(1024 / 2.5f, 768 / 2.5f));

        /* create balls */
        for (int i = 0; i < pBalls; i++) {
            Vec2 mRandomPosition = new Vec2((float) (Math.random() - 0.5f) * mBox.x,
                                            (float) (Math.random() - 0.5f) * mBox.y);
            float mRandomScale = (float) Math.random() * 5 + 2;
            createBall(pWorld, mRandomScale, mRandomPosition);
        }
    }

    private void createBall(World pWorld, float pRadius, Vec2 pPosition) {
        BodyDef bodyDef = new BodyDef();
        bodyDef.type = BodyType.DYNAMIC;
        bodyDef.position.set(pPosition);

        Body mBody = pWorld.createBody(bodyDef);
        mBody.setUserData(new BodyUserData(this, BodyUserData.ShapeType.OBSTACLE_BALL));

        CircleShape mShape = new CircleShape();
        mShape.setRadius(pRadius);

        Fixture fixture = mBody.createFixture(mShape, 1);
        fixture.setUserData(new ObstacleBallFUD());
    }
}
