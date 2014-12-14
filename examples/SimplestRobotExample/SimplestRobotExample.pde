import shiffman.box2d.*;
import de.hfkbremen.robots.challenge.*;

Environment mEnvironment;
MyRobot mRobot;

void setup() {
  size(1024, 768);
  mEnvironment = new Environment(this, Environment.MAP_SIMPLE);
  mRobot = new MyRobot(mEnvironment);
  mEnvironment.add(mRobot);
}

void draw() {
  background(255);
  mEnvironment.update();
  mEnvironment.draw(g);
}

class MyRobot extends Robot {

  MyRobot(Environment pEnvironment) {
    super(pEnvironment);
  }

  void update() {
    /* steer robot and controll its motor */
    if (keyPressed) {
      switch (key) {
      case 'a':
        steer(steer() + 0.1f);
        break;
      case 'd':
        steer(steer() - 0.1f);
        break;
      case 'w':
        speed(50);
        break;
      case 's':
        speed(0);
        break;
      }
    }
  }
}

