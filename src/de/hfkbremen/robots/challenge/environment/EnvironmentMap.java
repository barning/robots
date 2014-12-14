package de.hfkbremen.robots.challenge.environment;

import de.hfkbremen.robots.challenge.Wall;
import java.util.ArrayList;
import org.jbox2d.dynamics.Body;

public interface EnvironmentMap {

    ArrayList<Wall> walls();

    Body grounds();
}
