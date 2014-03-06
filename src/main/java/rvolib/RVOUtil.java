package rvolib;

import java.util.ArrayList;

import tt.euclid2i.Point;
import tt.euclid2i.region.Polygon;

public class RVOUtil {
    /*
     * vertices MUST BE ADDED IN CLOCKWISE ORDER
     */
    public static void addObstacle(ArrayList<Vector2> vertices, ArrayList<RVOObstacle> obstacles) {
        if (vertices.size() < 2) {
            return;
        }

        int obstacleNo = obstacles.size();

        for (int i = 0; i < vertices.size(); ++i) {
            RVOObstacle obstacle = new RVOObstacle();
            obstacle.point_ = vertices.get(i);
            if (i != 0) {
                obstacle.prevObstacle = obstacles.get(obstacles.size() - 1);
                obstacle.prevObstacle.nextObstacle = obstacle;
            }
            if (i == vertices.size() - 1) {
                obstacle.nextObstacle = obstacles.get(obstacleNo);
                obstacle.nextObstacle.prevObstacle = obstacle;
            }
            obstacle.unitDir_ = RVOMath.normalize(Vector2.minus(vertices.get(i == vertices.size() - 1 ? 0 : i + 1), vertices.get(i)));

            if (vertices.size() == 2) {
                obstacle.isConvex_ = true;
            } else {
                obstacle.isConvex_ = (RVOMath.leftOf(vertices.get(i == 0 ? vertices.size() - 1 : i - 1), vertices.get(i), vertices.get(i == vertices.size() - 1 ? 0 : i + 1)) >= 0);
            }

            obstacle.id_ = obstacles.size();

            obstacles.add(obstacle);
        }
    }

	public static ArrayList<Vector2> regionToVectorList(Polygon polygon) {

		Point[] points = polygon.getPoints();

		ArrayList<Vector2> obstacle = new ArrayList<Vector2>();

		for (Point point : points) {
			obstacle.add(new Vector2(point.x, point.y));
		}
		return obstacle;
	}
}
