package org.jgrapht.alg;

import org.jgrapht.graph.SimpleGraph;
import tt.euclid2i.Point;
import tt.euclid2i.Region;
import tt.euclid2i.region.Polygon;
import tt.euclid2i.util.Util;

import java.util.Collection;
import java.util.HashSet;

public class ObstacleVisibilityGraphProvider {

    private static ObstacleVisibilityGraphProvider instance = new ObstacleVisibilityGraphProvider();
    private SimpleGraph<Point, WeightedLine> obstacleGraph;

    private Collection<Region> inflatedObstacles;

    public Collection<Region> getInflatedObstacles() {
        return inflatedObstacles;
    }

    private boolean isInitialized = false;

    public static ObstacleVisibilityGraphProvider getInstance() {
        return instance;
    }

    private ObstacleVisibilityGraphProvider() {
    }

    public void init(Collection<Region> obstacles, int distanceFromObstacles,
                     Collection<Region> inflatedObstacles) {
        obstacleGraph = new SimpleGraph<Point, WeightedLine>(WeightedLine.class);
        this.isInitialized = true;
        this.inflatedObstacles = inflatedObstacles;
        createGraphFromInflatedObstacles(obstacles, distanceFromObstacles);
    }

    public SimpleGraph<Point, WeightedLine> getObstacleGraph() {
        if (!isInitialized) {
            throw new RuntimeException(
                    "ObstacleVisibilityGraphProvider used without inititalization");
        }
        return obstacleGraph;
    }

    private void createGraphFromInflatedObstacles(Collection<Region> obstacles,
                                                  int distanceFromObstacles) {
        Collection<Region> furtherInflatedObstacles = Util.inflateRegions(
                obstacles, (int) distanceFromObstacles);

        for (Region inflatedObstacle : furtherInflatedObstacles) {
            Polygon polygon = (Polygon) inflatedObstacle;
            Point[] points = polygon.getPoints();
            // add points
            for (int i = 0; i < points.length; i++) {
                safeAdd(points[i], obstacleGraph);
            }
            // add WeightedLines
            for (int i = 0; i < points.length; i++) {
                if (i != points.length - 1) {
                    safeAdd(points[i], points[i + 1], obstacleGraph);
                } else {
                    safeAdd(points[i], points[0], obstacleGraph);
                }

            }
        }
        createFullObstacleGraph();

    }

    /**
     * create full graph
     */
    private void createFullObstacleGraph() {

        HashSet<Point> vertexSet = new HashSet<Point>(obstacleGraph.vertexSet());
        for (Point p1 : vertexSet) {
            for (Point p2 : vertexSet) {

                if (!p1.equals(p2)) {

                    if ((obstacleGraph.getEdge(p1, p2) == null)
                            && (!conflicting(p1, p2))) {
                        addWeightedEdgeToGraph(obstacleGraph, p1, p2);
                    }
                }
            }
        }
    }

    private void safeAdd(Point point, SimpleGraph<Point, WeightedLine> graph) {
        if (!conflicting(point)) {
            graph.addVertex(point);
        }
    }

    private void safeAdd(Point point1, Point point2,
                         SimpleGraph<Point, WeightedLine> obstacleGraph2) {
        if (!conflicting(point1, point2)) {
            addWeightedEdgeToGraph(obstacleGraph, point1, point2);
        }
    }

    private void addWeightedEdgeToGraph(SimpleGraph<Point, WeightedLine> graph,
                                        Point p1, Point p2) {
        if (!p1.equals(p2) && graph.containsVertex(p1)
                && graph.containsVertex(p2)) {

            graph.addEdge(p1, p2);
            WeightedLine l = graph.getEdge(p1, p2);
            double length = p1.distance(p2);
            graph.setEdgeWeight(l, length);
        }
    }

    private boolean conflicting(Point p1m, Point p2m) {

        for (Region region : inflatedObstacles) {
            if (region.intersectsLine(p1m, p2m)) {
                return true;
            }
        }
        return false;
    }

    private boolean conflicting(Point p1) {
        for (Region region : inflatedObstacles) {
            if (region.isInside(p1)) {
                return true;
            }
        }
        return false;
    }
}
