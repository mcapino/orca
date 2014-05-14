package org.jgrapht.alg;

import java.util.Collection;
import java.util.HashMap;
import java.util.Map.Entry;

import org.jgrapht.DirectedGraph;
import org.jgrapht.GraphPath;
import org.jgrapht.WeightedGraph;
import org.jgrapht.util.Goal;
import org.jgrapht.util.HeuristicToGoal;

import rvolib.VisibilityGraphLayer;
import tt.euclid2i.Line;
import tt.euclid2i.Point;
import tt.euclid2i.Region;
import tt.euclid2d.Vector;
import tt.euclid2i.discretization.VisibilityGraph;
import tt.euclid2i.util.Util;
import util.DesiredControl;
import cz.agents.alite.vis.VisManager;

public class GraphBasedController implements DesiredControl{

	private Point goal;
    private DirectedGraph<Point, Line>  graph;
	private HashMap<Point, Double> nodeValues;
	private Collection<Region> obstacles;
	private double vmax;
	private double bestNodeSearchRadius;

	public GraphBasedController(DirectedGraph<Point, Line> graph,
			Point goal, Collection<Region> obstacles,
			double vmax, double bestNodeSearchRadius, boolean showVis) {
        this.graph = graph;
        this.vmax = vmax;
        this.goal = goal;
        this.nodeValues = evaluateGraph(goal);
        this.obstacles = obstacles;
        this.bestNodeSearchRadius = bestNodeSearchRadius;

        if (showVis) {
            VisManager.registerLayer(VisibilityGraphLayer.create(this));
        }
    }

    public HashMap<Point, Double> evaluateGraph(Point goal) {
        DijkstraShortestPaths<Point, Line> dijkstra = new DijkstraShortestPaths<Point, Line>(
                graph, goal, new Goal<Point>() {
            @Override
            public boolean isGoal(Point current) {
                return false;
            }
        }, Double.MAX_VALUE);

        HashMap<Point, Double> lengths = dijkstra.findLengths(100000000000l);
        return lengths;
    }

    public GraphPath<Point, Line> getShortestPath(Point start, final Point goal) {
        AStarShortestPathSimple<Point, Line> astarSimple = new AStarShortestPathSimple<Point, Line>(
                graph, new HeuristicToGoal<Point>() {

            @Override
            public double getCostToGoalEstimate(Point current) {
                double estimate = current.distance(goal);
                return estimate;
            }
        }, start, new Goal<Point>() {
            @Override
            public boolean isGoal(Point current) {
                return current.equals(goal);
            }
        }
        );

        GraphPath<Point, Line> path = astarSimple.findPath(100000000);
        return path;
    }

    public DirectedGraph<Point, Line>  getGraph() {
        return graph;
    }

	@Override
	public tt.euclid2d.Vector getDesiredControl(tt.euclid2d.Point currentPosition) {

		double minTotalDistanceToGoal = Double.MAX_VALUE;
		tt.euclid2d.Vector bestDirection = null;

		for (Entry<Point, Double> pointValuePair : nodeValues.entrySet()) {
			Point node = pointValuePair.getKey();
			double value = pointValuePair.getValue();

			Vector vector = new Vector(node.x - currentPosition.x, node.y - currentPosition.y);

			double distToCurrentPosition = vector.length();
			double sumTotalDistToGoal = distToCurrentPosition + value;
			Point currentPos2i = new Point((int)currentPosition.x, (int) currentPosition.y);

			if (sumTotalDistToGoal < minTotalDistanceToGoal
					&& currentPos2i.distance(node) <= bestNodeSearchRadius
					&& Util.isVisible(currentPos2i, node, obstacles)
					) {
				bestDirection = vector;
				minTotalDistanceToGoal = sumTotalDistToGoal;
			}
		}

		if (bestDirection != null) {
			bestDirection.normalize();
			bestDirection.scale(vmax);
			return bestDirection;
		} else {
			System.err.println("Cannot find path to goal. Using straight-line direction to goal");
			tt.euclid2d.Vector vector = new tt.euclid2d.Vector(goal.x - currentPosition.x, goal.y - currentPosition.y);
			vector.normalize();
			vector.scale(vmax);
			return vector;
		}
	}

}
