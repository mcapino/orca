package org.jgrapht.alg;

import java.util.Collection;
import java.util.HashMap;

import org.jgrapht.GraphPath;
import org.jgrapht.WeightedGraph;
import org.jgrapht.util.Goal;
import org.jgrapht.util.HeuristicToGoal;

import rvolib.VisibilityGraphLayer;
import tt.euclid2i.Line;
import tt.euclid2i.Point;
import tt.euclid2i.Region;
import tt.euclid2i.discretization.VisibilityGraph;
import cz.agents.alite.vis.VisManager;

public class VisibilityGraphPlanner {

    private WeightedGraph<Point, Line>  graph;

    public VisibilityGraphPlanner(Point start, Point goal,
                                  Collection<Region> lessInflatedObstacles,
                                  Collection<Region> moreInflatedObstacles,
                                  boolean showVis) {
        this.graph = VisibilityGraph.createVisibilityGraph(start, goal, lessInflatedObstacles, moreInflatedObstacles);

        if (showVis) {
            VisManager.registerLayer(VisibilityGraphLayer.create(this));
        }
    }

    public HashMap<Point, Double> evaluateGraph(float graphRadius, Point goal) {
        DijkstraShortestPaths<Point, Line> dijkstra = new DijkstraShortestPaths<Point, Line>(
                graph, goal, new Goal<Point>() {
            @Override
            public boolean isGoal(Point current) {
                return false;
            }
        }, graphRadius);

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

    public WeightedGraph<Point, Line>  getGraph() {
        return graph;
    }

}
