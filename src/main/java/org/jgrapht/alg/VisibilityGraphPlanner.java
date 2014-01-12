package org.jgrapht.alg;

import cz.agents.alite.vis.VisManager;

import org.jgrapht.GraphPath;
import org.jgrapht.WeightedGraph;
import org.jgrapht.graph.AbstractBaseGraph;
import org.jgrapht.util.Goal;
import org.jgrapht.util.HeuristicToGoal;
import rvolib.VisibilityGraphLayer;
import tt.euclid2i.Point;
import tt.euclid2i.Line;
import tt.euclid2i.Region;

import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;

/**
 * @author Pavel Janovsky Planner creates visibility graph and finds the path
 *         using implemented A Star algorithm. First it creates graph from
 *         buildings only (it moves the vertexes from the collision zones), then
 *         it constructs full graph. Finally for each start and end points it
 *         adds the points and edges to the graph and search the graph using
 *         AStar.
 */
public class VisibilityGraphPlanner {

    private WeightedGraph<Point, Line>  graph;
    private WeightedGraph<Point, Line>  obstacleGraph;

    private Collection<Region> inflatedObstacles;

    public VisibilityGraphPlanner(WeightedGraph<Point, Line>  obstacleGraph, Collection<Region> inflatedObstacles,
                                  boolean showVis) {

        this.obstacleGraph = obstacleGraph;
        this.inflatedObstacles = inflatedObstacles;

        if (showVis) {
            VisManager.registerLayer(VisibilityGraphLayer.create(this));
        }
    }

    /**
     * add init and goal vertexes
     *
     * @param goals
     * @param init
     * @param goal
     */
    @SuppressWarnings("unchecked")
    public void createVisibilityGraph(Point[] starts, Point[] goals) {
        // System.out.println("Creating final graph");
        long t1 = System.nanoTime();
        graph = (WeightedGraph<Point, Line>)((AbstractBaseGraph)obstacleGraph).clone();
        long t2 = System.nanoTime();
//		System.out.println("clone graph: "+(t2 - t1));
        t1 = System.nanoTime();
        addNodes(starts);
        addNodes(goals);
        t2 = System.nanoTime();
//		System.out.println("add nodes: "+(t2 - t1));
        // System.out.println("final graph created");
    }

    public void createVisibilityGraph(Point[] points) {
        createVisibilityGraph(null, points);
    }

    public void createVisibilityGraph(Point point) {
        Point[] arr = new Point[]{point};
        createVisibilityGraph(null, arr);
    }

    private void addWeightedEdgeToGraph(WeightedGraph<Point, Line>  graph,
                                        Point p1, Point p2) {
        if (!p1.equals(p2) && graph.containsVertex(p1)
                && graph.containsVertex(p2)) {
            graph.addEdge(p1, p2);
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

    /**
     * find shortest paths from all nodes to goal, use ONLY for ORCA
     *
     * @param goal
     * @return
     * @return
     */
    public HashMap<Point, Double> evaluateGraph(int agentNumber, float graphRadius, Point goal) {
        // LOGGER.info("VISIBILITY GRAPH CREATING PLAN");

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

    public GraphPath<Point, Line> getShortestPath(float graphRadius,
                                                          Point start, final Point goal) {

//		AStarShortestPath<Point, Line> astar = new AStarShortestPath<Point, Line>(
//				graph, new HeuristicToGoal<Point>() {
//
//					@Override
//					public double getCostToGoalEstimate(Point current) {
//						double estimate = current.distance(goal);
//						return estimate;
//					}
//				}, start, new Goal<Point>() {
//					@Override
//					public boolean isGoal(Point current) {
//						return current.equals(goal);
//					}
//				}, graphRadius);
        long t1 = System.nanoTime();
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

        GraphPath<Point, Line> path = astarSimple.findPath((int) 100000000000l);
        long t2 = System.nanoTime();
//		System.out.println("a*: "+(t2 - t1));
        return path;
    }

    public WeightedGraph<Point, Line>  getGraph() {
        return graph;
    }

    private void addNodes(Point[] nodes) {
        if (nodes != null) {
            for (int i = 0; i < nodes.length; i++) {
                if (nodes[i] != null) {
                    graph.addVertex(nodes[i]);
                    HashSet<Point> vertexSet = new HashSet<Point>(
                            graph.vertexSet());
                    for (Point p1 : vertexSet) {
                        if (!conflicting(p1, nodes[i])) {
                            addWeightedEdgeToGraph(graph, p1, nodes[i]);
                        }
                    }
                }
            }
        }
    }

}
