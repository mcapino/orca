package org.jgrapht.alg;

import org.jgrapht.EdgeFactory;
import org.jgrapht.WeightedGraph;
import org.jgrapht.graph.SimpleWeightedGraph;

import tt.euclid2i.Point;
import tt.euclid2i.Line;
import tt.euclid2i.Region;
import tt.euclid2i.region.Polygon;
import tt.euclid2i.util.Util;
import tt.util.NotImplementedException;

import java.util.Collection;

public class VisibilityGraph {

    public static WeightedGraph<Point, Line> createVisibilityGraph(Collection<Region> lessInflatedObstacles,
    		Collection<Region> moreInflatedObstacles) {

        @SuppressWarnings("serial")
		WeightedGraph<Point, Line> visibilityGraph = new SimpleWeightedGraph<Point, Line>(new EdgeFactory<Point, Line>() {

			@Override
			public Line createEdge(Point from, Point to) {
				return new Line(from, to);
			}

		}) {

			@Override
			public double getEdgeWeight(Line line) {
				return line.getDistance();
			}

			@Override
			@Deprecated
			public void setEdgeWeight(Line arg0, double arg1) {
				throw new NotImplementedException();
			}
        };

        for (Region inflatedObstacle : moreInflatedObstacles) {
            Polygon polygon = (Polygon) inflatedObstacle;
            Point[] points = polygon.getPoints();

            // add points
            for (int i = 0; i < points.length; i++) {
            	 if (!conflicting(lessInflatedObstacles, points[i])) {
            		 visibilityGraph.addVertex(points[i]);
                 }
            }
        }

        Point[] vertices = visibilityGraph.vertexSet().toArray(new Point[0]);

        for (int i=0; i < vertices.length; i++) {
        	for (int j=i+1; j < vertices.length; j++) {
        		if (!conflicting(lessInflatedObstacles, vertices[i], vertices[j])) {

        			 if (!vertices[i].equals(vertices[j]) && visibilityGraph.containsVertex(vertices[i])
        		                && visibilityGraph.containsVertex(vertices[j])) {

        				    Line l = visibilityGraph.addEdge(vertices[i], vertices[j]);
        		        }
                }
            }
        }
        return visibilityGraph;
    }

    private static boolean conflicting(Collection<Region> inflatedObstacles, Point p1m, Point p2m) {

        for (Region region : inflatedObstacles) {
            if (region.intersectsLine(p1m, p2m)) {
                return true;
            }
        }
        return false;
    }

    private static boolean conflicting(Collection<Region> inflatedObstacles, Point p1) {
        for (Region region : inflatedObstacles) {
            if (region.isInside(p1)) {
                return true;
            }
        }
        return false;
    }
}
