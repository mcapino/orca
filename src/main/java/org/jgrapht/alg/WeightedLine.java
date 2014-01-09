package org.jgrapht.alg;


import org.jgrapht.graph.DefaultWeightedEdge;
import tt.euclid2i.Point;

import javax.vecmath.Point2d;


public class WeightedLine extends DefaultWeightedEdge {
    private static final long serialVersionUID = -2519868162204278196L;


    public WeightedLine() {
    }


    public Point getStart() {
        return (Point) this.getSource();
    }

    public Point getEnd() {
        return (Point) this.getTarget();
    }

    @Override
    public String toString() {
        return String.format("(%s : %s)", this.getSource(), this.getTarget());
    }

    public double getDistance() {
        return new Point2d(getStart().x, getStart().y).distance(new Point2d(getEnd().x, getEnd().y));
    }

    /*
    @Override
    public Trajectory getTrajectory(final double startTime) {
        return new Trajectory() {

            @Override
            public OrientedPoint getPosition(double t) {
                if (t < startTime || t > startTime + getDuration())
                    throw new IllegalArgumentException("The position for time " + t + " which is undefined for this trajectory. Length: " + getDistance() + ". Trajectory defined for interval (" + startTime + ", " + (startTime + getDuration()) + ")");

                if (getDuration() > 0) {
                    double alpha = (t - startTime) / getDuration();
                    assert(alpha >= -0.00001 && alpha <= 1.00001);

                    Euclidean2dPoint pos = Euclidean2dPoint.interpolate(new Euclidean2dPoint(start.x, start.y), new Euclidean2dPoint(end.x, end.y), alpha);
                    Vector2d dir = new Vector2d(0,1);
                    if (!end.equals(start)) {
                        dir.sub(new Point2d(end.x, end.y), new Point2d(start.x, start.y));
                        dir.normalize();
                    }
                    return new OrientedPoint(new SpatialPoint(pos.x, pos.y, 0), new Vector(dir.x, dir.y, 0));
                } else {
                    return new OrientedPoint(new SpatialPoint(start.x, start.y, 0) , new Vector(0, 1, 0));
                }


            }

            @Override
            public double getMinTime() {
                return startTime;
            }

            @Override
            public double getMaxTime() {
                return startTime + getDuration();
            }
        };
    }*/


}
