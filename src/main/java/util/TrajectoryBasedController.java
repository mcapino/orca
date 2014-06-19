package util;

import java.util.Arrays;

import org.apache.log4j.Logger;

import tt.euclid2d.Vector;
import tt.euclid2i.Point;
import tt.euclid2i.Trajectory;

public class TrajectoryBasedController implements DesiredControl{
	
	static Logger LOGGER = Logger.getLogger(TrajectoryBasedController.class);
	
	protected Trajectory traj;
	protected double vmax;
	protected int samplingStep = 1;
	Point[] trajArray;
	
	
	public TrajectoryBasedController(Trajectory traj, double vmax, int samplingStep) {
		super();
		this.traj = traj;
		this.vmax = vmax;
		this.samplingStep = samplingStep;
		
		trajArray = new Point[(traj.getMaxTime()-traj.getMinTime()) / samplingStep];
		for (int i=0; i*samplingStep < traj.getMaxTime(); i++) {
			trajArray[i] = traj.get(i*samplingStep);
		}
	}

	@Override
	public Vector getDesiredControl(tt.euclid2d.Point currentPosition) {
		
		int indexOfClosest = 0;
		for (int i=0; i < trajArray.length; i++) {
			if (trajArray[i].toPoint2d().distance(currentPosition) < trajArray[indexOfClosest].toPoint2d().distance(currentPosition)) {
				indexOfClosest = i;
			}
		}
		
		Point pointToFollow = null;
		
		if (indexOfClosest < trajArray.length) {
			pointToFollow = trajArray[indexOfClosest+1];
		} else {
			pointToFollow = trajArray[indexOfClosest];
		}
		
		assert pointToFollow != null;
		
		Vector vector = new Vector(pointToFollow.x - trajArray[indexOfClosest].x, pointToFollow.y - trajArray[indexOfClosest].y);
		LOGGER.trace("closest time: " + indexOfClosest + " closest point: " + trajArray[indexOfClosest] + " pointToFollow: " + pointToFollow  + "vector: " + vector /* + " traj array: " + Arrays.toString(trajArray)*/);
		if (vector.length() > 0) {
			vector.normalize();
			vector.scale(vmax);
		}
		return vector;
	}

}
