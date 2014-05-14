package util;

import tt.euclid2d.Point;
import tt.euclid2d.Vector;


public interface DesiredControl {
	public Vector getDesiredControl(Point currentPosition);
}
