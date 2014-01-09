package rvolib;

public class RVOObstacle {

    public Vector2 point_;
    public Vector2 unitDir_;
    public boolean isConvex_;
    public int id_;
    public RVOObstacle prevObstacle;
    public RVOObstacle nextObstacle;
}
