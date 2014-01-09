package rvolib;

public class RVOMath {
    public static float absSq(Vector2 v) {
        return Vector2.dot(v, v);
    }

    public static Vector2 normalize(Vector2 v) {
        return Vector2.divide(v, abs(v));
    }

    public final static float RVO_EPSILON = 0.00001f;

    public static float sqrt(float a) {
        return (float) Math.sqrt(a);
    }

    public static float fabs(float a) {
        return Math.abs(a);
    }

    public static float distSqPointLineSegment(Vector2 a, Vector2 b, Vector2 c) {
        float r = (Vector2.dot(Vector2.minus(c, a), Vector2.minus(b, a))) / absSq(Vector2.minus(b, a));

        if (r < 0.0f) {
            return absSq(Vector2.minus(c, a));
        } else if (r > 1.0f) {
            return absSq(Vector2.minus(c, b));
        } else {
            return absSq(Vector2.minus(c, (Vector2.plus(a, Vector2.scale(r, (Vector2.minus(b, a)))))));
        }
    }

    public static float sqr(float p) {
        return p * p;
    }

    public static float det(Vector2 v1, Vector2 v2) {
        return v1.x_ * v2.y_ - v1.y_ * v2.x_;
    }

    public static float abs(Vector2 v) {
        return (float) Math.sqrt(absSq(v));
    }

    public static float leftOf(Vector2 a, Vector2 b, Vector2 c) {
        return det(Vector2.minus(a, c), Vector2.minus(b, a));
    }
}


