package rvolib;

import java.util.ArrayList;
import java.util.Arrays;

public class KdTree {

    //TODO check
    private static class FloatPair {
        private float _a;
        private float _b;


        public FloatPair(float a, float b) {
            _a = a;
            _b = b;
        }

        public static boolean lessThan(FloatPair lhs, FloatPair rhs) {
            return (lhs._a < rhs._a || !(rhs._a < lhs._a) && lhs._b < rhs._b);
        }

        public static boolean lessThanOrEqual(FloatPair lhs, FloatPair rhs) {
            return (lhs._a == rhs._a && lhs._b == rhs._b) || lessThan(lhs, rhs);
        }

        public static boolean greaterThan(FloatPair lhs, FloatPair rhs) {
            return !lessThanOrEqual(lhs, rhs);
        }

        public static boolean greaterThanOrEqual(FloatPair lhs, FloatPair rhs) {
            return !lessThan(lhs, rhs);
        }
    }

    private final int MAX_LEAF_SIZE = 10;

    private class AgentTreeNode {
        public int begin;
        public int end;
        public int left;
        public float maxX;
        public float maxY;
        public float minX;
        public float minY;
        public int right;
    }

    private class ObstacleTreeNode {
        public ObstacleTreeNode left;
        public RVOObstacle obstacle;
        public ObstacleTreeNode right;
    }

    private RVOAgent[] agents_;
    private AgentTreeNode[] agentTree_;
    private ObstacleTreeNode obstacleTree_;

    public void clearAgents() {
    	agents_ = null;
    }

    public void buildAgentTree(RVOAgent[] newAgents) {
    	// if inconsistent, rebuild the tree
    	System.err.println(Arrays.toString(newAgents));
        if (agents_ == null || agents_.length != newAgents.length) {
            agents_ = new RVOAgent[newAgents.length];
            for (int i = 0; i < agents_.length; ++i) {
                agents_[i] = newAgents[i];
            }

            agentTree_ = new AgentTreeNode[2 * agents_.length];
            for (int i = 0; i < agentTree_.length; ++i) {
                agentTree_[i] = new AgentTreeNode();
            }
        }

        // update existing objects
        if (agents_.length != 0) {
            buildAgentTreeRecursive(0, agents_.length, 0);
        }
    }

    void buildAgentTreeRecursive(int begin, int end, int node) {
        agentTree_[node].begin = begin;
        agentTree_[node].end = end;
        agentTree_[node].minX = agentTree_[node].maxX = agents_[begin].position_.x_;
        agentTree_[node].minY = agentTree_[node].maxY = agents_[begin].position_.y_;

        for (int i = begin + 1; i < end; ++i) {
            agentTree_[node].maxX = Math.max(agentTree_[node].maxX, agents_[i].position_.x_);
            agentTree_[node].minX = Math.min(agentTree_[node].minX, agents_[i].position_.x_);
            agentTree_[node].maxY = Math.max(agentTree_[node].maxY, agents_[i].position_.y_);
            agentTree_[node].minY = Math.min(agentTree_[node].minY, agents_[i].position_.y_);
        }

        if (end - begin > MAX_LEAF_SIZE) {
            /* No leaf node. */
            boolean isVertical = (agentTree_[node].maxX - agentTree_[node].minX > agentTree_[node].maxY - agentTree_[node].minY);
            float splitValue = (isVertical ? 0.5f * (agentTree_[node].maxX + agentTree_[node].minX) : 0.5f * (agentTree_[node].maxY + agentTree_[node].minY));

            int left = begin;
            int right = end;

            while (left < right) {
                while (left < right && (isVertical ? agents_[left].position_.x_ : agents_[left].position_.y_) < splitValue) {
                    ++left;
                }

                while (right > left && (isVertical ? agents_[right - 1].position_.x_ : agents_[right - 1].position_.y_) >= splitValue) {
                    --right;
                }

                if (left < right) {
                    RVOAgent tmp = agents_[left];
                    agents_[left] = agents_[right - 1];
                    agents_[right - 1] = tmp;
                    ++left;
                    --right;
                }
            }

            int leftSize = left - begin;

            if (leftSize == 0) {
                ++leftSize;
                ++left;
                ++right;
            }

            agentTree_[node].left = node + 1;
            agentTree_[node].right = node + 1 + (2 * leftSize - 1);

            buildAgentTreeRecursive(begin, left, agentTree_[node].left);
            buildAgentTreeRecursive(left, end, agentTree_[node].right);
        }
    }

    public void buildObstacleTree(RVOObstacle[] obstaclesArg, ArrayList<RVOObstacle> obstacles2) {
        obstacleTree_ = new ObstacleTreeNode();

        ArrayList<RVOObstacle> obstacles = new ArrayList<RVOObstacle>(obstaclesArg.length);

        for (int i = 0; i < obstaclesArg.length ; ++i) {
            obstacles.add(obstaclesArg[i]);
        }

        obstacleTree_ = buildObstacleTreeRecursive(obstacles, obstacles2);
    }

    ObstacleTreeNode buildObstacleTreeRecursive(ArrayList<RVOObstacle> obstacles, ArrayList<RVOObstacle> obstacles2) {
        if (obstacles.size() == 0) {
            return null;
        } else {
            ObstacleTreeNode node = new ObstacleTreeNode();

            int optimalSplit = 0;
            int minLeft = obstacles.size();
            int minRight = obstacles.size();

            for (int i = 0; i < obstacles.size(); ++i) {
                int leftSize = 0;
                int rightSize = 0;

                RVOObstacle obstacleI1 = obstacles.get(i);
                RVOObstacle obstacleI2 = obstacleI1.nextObstacle;

                /* Compute optimal split node. */
                for (int j = 0; j < obstacles.size(); ++j) {
                    if (i == j) {
                        continue;
                    }

                    RVOObstacle obstacleJ1 = obstacles.get(j);
                    RVOObstacle obstacleJ2 = obstacleJ1.nextObstacle;

                    float j1LeftOfI = RVOMath.leftOf(obstacleI1.point_, obstacleI2.point_, obstacleJ1.point_);
                    float j2LeftOfI = RVOMath.leftOf(obstacleI1.point_, obstacleI2.point_, obstacleJ2.point_);

                    if (j1LeftOfI >= -RVOMath.RVO_EPSILON && j2LeftOfI >= -RVOMath.RVO_EPSILON) {
                        ++leftSize;
                    } else if (j1LeftOfI <= RVOMath.RVO_EPSILON && j2LeftOfI <= RVOMath.RVO_EPSILON) {
                        ++rightSize;
                    } else {
                        ++leftSize;
                        ++rightSize;
                    }

                    if (FloatPair.greaterThanOrEqual(new FloatPair(Math.max(leftSize, rightSize), Math.min(leftSize, rightSize)), new FloatPair(Math.max(minLeft, minRight), Math.min(minLeft, minRight)))) {
                        break;
                    }
                }

                if (FloatPair.lessThan(new FloatPair(Math.max(leftSize, rightSize), Math.min(leftSize, rightSize)), new FloatPair(Math.max(minLeft, minRight), Math.min(minLeft, minRight)))) {
                    minLeft = leftSize;
                    minRight = rightSize;
                    optimalSplit = i;
                }
            }

            {
                /* Build split node. */
                ArrayList<RVOObstacle> leftObstacles = new ArrayList<RVOObstacle>(minLeft);
                for (int n = 0; n < minLeft; ++n) leftObstacles.add(null);
                ArrayList<RVOObstacle> rightObstacles = new ArrayList<RVOObstacle>(minRight);
                for (int n = 0; n < minRight; ++n) rightObstacles.add(null);

                int leftCounter = 0;
                int rightCounter = 0;
                int i = optimalSplit;

                RVOObstacle obstacleI1 = obstacles.get(i);
                RVOObstacle obstacleI2 = obstacleI1.nextObstacle;

                for (int j = 0; j < obstacles.size(); ++j) {
                    if (i == j) {
                        continue;
                    }

                    RVOObstacle obstacleJ1 = obstacles.get(j);
                    RVOObstacle obstacleJ2 = obstacleJ1.nextObstacle;

                    float j1LeftOfI = RVOMath.leftOf(obstacleI1.point_, obstacleI2.point_, obstacleJ1.point_);
                    float j2LeftOfI = RVOMath.leftOf(obstacleI1.point_, obstacleI2.point_, obstacleJ2.point_);

                    if (j1LeftOfI >= -RVOMath.RVO_EPSILON && j2LeftOfI >= -RVOMath.RVO_EPSILON) {
                        leftObstacles.set(leftCounter++, obstacles.get(j));
                    } else if (j1LeftOfI <= RVOMath.RVO_EPSILON && j2LeftOfI <= RVOMath.RVO_EPSILON) {
                        rightObstacles.set(rightCounter++, obstacles.get(j));
                    } else {
                        /* Split obstacle j. */
                        float t = RVOMath.det(Vector2.minus(obstacleI2.point_, obstacleI1.point_), Vector2.minus(obstacleJ1.point_, obstacleI1.point_)) / RVOMath.det(Vector2.minus(obstacleI2.point_, obstacleI1.point_), Vector2.minus(obstacleJ1.point_, obstacleJ2.point_));

                        Vector2 splitpoint = Vector2.plus(obstacleJ1.point_, Vector2.scale(t, Vector2.minus(obstacleJ2.point_, obstacleJ1.point_)));

                        RVOObstacle newObstacle = new RVOObstacle();
                        newObstacle.point_ = splitpoint;
                        newObstacle.prevObstacle = obstacleJ1;
                        newObstacle.nextObstacle = obstacleJ2;
                        newObstacle.isConvex_ = true;
                        newObstacle.unitDir_ = obstacleJ1.unitDir_;

                        newObstacle.id_ = obstacles2.size();

                        obstacles2.add(newObstacle);

                        obstacleJ1.nextObstacle = newObstacle;
                        obstacleJ2.prevObstacle = newObstacle;

                        if (j1LeftOfI > 0.0f) {
                            leftObstacles.set(leftCounter++, obstacleJ1);
                            rightObstacles.set(rightCounter++, newObstacle);
                        } else {
                            rightObstacles.set(rightCounter++, obstacleJ1);
                            leftObstacles.set(leftCounter++, newObstacle);
                        }
                    }
                }

                node.obstacle = obstacleI1;
                node.left = buildObstacleTreeRecursive(leftObstacles, obstacles2);
                node.right = buildObstacleTreeRecursive(rightObstacles, obstacles2);
                return node;
            }
        }
    }

    public void computeAgentNeighbors(RVOAgent agent, MutableFloat rangeSq) {
        queryAgentTreeRecursive(agent, rangeSq, 0);
    }

    public void computeObstacleNeighbors(RVOAgent agent, float rangeSq) {
        queryObstacleTreeRecursive(agent, rangeSq, obstacleTree_);
    }

    private void queryAgentTreeRecursive(RVOAgent agent, MutableFloat rangeSq, int node) {
        if (agentTree_[node].end - agentTree_[node].begin <= MAX_LEAF_SIZE) {
            for (int i = agentTree_[node].begin; i < agentTree_[node].end; ++i) {
                agent.insertAgentNeighbor(agents_[i], rangeSq);
            }
        } else {
            float distSqLeft = RVOMath.sqr(Math.max(0.0f, agentTree_[agentTree_[node].left].minX - agent.position_.x_)) + RVOMath.sqr(Math.max(0.0f, agent.position_.x_ - agentTree_[agentTree_[node].left].maxX)) + RVOMath.sqr(Math.max(0.0f, agentTree_[agentTree_[node].left].minY - agent.position_.y_)) + RVOMath.sqr(Math.max(0.0f, agent.position_.y_ - agentTree_[agentTree_[node].left].maxY));

            float distSqRight = RVOMath.sqr(Math.max(0.0f, agentTree_[agentTree_[node].right].minX - agent.position_.x_)) + RVOMath.sqr(Math.max(0.0f, agent.position_.x_ - agentTree_[agentTree_[node].right].maxX)) + RVOMath.sqr(Math.max(0.0f, agentTree_[agentTree_[node].right].minY - agent.position_.y_)) + RVOMath.sqr(Math.max(0.0f, agent.position_.y_ - agentTree_[agentTree_[node].right].maxY));

            if (distSqLeft < distSqRight) {
                if (distSqLeft < rangeSq.getValue()) {
                    queryAgentTreeRecursive(agent, rangeSq, agentTree_[node].left);

                    if (distSqRight < rangeSq.getValue()) {
                        queryAgentTreeRecursive(agent, rangeSq, agentTree_[node].right);
                    }
                }
            } else {
                if (distSqRight < rangeSq.getValue()) {
                    queryAgentTreeRecursive(agent, rangeSq, agentTree_[node].right);

                    if (distSqLeft < rangeSq.getValue()) {
                        queryAgentTreeRecursive(agent, rangeSq, agentTree_[node].left);
                    }
                }
            }

        }
    }

    private void queryObstacleTreeRecursive(RVOAgent agent, float rangeSq, ObstacleTreeNode node) {
        if (node == null) {
            return;
        } else {
            RVOObstacle obstacle1 = node.obstacle;
            RVOObstacle obstacle2 = obstacle1.nextObstacle;

            float agentLeftOfLine = RVOMath.leftOf(obstacle1.point_, obstacle2.point_, agent.position_);

            queryObstacleTreeRecursive(agent, rangeSq, (agentLeftOfLine >= 0.0f ? node.left : node.right));

            float distSqLine = RVOMath.sqr(agentLeftOfLine) / RVOMath.absSq(Vector2.minus(obstacle2.point_, obstacle1.point_));

            if (distSqLine < rangeSq) {
                if (agentLeftOfLine < 0.0f) {
                    /*
                     * Try obstacle at this node only if agent is on right side of
                     * obstacle (and can see obstacle).
                     */
                    agent.insertObstacleNeighbor(node.obstacle, rangeSq);
                }

                /* Try other side of line. */
                queryObstacleTreeRecursive(agent, rangeSq, (agentLeftOfLine >= 0.0f ? node.right : node.left));

            }
        }
    }

    public boolean queryVisibility(Vector2 q1, Vector2 q2, float radius) {
        return queryVisibilityRecursive(q1, q2, radius, obstacleTree_);
    }

    private boolean queryVisibilityRecursive(Vector2 q1, Vector2 q2, float radius, ObstacleTreeNode node) {
        if (node == null) {
            return true;
        } else {
            RVOObstacle obstacle1 = node.obstacle;
            RVOObstacle obstacle2 = obstacle1.nextObstacle;

            float q1LeftOfI = RVOMath.leftOf(obstacle1.point_, obstacle2.point_, q1);
            float q2LeftOfI = RVOMath.leftOf(obstacle1.point_, obstacle2.point_, q2);
            float invLengthI = 1.0f / RVOMath.absSq(Vector2.minus(obstacle2.point_, obstacle1.point_));

            if (q1LeftOfI >= 0.0f && q2LeftOfI >= 0.0f) {
                return queryVisibilityRecursive(q1, q2, radius, node.left) && ((RVOMath.sqr(q1LeftOfI) * invLengthI >= RVOMath.sqr(radius) && RVOMath.sqr(q2LeftOfI) * invLengthI >= RVOMath.sqr(radius)) || queryVisibilityRecursive(q1, q2, radius, node.right));
            } else if (q1LeftOfI <= 0.0f && q2LeftOfI <= 0.0f) {
                return queryVisibilityRecursive(q1, q2, radius, node.right) && ((RVOMath.sqr(q1LeftOfI) * invLengthI >= RVOMath.sqr(radius) && RVOMath.sqr(q2LeftOfI) * invLengthI >= RVOMath.sqr(radius)) || queryVisibilityRecursive(q1, q2, radius, node.left));
            } else if (q1LeftOfI >= 0.0f && q2LeftOfI <= 0.0f) {
                /* One can see through obstacle from left to right. */
                return queryVisibilityRecursive(q1, q2, radius, node.left) && queryVisibilityRecursive(q1, q2, radius, node.right);
            } else {
                float point1LeftOfQ = RVOMath.leftOf(q1, q2, obstacle1.point_);
                float point2LeftOfQ = RVOMath.leftOf(q1, q2, obstacle2.point_);
                float invLengthQ = 1.0f / RVOMath.absSq(Vector2.minus(q2, q1));

                return (point1LeftOfQ * point2LeftOfQ >= 0.0f && RVOMath.sqr(point1LeftOfQ) * invLengthQ > RVOMath.sqr(radius) && RVOMath.sqr(point2LeftOfQ) * invLengthQ > RVOMath.sqr(radius) && queryVisibilityRecursive(q1, q2, radius, node.left) && queryVisibilityRecursive(q1, q2, radius, node.right));
            }
        }
    }
}
