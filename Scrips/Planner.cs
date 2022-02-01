using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using Priority_Queue;

namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    [RequireComponent(typeof(DroneController))]

    public class Planner
    {
        private Node startNode { get; set;}
        private Node goalNode { get; set;}
        private float[] steering { get; set;}
        public float gridSize = 0;
        public float x_low = 0;
        public float z_low = 0;
        public float maxSteerAngle = 0;

        public Planner()
        {
            ;
        }

        private float calculateEuclidean(float x1, float z1, float x2, float z2)
        {
            return (float)Math.Sqrt(Math.Pow( (x1-x2) , 2) + Math.Pow( (z1-z2), 2 ));
        }

        public int calculateGridIndex(float x, float z)
        {
            int xIdx = (int)Math.Round((x - x_low) / 1);
            int zIdx = (int)Math.Round((z - z_low) / 1);
            int gridIdx = zIdx*( (int)gridSize + 1) + xIdx;

            return gridIdx;
        }

        public Node HybridAStar(TerrainManager terrain_manager, CarController m_Car, Vector3 start_pos, float start_angle, Vector3 goal_pos, float[,] obstacle_map, int MAX_SIZE = 10000)
        {   
            Debug.Log("In HybridAStar");
            // Computing the gridSize
            //maxSteerAngle = m_Car.m_MaximumSteerAngle;
            //Debug.Log("Maximum steer angle: " + maxSteerAngle);
            steering = new float[]{-(float)Math.PI/4, -(float)Math.PI/8, 0, (float)Math.PI/8, (float)Math.PI/4};
            gridSize = terrain_manager.myInfo.x_high - terrain_manager.myInfo.x_low;
            x_low = terrain_manager.myInfo.x_low;
            z_low = terrain_manager.myInfo.z_low;
            int k = 0;

            // Create the startnode
            startNode = new Node(start_pos.x, start_pos.z, start_angle, calculateGridIndex(start_pos.x, start_pos.z), 0, 0, 0, null);

            // Create the goalnode
            goalNode = new Node(goal_pos.x, goal_pos.z, 0, calculateGridIndex(goal_pos.x, goal_pos.z), 0, 0, 0, null);

            // Creating the open set (Priority queue for guided search of map)
            FastPriorityQueue<Node> openSet = new FastPriorityQueue<Node>(MAX_SIZE);

            // Creating the closed set (Dictionary for book keeping of expanded nodes)
            Dictionary<float,Node> closedSet = new Dictionary<float, Node>();

            // Starting the algorithm
            openSet.Enqueue(startNode, startNode.f);

            while(openSet.Count > 0)
            {   
                k++;

                if(k == 2)
                {
                    return null;
                }
                
                // Getting node with highest priority from open set
                Node highestPriorityNode = openSet.Dequeue();

                // Push node onto the set of expanded nodes
                closedSet.Add(highestPriorityNode.gridIdx, highestPriorityNode);

                // If the node is in the vicinity of the goal, assign it as parent to the goalnode and return the goalnode
                if(calculateEuclidean(highestPriorityNode.x, highestPriorityNode.z, goalNode.x, goalNode.z) < 2)
                {
                    Node finalNode = new Node(goalNode.x, goalNode.z, goalNode.theta, calculateGridIndex(goalNode.x, goalNode.z), 0, 0, 0, highestPriorityNode);
                    goalNode = finalNode;
                    return goalNode;
                }

                // Expand node
                foreach(float steerAngle in steering)
                {   
                    Debug.Log("Steer angle: " + steerAngle);
                    // Compute new values
                    float actualAngle = highestPriorityNode.theta + steerAngle;
                    Debug.Log("Actual angle: " + actualAngle);
                    float dx = (float)Math.Cos(actualAngle)*(float)Math.Sqrt(2);
                    Debug.Log("Change in x: " + dx);
                    float dz = (float)Math.Sin(actualAngle)*(float)Math.Sqrt(2);
                    Debug.Log("Change in z: " + dz);
                    Node sucessor = new Node(highestPriorityNode.x + dx, highestPriorityNode.z + dz, actualAngle, calculateGridIndex(highestPriorityNode.x + dx, highestPriorityNode.z + dz), 0, 0, 0, highestPriorityNode);
                    Debug.Log("x of precessor: " + highestPriorityNode.x);
                    Debug.Log("z of precessor: " + highestPriorityNode.z);
                    Debug.Log("Grid index of precessor: " + highestPriorityNode.gridIdx);
                    Debug.Log("x of sucessor: " + sucessor.x);
                    Debug.Log("z of sucessor: " + sucessor.z);
                    Debug.Log("Grid index of sucessor: " + sucessor.gridIdx);

                    // Check traversability
                    if(obstacle_map[(int)Math.Round((sucessor.x - x_low) / 1),(int)Math.Round((sucessor.z - z_low) / 1)] == 1)
                    {
                        continue;
                    }                    
                    
                    // Check if the sucessor is expanded
                    if (!closedSet.ContainsKey(sucessor.gridIdx))
                    {
                        sucessor.g = highestPriorityNode.g + 1;
                        bool flag = false;
                        
                        foreach(Node one in openSet)
                        {   
                            // Check if the sucessor has a neighbour in the same cell
                            if(one.gridIdx == sucessor.gridIdx)
                            {   
                                // Remove the neighbour from the cell if the sucessor has lower cost
                                if (sucessor.g < one.g)
                                {   
                                    flag = true;
                                    openSet.Remove(one);
                                    break;
                                }
                            }
                        }
                        

                        if(!openSet.Contains(sucessor) || flag)
                        {   
                            // Add the sucessor to the open list if it is not there or it has lower cost than a currently existing node in the same cell
                            sucessor.h = calculateEuclidean(sucessor.x, sucessor.z, goalNode.x, goalNode.z);
                            sucessor.f = sucessor.g + sucessor.h;
                            openSet.Enqueue(sucessor, sucessor.f);
                        }
                    }
                }
            }
            
            return null;
        }
    }

    public class Node : FastPriorityQueueNode
    {
        public float x { get; set; }
        public float z { get; set; }
        public float theta { get; set; }
        public float g { get; set; }
        public float h { get; set; }
        public float f { get; set; }
        public int gridIdx { get; set;}
        public Node parent { get; set; }

        public Node(float x, float z, float theta, int gridIdx, float g = 0, float h = 0, float f = 0, Node parent = null)
        {
            this.x = x;
            this.z = z;
            this.theta = theta;
            this.g = g;
            this.h = h;
            this.f = g+h;
            this.gridIdx = gridIdx;
            this.parent = parent;
        }
    }
}