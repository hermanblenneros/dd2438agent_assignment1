using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;


public class Node
{
    public Vector3 pos { get; set; }
    public float cost { get; set; }
    public float angle { get; set; }
    public float velocity { get; set; }
    public Node ParentNode { get; set; }
    public List<Node> children { get; set; }
    public Node()
    {

    }
    public Node(Node ParentNode, Vector3 pos, float cost, float angle, float velocity)
    {
        this.ParentNode = ParentNode;
        this.pos = pos;
        this.cost = cost;
        this.velocity = velocity;
        this.angle = angle;
        this.children = new List<Node>();
    }
    public void addchild(Node child)
    {
        children.Add(child);
    }

    public void deletechild(Node child)
    {
        children.Remove(child);
    }

    public double CalcDist(Node one)
    {
        Vector3 posone = one.pos;
        var distance = Math.Sqrt((Math.Pow(pos.x - posone.x, 2) + Math.Pow(pos.z - posone.z, 2)));
        return distance;
    }
}
public class Graph
{
    public Node root{ get; set; }
    public List<Vector3> node_path;
    public Graph(Vector3 pos)
    {
        List<Vector3> root_path = new List<Vector3>();
        root_path.Add(pos);
        node_path = root_path;
        this.root = new Node(null, pos, 0, 0, Mathf.PI / 2);
    }
}

namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    [RequireComponent(typeof(DroneController))]
    public class HybridAstar
    {
        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;
        public Graph graph;
        Vector3 start_pos, goal_pos;
        public float x_min, x_max, z_min, z_max;

        Node node_best;
        List<Node> OpenList = new List<Node>();
        List<Node> CloseList = new List<Node>();
        List<Node> result = new List<Node>();
        public float maxSpeed;
        public float maxSteerAngle;
        public float t = 0.02f;
        public float leastcost = 1000;
        public int maxIter = 40000;
        public int iterations = 0;
        public int stepSize;
        public int nearRadius = 30;
        public List<Node> near;
        public float carLength = 3f;
        public int buffer = 2;
        public System.Random r = new System.Random(DateTime.Now.Millisecond);

        public HybridAstar(TerrainManager terrain_manager, CarController m_Car)
        {
            stepSize = (int)(5 / t);
            maxSpeed = m_Car.MaxSpeed;
            maxSteerAngle = m_Car.m_MaximumSteerAngle * Mathf.PI / 180;
            start_pos = terrain_manager.myInfo.start_pos;
            graph = new Graph(start_pos);
            goal_pos = terrain_manager.myInfo.goal_pos;
            x_min = terrain_manager.myInfo.x_low;
            x_max = terrain_manager.myInfo.x_high;
            z_min = terrain_manager.myInfo.z_low;
            z_max = terrain_manager.myInfo.z_high;

            OpenList.Add(new Node(null, start_pos, 0, 0, Mathf.PI / 2));
            result.Add(new Node(null, start_pos, 0, 0, Mathf.PI / 2));
            Node one = null;
            Node child = null;

            while (OpenList != null)
            {
                foreach (Node n in OpenList)
                {
                    if (n.cost< leastcost)
                    {
                        leastcost = n.cost;
                        one = n;
                    }
                }
                OpenList.Remove(one);
                iterations++;
                if (CloseList.Contains(one))
                    continue;
                else
                    CloseList.Add(one);

                if (one == RoundState(new Node(null, goal_pos, 0, 0, Mathf.PI / 2))|| iterations> maxIter)
                {
                    //result.Add(one);
                    break;
                }
                else
                {
                    //Search with dubin or reed-shepp
                    if (CollisionCheck())
                    {
                        child = AnalysticExpantion(one);
                        if(child!=null)
                        {
                            OpenList.Add(child);
                            continue;
                        }
                    }
                    for (int i = 0; i < 6; i++)
                    {
                        Vector3 next = one.pos + new Vector3(UnityEngine.Random.Range(-50.0f, 50.0f), 0, UnityEngine.Random.Range(-30.0f, 30.0f));
                        child = new Node(null, next, 0, 0, Mathf.PI / 2);
                        //ensure successor is on grid and traversable
                        if (child!=null)
                        {
                            if (!CloseList.Contains(child))
                            {
                                child = Update(child);

                                //if the successor is in the same cell but the C value is larger
                                if (true)
                                { 

                                }
                                //if successor is in the same cell and the C value is lower, set predecessor to predecessor of predecessor
                                else if (false)
                                {
                                
                                }

                                OpenList.Add(child);

                            }

                        }
                    }

                }

            }
        }
        private Node RoundState(Node node)
        {

            return null;
        }

        private Node AnalysticExpantion(Node node)
        {

            return null;
        }


        //Heuristic fuction
        private Node Update(Node node)
        {

            return null;
        }
        private bool CollisionCheck()
        {

            return true;
        }
        public List<Node> Build()
        {
            //Node one = CloseList[CloseList.Count-1];

            List<Node> test = new List<Node>();
            Node start = new Node(null, start_pos, 0, 0, Mathf.PI / 2);
            test.Add(start);
            for (int i = 0; i < 3; i++)
            {
                Vector3 waypoint = start_pos + new Vector3(UnityEngine.Random.Range(-50.0f, 50.0f), 0, UnityEngine.Random.Range(-30.0f, 30.0f));
                test.Add(new Node(null, waypoint, 0, 0, Mathf.PI / 2));
            }
            Node goal = new Node(null, goal_pos, 0, 0, Mathf.PI / 2);
            test.Add(goal);
            return test;
        }
    }


}