using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;


namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI : MonoBehaviour
    {
        private CarController m_Car; // the car controller we want to use
        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;
        List<Vector3> my_path = new List<Vector3>();
        Vector3 start_pos;
        Vector3 goal_pos;
        public float speedchange = 0;
        public bool hasnext = false;

        List<Node2> track_Node = new List<Node2>();

        private void Start()
        {
            // Get the car controller
            m_Car = GetComponent<CarController>();

            // Get the terrain manager
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();

            // Get start and goal position
            start_pos = terrain_manager.myInfo.start_pos;
            goal_pos = terrain_manager.myInfo.goal_pos;
            
            // Create mapper and compute obstacle map
            Mapper mapper = new Mapper(terrain_manager);
            float[,] obstacle_map = mapper.configure_obstacle_map(terrain_manager);

            // Create planner and compute path
            Planner planner = new Planner();

            Node goalNode = planner.HybridAStar(terrain_manager, m_Car, start_pos, (float)Math.PI/2, goal_pos, obstacle_map, 10000);
            
            my_path.Add(goal_pos);
            
            Node parent = goalNode.parent;

            while(parent != null)
            {   
                Vector3 waypoint = new Vector3(parent.x, 0, parent.z);
                my_path.Add(waypoint);
                parent = parent.parent;
            }
            my_path.Add(start_pos);
            my_path.Reverse();

            // Plot your path to see if it makes sense
            Vector3 old_wp = start_pos;
            foreach (var wp in my_path)
            {
                Debug.DrawLine(old_wp, wp, Color.red, 100f);
                old_wp = wp;
            }
        }

        private void FixedUpdate()
        {
            // Execute your path here
            // ...
            if(track_Node.Count == 0 )
            {
                
                Debug.Log("Track start!!!: " + m_Car.CurrentSpeed);
                Node2 start = new Node2(start_pos.x, start_pos.z,0,0 ,(float)Math.PI / 2, m_Car.CurrentSpeed);
                PurePursuit pp = new PurePursuit(start, my_path);
                track_Node = pp.PurePursuitA();
                
                //draw the blue line for tracking path
                List<Vector3> track_path = new List<Vector3>();
                foreach (Node2 one in track_Node)
                {
                    Vector3 waypoint = new Vector3(one.x, 0, one.z);
                    track_path.Add(waypoint);
                }

                Vector3 old_wp = start_pos;
                foreach (var wp in track_path)
                {
                    Debug.DrawLine(old_wp, wp, Color.yellow, 100f);
                    old_wp = wp;
                }

               // Debug.Log("Track node size: " + track_Node.Count);

            }
            else
            {
               // Debug.Log("Track node size: " + track_Node.Count);
            }

            //m_Car.Move(1f, 1f, 1f, 0f);
            /*// this is how you access information about the terrain from the map
            int i = terrain_manager.myInfo.get_i_index(transform.position.x);
            int j = terrain_manager.myInfo.get_j_index(transform.position.z);
            float grid_center_x = terrain_manager.myInfo.get_x_pos(i);
            float grid_center_z = terrain_manager.myInfo.get_z_pos(j);

            Debug.DrawLine(transform.position, new Vector3(grid_center_x, 0f, grid_center_z));

            // this is how you access information about the terrain from a simulated laser range finder
            RaycastHit hit;
            float maxRange = 50f;
            if (Physics.Raycast(transform.position + transform.up, transform.TransformDirection(Vector3.forward), out hit, maxRange))
            {
                Vector3 closestObstacleInFront = transform.TransformDirection(Vector3.forward) * hit.distance;
                Debug.DrawRay(transform.position, closestObstacleInFront, Color.yellow);
                Debug.Log("Did Hit");
            }


            // this is how you control the car
            m_Car.Move(1f, 1f, 1f, 0f);
            */


        }
    }
}
