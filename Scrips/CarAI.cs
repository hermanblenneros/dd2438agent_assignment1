using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;


namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI : MonoBehaviour
    {   
        // The car controller
        private CarController m_Car;

        // The rigid body of the car
        Rigidbody my_rigidbody;

        // The terrain manager object
        public GameObject terrain_manager_game_object;

        // The terrain manager
        TerrainManager terrain_manager;

        // Our planned path
        List<Node> my_path = new List<Node>();

        // The start position
        Vector3 start_pos;

        // The goal position
        Vector3 goal_pos;

        // A target velocity
        public Vector3 target_velocity;

        public Node target;

        public Node aheadOfTarget;

        // Proportional gain to PD-controller
        public float k_p = 2f;

        // Derivative gain to PD-controller
        public float k_d = 0.5f;
    
        private void Start()
        {
            // Get the car controller
            m_Car = GetComponent<CarController>();

            // Get the terrain manager
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();

            // Get the rigid body
            my_rigidbody = GetComponent<Rigidbody>();

            // Get start and goal position
            start_pos = terrain_manager.myInfo.start_pos;
            goal_pos = terrain_manager.myInfo.goal_pos;
            
            // Create mapper and compute obstacle map
            Mapper mapper = new Mapper(terrain_manager);
            float[,] obstacle_map = mapper.configure_obstacle_map(terrain_manager);

            // Create planner and find path
            Planner planner = new Planner();
            Node goalNode = planner.HybridAStar(terrain_manager, m_Car, start_pos, (float)Math.PI/2, goal_pos, obstacle_map, 10000);
            
            // Construct path
            my_path.Add(goalNode);
            Node parent = goalNode.parent;
            while(parent != null)
            {  
                my_path.Add(parent);
                parent = parent.parent;
            }
            
            my_path.Reverse();

            // Plot path
            Vector3 old_wp = start_pos;
            foreach (Node n in my_path)
            {   
                Vector3 wp = new Vector3(n.x, 0, n.z);
                Debug.DrawLine(old_wp, wp, Color.yellow, 100f);
                old_wp = wp;
            }
        }

        private void FixedUpdate()
        {
            // Execute your path here
            // Get waypoint
            // Find closest point on path

            float minDist = 100;
            int minDistIdx = 0;
            int Idx = 0;
            foreach(Node n in my_path)
            {   
                Vector3 pos = new Vector3(n.x, 0, n.z);
                Vector3 difference = transform.position - pos;
                float dist = (float)Math.Sqrt( Math.Pow(difference.x,2) + Math.Pow(difference.z, 2) );

                if(dist < minDist)
                {
                    minDist = dist;
                    minDistIdx = Idx;
                }

                Idx += 1;
            }

            // Move along the path a distance equal to the lookahead
            int lookahead = 4; // Lookahead in meters equals 4 * sqrt(2) = 5

            // Break condition
            try
            {
                target = my_path[minDistIdx + lookahead];
                aheadOfTarget = my_path[minDistIdx + lookahead + 1];
            }
            catch(ArgumentOutOfRangeException e)
            {
                Debug.Log("Congratz! You have reached the end!");
            }
            
            // Keep track of target position and velocity
            float speed = 1;
            Vector3 target_position = new Vector3(target.x, 0, target.z);
            Vector3 aheadOfTarget_pos = new Vector3(aheadOfTarget.x, 0, aheadOfTarget.z);
            target_velocity = speed*(aheadOfTarget_pos-target_position); // Velocity will be 5 * sqrt(2)
            Debug.Log("Target position: " + target_position);
            Debug.Log("Target position + target velocity: " + (target_position + target_velocity));

            // a PD-controller to get desired velocity
            Vector3 position_error = target_position - transform.position;
            Vector3 velocity_error = target_velocity - my_rigidbody.velocity;
            Vector3 desired_acceleration = k_p * position_error + k_d * velocity_error;

            float steering = Vector3.Dot(desired_acceleration, transform.right);
            float acceleration = Vector3.Dot(desired_acceleration, transform.forward);

            Debug.DrawLine(target_position, target_position + target_velocity, Color.red);
            Debug.DrawLine(transform.position, transform.position + my_rigidbody.velocity, Color.blue);
            Debug.DrawLine(transform.position, transform.position + desired_acceleration, Color.black);

            // this is how you control the car
            Debug.Log("Steering:" + steering + " Acceleration:" + acceleration);
            m_Car.Move(steering, acceleration, acceleration, 0f);
        }
    }
}
