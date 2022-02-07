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

        // The start position
        Vector3 start_pos;

        // The goal position
        Vector3 goal_pos;

        // Our planned path
        List<Node> my_path = new List<Node>();

        float minDist;

        int minDistIdx;

        int Idx;

        Vector3 pos;

        Vector3 difference;

        float dist;

        Node target;

        Node aheadOfTarget;

        int lookahead = 4;

        Vector3 target_position;

        Vector3 aheadOfTarget_pos;

        Vector3 target_velocity;

        Vector3 position_error;

        Vector3 velocity_error;

        Vector3 desired_acceleration;

        float steering;

        float acceleration;

        // Proportional gain to PD-controller
        public float k_p = 2f;

        // Derivative gain to PD-controller
        public float k_d = 0.5f;

        // Stuck condition
        public bool is_stuck = false;

        // Starting out condition
        public bool starting_out = true;

        // Timer
        public float timer = 0;

        // Another timer
        public float reverse_timer = 0;

        // Old acceleration
        public bool counting = false;

        // Null vector
        Vector3 null_vector = new Vector3(0,0,0);

        public float old_acc_amp = 0;

    
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
            Debug.Log("Planning path");
            Node goalNode = planner.HybridAStar(terrain_manager, m_Car, start_pos, (float)Math.PI/2, goal_pos, obstacle_map, 10000);
            
            if(goalNode != null)
            {
                Debug.Log("Path found");
            }
            else
            {
                Debug.Log("Path failed");
            }

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
                Debug.DrawLine(old_wp, wp, Color.red, 100f);
                old_wp = wp;
            }

            Debug.Log("Tracking path");
        }

        private void FixedUpdate()
        {
            // Execute your path here
            // Get waypoint
            // Find closest point on path

            if(!is_stuck)
            {   
                // Finding closest node on path
                minDist = 100;
                minDistIdx = 0;
                Idx = 0;

                foreach(Node node in my_path)
                {
                    pos = new Vector3(node.x, 0, node.z);
                    difference = pos - transform.position;
                    dist = (float) Math.Sqrt( Math.Pow(difference.x,2) + Math.Pow(difference.z, 2) );
                    
                    if(dist < minDist)
                    {
                        minDist = dist;
                        minDistIdx = Idx;
                    }
                    Idx += 1;
                }

                // Decide on lookahead
                lookahead = 4;

                // Break condition
                try
                {
                    target = my_path[minDistIdx + lookahead];
                    aheadOfTarget = my_path[minDistIdx + lookahead + 4];
                }
                catch(ArgumentOutOfRangeException e)
                {   
                    Debug.Log("Goal reached");
                    this.enabled = false;
                }

                // Keep track of target position and velocity
                target_position = new Vector3(target.x, 0, target.z);
                aheadOfTarget_pos = new Vector3(aheadOfTarget.x, 0, aheadOfTarget.z);
                target_velocity = aheadOfTarget_pos-target_position;

                // a PD-controller to get desired velocity
                float my_speed = (float) Math.Sqrt( Math.Pow(my_rigidbody.velocity.x, 2) + Math.Pow(my_rigidbody.velocity.z, 2) );
                //Debug.Log("My speed: " + my_speed);
                position_error = target_position - transform.position;
                velocity_error = target_velocity - my_rigidbody.velocity;
                desired_acceleration = k_p * position_error + k_d * velocity_error;
                float acc_amp = (float) Math.Sqrt( Math.Pow(desired_acceleration.x, 2) + Math.Pow(desired_acceleration.z, 2) );
                float acc_change = acc_amp - old_acc_amp;
                old_acc_amp = acc_amp;

                float steering = Vector3.Dot(desired_acceleration, transform.right);
                float acceleration = Vector3.Dot(desired_acceleration, transform.forward);

                m_Car.Move(steering, acceleration, acceleration, 0f);

                if(my_speed < 0.1 && acc_change < 0.0001)
                {   
                    if(timer <= Time.time && !counting)
                    {   
                        counting = true;
                        timer = Time.time + 1;
                    }

                    if(Time.time > timer)
                    {   
                        Debug.Log("Car stuck detected");
                        counting = false;
                        is_stuck = true;
                    }
                }
            }
            else
            {   
                if(reverse_timer <= Time.time && !counting)
                {   
                    counting = true;
                    Debug.Log("Getting unstuck");
                    reverse_timer = Time.time + 1;
                }

                if(Time.time > reverse_timer)
                {   
                    counting = false;
                    is_stuck = false;
                }

                // Finding closest node on path
                minDist = 100;
                minDistIdx = 0;
                Idx = 0;
                foreach(Node node in my_path)
                {
                    pos = new Vector3(node.x, 0, node.z);
                    difference = transform.position - pos;
                    dist = (float) Math.Sqrt( Math.Pow(difference.x,2) + Math.Pow(difference.z, 2) );
                    if(dist < minDist)
                    {
                        minDist = dist;
                        minDistIdx = Idx;
                    }

                    Idx += 1;
                }

                // Decide on lookahead
                lookahead = 4;

                // Break condition
                try
                {
                    target = my_path[minDistIdx - lookahead];
                }
                catch(ArgumentOutOfRangeException e)
                {   
                    ;
                }

                // Keep track of target position and velocity
                target_position = new Vector3(target.x, 0, target.z);
                target_velocity = null_vector;

                // a PD-controller to get desired velocity
                position_error = target_position - transform.position;
                velocity_error = target_velocity - my_rigidbody.velocity;
                desired_acceleration = k_p * position_error + k_d * velocity_error;

                steering = Vector3.Dot(desired_acceleration, transform.right);
                acceleration = Vector3.Dot(desired_acceleration, transform.forward);

                m_Car.Move(steering, acceleration, acceleration, 0f);
            }
        }
    }
}
