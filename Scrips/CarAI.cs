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

        PurePursuit purepursuit;
        Node2 track_Node = null;
       // List<Node2> track_result = new List<Node2>();

        public Vector3 target_velocity;
        Vector3 old_target_pos;
        Vector3 desired_velocity;

        public float k_p = 2f;
        public float k_d = 0.5f;

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
            if(track_Node == null )
            {
                //Debug.Log("Track start!!!: " + m_Car.CurrentSpeed);
                Node2 start = new Node2(start_pos.x, start_pos.z,0,0 ,(float)Math.PI / 2, m_Car.CurrentSpeed);
                purepursuit = new PurePursuit(my_path);
                track_Node = purepursuit.PurePursuitA(start);
              
                // keep track of target position and velocity
                Vector3 target_position = new Vector3(track_Node.x, 0, track_Node.z);
                Vector3 target_velocity = (target_position - start_pos) / Time.fixedDeltaTime;
                old_target_pos = target_position;

                // a PD-controller to get desired velocity
                Vector3 my_velocity = new Vector3(track_Node.v * (float)Math.Cos(track_Node.theta), 0, track_Node.v * (float)Math.Sin(track_Node.theta));
                Vector3 position_error = target_position - transform.position;
                Vector3 velocity_error = target_velocity - my_velocity;
                Vector3 desired_acceleration = k_p * position_error + k_d * velocity_error;
                //Vector3 desired_acceleration = new Vector3(0,0,0);

                float steering = Vector3.Dot(desired_acceleration, transform.right);
                float acceleration = Vector3.Dot(desired_acceleration, transform.forward);

                // this is how you control the car
                Debug.Log("Steering:" + steering + " Acceleration:" + acceleration);
                m_Car.Move(steering, acceleration, acceleration, 0f);

                /*
               //draw the yellow line for tracking path
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
               */
            }
            else
            {
                Node2 now = new Node2(transform.position.x, transform.position.z, 0, 0, m_Car.CurrentSteerAngle, m_Car.CurrentSpeed);
                track_Node = purepursuit.PurePursuitA(track_Node);
                Vector3 target_position = new Vector3(track_Node.x, 0, track_Node.z);
                Vector3 target_velocity = (target_position - old_target_pos) / Time.fixedDeltaTime;
                old_target_pos = target_position;

                // a PD-controller to get desired velocity
                Vector3 my_velocity = new Vector3(track_Node.v * (float)Math.Cos(track_Node.theta), 0, track_Node.v * (float)Math.Sin(track_Node.theta));
                Vector3 position_error = target_position - transform.position;
                Vector3 velocity_error = target_velocity - my_velocity;
                Vector3 desired_acceleration = k_p * position_error + k_d * velocity_error;

                float steering = Vector3.Dot(desired_acceleration, transform.right);
                float acceleration = Vector3.Dot(desired_acceleration, transform.forward);

                Debug.DrawLine(target_position, target_position + target_velocity, Color.red);
                Debug.DrawLine(transform.position, transform.position + my_velocity, Color.blue);
                Debug.DrawLine(transform.position, transform.position + desired_acceleration, Color.black);

                // this is how you control the car
                Debug.Log("Steering:" + steering + " Acceleration:" + acceleration);
                m_Car.Move(steering, acceleration, acceleration, 0f);
            
            }


        }
    }
}
