﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;


namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI : MonoBehaviour
    {   
        //// Declaration of variables

        // Unity variables
        private CarController m_Car;
        Rigidbody my_rigidbody;
        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;

        // Planning variables
        Vector3 start_pos;
        Vector3 goal_pos;
        List<Node> my_path = new List<Node>();

        // Tracking variables
        public float k_p = 2f, k_d = 0.5f;
        float to_path, to_target, distance, steering, acceleration, starting_timer = 0, stuck_timer = 0, reverse_timer = 0, break_timer = 0, old_acceleration = 0, new_acceleration, acceleration_change, my_speed = 0, old_angle, new_angle, angle_change, unstuck_error, old_unstuck_error = 100, unstuck_error_change, clearance, max_clearance = 0;
        int to_path_idx, to_target_idx, dummy_idx, dummy_idx2, lookahead = 0, my_max_speed = 25;
        bool starting_phase = true, is_stuck = false, is_breaking = false, counting = false, no_waypoint = true;
        Vector3 pos, difference, target_position, aheadOfTarget_pos, target_velocity, position_error, velocity_error, desired_acceleration, closest, null_vector = new Vector3(0,0,0);
        Node target, aheadOfTarget, closestNode;
        List<float> controls = new List<float>(), cum_angle_change = new List<float>();

        //// Definition of functions

        // Function that computes the appropriate maximum speed depending on the curvature ahead
        public int curvatureToSpeed(float curvature_ahead)
        {
            int multiple = (int)Math.Round( (Math.Abs( curvature_ahead / (Math.PI/7.2) )) );

            if(multiple > 2)
            {
                return (int)5;
            }
            else if(multiple > 1)
            {
                return (int)10;
            }
            else
            {
                return (int)30;
            }
        }

        // Function that computes the appropriate lookahead depending on the current speed
        public int speedToLookahead(float my_speed)
        {
            lookahead = (int)(4 + Math.Sqrt(my_speed));
            return lookahead;
        }

        // Start function is run once every time the CarAI script is invoked
        private void Start()
        {
            // Get stuff
            m_Car = GetComponent<CarController>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();
            my_rigidbody = GetComponent<Rigidbody>();
            start_pos = terrain_manager.myInfo.start_pos;
            goal_pos = terrain_manager.myInfo.goal_pos;
            
            // Create mapper and compute obstacle map
            Debug.Log("Creating obstacle map of current terrain");
            Mapper mapper = new Mapper(terrain_manager);
            float[,] obstacle_map = mapper.configure_obstacle_map(terrain_manager, (float)2);
            float[,] distance_map = mapper.configure_distance_map(obstacle_map);

            // Create planner and find path
            Debug.Log("Planning path");
            Planner planner = new Planner();
            Node goalNode = planner.HybridAStar(terrain_manager, m_Car, start_pos, (float)Math.PI/2, goal_pos, obstacle_map, distance_map, 10000);
            
            // Disable code if path not found
            if(goalNode == null)
            {
                Debug.Log("Pathing failed");
                this.enabled = false;
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

            // Removing abudant nodes from path
            DouglasPeucker dp = new DouglasPeucker();
            List<Node> dp_path = dp.DouglasPeuckerReduction(my_path, 1);
            Vector3 old_wpp = start_pos;
            foreach (Node n in dp_path)
            {
                Vector3 wp = new Vector3(n.x, 0, n.z);
                Debug.DrawLine(old_wpp, wp, Color.green, 100f);
                old_wpp = wp;
            }

            // Getting controls of path
            for (int i = 0; i < my_path.Count; i++)
            {   
                if(i == 0)
                {
                    old_angle = my_path[i].theta;
                    controls.Add(0);
                }

                new_angle = my_path[i].theta;
                angle_change = new_angle - old_angle;
                old_angle = new_angle;

                controls.Add(angle_change);
            }

            // Approximating cumulative forward angle changes in path
            for(int i = 0; i < my_path.Count - 3; i++)
            {
                cum_angle_change.Add(controls[i] + controls[i+1] + controls[i+2] + controls[i+3]);
            }
            
            // Going to FixedUpdate()
            Debug.Log("Tracking path");
        }

        private void FixedUpdate()
        {   

            // Starting out phase means no checking for stuck and no checking for breaking
            if(starting_phase)
            {   
                if(starting_timer < Time.time && !counting)
                {
                    starting_timer = Time.time + 5;
                    counting = true;
                }

                if(Time.time >= starting_timer)
                {
                    counting = false;
                    starting_phase = false;
                }

            }

            // Tracks the path generated by planner

            // Decide on lookahead based on speed;
            lookahead = speedToLookahead(my_speed);

            // Tracks forward along the path if not stuck
            if(!is_stuck)
            {   
                // Finding closest node on path
                to_path = 100;
                to_path_idx = 0;
                dummy_idx = 0;
                foreach(Node node in my_path)
                {
                    pos = new Vector3(node.x, 0, node.z);
                    difference = pos - transform.position;
                    distance = (float) Math.Sqrt( Math.Pow(difference.x,2) + Math.Pow(difference.z, 2) );
                    
                    if(distance < to_path)
                    {
                        to_path = distance;
                        to_path_idx = dummy_idx;
                    }
                    dummy_idx += 1;
                }

                // Saving data about node on path closest to the car
                closestNode = my_path[to_path_idx];
                closest = new Vector3(closestNode.x, 0, closestNode.z);
                difference = closest - transform.position;
                to_path = (float) Math.Sqrt( Math.Pow(difference.x,2) + Math.Pow(difference.z, 2) );

                // Finding target node on path
                to_target = 100;
                to_target_idx = 0;
                dummy_idx2 = 0;
                foreach(Node node in my_path)
                {
                    if(dummy_idx2 < to_path_idx)
                    {   
                        dummy_idx2 += 1;
                        continue;
                    }

                    pos = new Vector3(node.x, 0, node.z);
                    difference = pos - closest;
                    distance = (float) Math.Sqrt( Math.Pow(difference.x,2) + Math.Pow(difference.z, 2) );

                    if(Math.Abs(distance + to_path - lookahead) < to_target)
                    {
                        to_target = Math.Abs(distance + to_path - lookahead);
                        to_target_idx = dummy_idx2;
                    }
                    dummy_idx2 += 1;
                }

                // Break condition
                try
                {
                    target = my_path[to_target_idx];
                    aheadOfTarget = my_path[to_target_idx + 1];
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
                Debug.DrawLine(transform.position, target_position);
                Debug.DrawLine(transform.position, transform.position + 10*target_velocity, Color.black);

                // a PD-controller to get desired velocity
                position_error = target_position - transform.position;
                velocity_error = target_velocity - my_rigidbody.velocity;
                desired_acceleration = k_p * position_error + k_d * velocity_error;

                // Apply controls
                steering = Vector3.Dot(desired_acceleration, transform.right);
                acceleration = Vector3.Dot(desired_acceleration, transform.forward);
                
                if(!starting_phase)
                {
                    my_max_speed = curvatureToSpeed(cum_angle_change[to_target_idx + 2]);
                }


                if(!is_breaking)
                {
                    break_timer = 0;
                }
            
                if(my_speed > my_max_speed || is_breaking)
                {   

                    is_breaking = true;

                    if(break_timer == 0)
                    {
                        break_timer = Time.time + 1;
                    }
                    
                    m_Car.Move(steering, -acceleration, -acceleration, 0f);

                    if(Time.time > break_timer)
                    {
                        is_breaking = false;
                    }

                    if(cum_angle_change[to_target_idx] > Math.PI/3.6)
                    {   
                        break_timer = Time.time + 1;                     
                    }
                }

                else
                {
                    m_Car.Move(steering, acceleration, acceleration, 0f);
                }

                // State variables for stuck condition
                my_speed = (float) Math.Sqrt( Math.Pow(my_rigidbody.velocity.x, 2) + Math.Pow(my_rigidbody.velocity.z, 2) );
                new_acceleration = (float) Math.Sqrt( Math.Pow(desired_acceleration.x, 2) + Math.Pow(desired_acceleration.z, 2) );
                acceleration_change = new_acceleration - old_acceleration;
                old_acceleration = new_acceleration;

                // Check stuckness. If stuck -> go to else statement below
                if(my_speed < 0.3 && acceleration_change < 0.0001 && !starting_phase)
                {   
                    if(stuck_timer <= Time.time && !counting)
                    {   
                        counting = true;
                        stuck_timer = Time.time + 5;
                    }

                    if(Time.time > stuck_timer)
                    {   
                        Debug.Log("Car stuck detected");
                        counting = false;
                        is_stuck = true;
                    }
                }
            }

            // We are stuck
            else
            {   
                // Decide on lookbehind based on speed;
                lookahead = speedToLookahead(my_speed);

                if(no_waypoint)
                {
                    // Finding closest node on path
                    to_path = 100;
                    to_path_idx = 0;
                    dummy_idx = 0;
                    foreach(Node node in my_path)
                    {
                        pos = new Vector3(node.x, 0, node.z);
                        difference = transform.position - pos;
                        distance = (float) Math.Sqrt( Math.Pow(difference.x,2) + Math.Pow(difference.z, 2) );

                        if(distance < to_path)
                        {
                            to_path = distance;
                            to_path_idx = dummy_idx;
                        }

                        dummy_idx += 1;
                    }

                    // Saving data about node on path closest to the car
                    closestNode = my_path[to_path_idx];
                    closest = new Vector3(closestNode.x, 0, closestNode.z);
                    difference = closest - transform.position;
                    to_path = (float) Math.Sqrt( Math.Pow(difference.x,2) + Math.Pow(difference.z, 2) );

                    // Finding target node on path
                    to_target = 100;
                    to_target_idx = 0;
                    dummy_idx2 = 0;
                    foreach(Node node in my_path)
                    {
                        if(dummy_idx2 > to_path_idx)
                        {   
                            break;
                        }

                        pos = new Vector3(node.x, 0, node.z);
                        difference = pos - closest;
                        distance = (float) Math.Sqrt( Math.Pow(difference.x,2) + Math.Pow(difference.z, 2) );

                        if(Math.Abs(distance + to_path - lookahead) < to_target)
                        {
                            to_target = Math.Abs(distance + to_path - lookahead);
                            to_target_idx = dummy_idx2;
                        }
                        dummy_idx2 += 1;
                    }
                    no_waypoint = false;
                }

                // Break condition
                try
                {
                    target = my_path[to_target_idx];
                }
                catch(ArgumentOutOfRangeException e)
                {   
                    ;
                }

                // Keep track of target position and velocity
                target_position = new Vector3(target.x, 0, target.z);
                target_velocity = null_vector;
                Debug.DrawLine(transform.position, target_position);

                // a PD-controller to get desired velocity
                position_error = target_position - transform.position;
                velocity_error = target_velocity - my_rigidbody.velocity;
                desired_acceleration = k_p * position_error + k_d * velocity_error;

                // Apply controls
                steering = Vector3.Dot(desired_acceleration, transform.right);
                acceleration = Vector3.Dot(desired_acceleration, transform.forward);
                m_Car.Move(steering, acceleration, acceleration, 0f);

                // State variables for unstuck condition
                unstuck_error = (float) Math.Sqrt( Math.Pow(position_error.x, 2) + Math.Pow(position_error.z, 2) );
                unstuck_error_change = (float)Math.Abs(unstuck_error - old_unstuck_error);
                old_unstuck_error = unstuck_error;
                
                if(unstuck_error_change < 0.0001)
                {   
                    old_unstuck_error = 100;
                    is_stuck = false;
                    no_waypoint = true;
                }
            }
        }
    }
}
