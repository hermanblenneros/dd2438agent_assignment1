﻿using System.Collections;
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
        //new items
        List<Vector3> OpenList = new List<Vector3>();
        List<Vector3> CloseList = new List<Vector3>();
        //
        //new fuction
        public bool leastcost(Vector3 node)//need to code
        {
            if (true)
            {
                return true;
            }
            return false;
        }
        private Vector3 RoundState(Vector3 point)
        {

            return null;
        }
        //new fuction end

        private void Start()
        {
            // get the car controller
            m_Car = GetComponent<CarController>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();

            // Plan your path here
            // Replace the code below that makes a random path
            // ...

            Vector3 start_pos = terrain_manager.myInfo.start_pos;
            Vector3 goal_pos = terrain_manager.myInfo.goal_pos;

            List<Vector3> my_path = new List<Vector3>();

            my_path.Add(start_pos);

            //hybrid A* 
            List<Vector3> result = new List<Vector3>();
            OpenList.Add(start_pos);
            Vector3 pred = new Vector3();
            Vector3 succ = new Vector3();

            while (OpenList!=null)
            {
                Vector3 pred = OpenList.Find(leastcost);
                OpenList.Remove(pred);
                if(CloseList.Contains(pred))
                    continue;
                else
                    CloseList.Add(pred);
                succ = RoundState(pred);
                if (succ == RoundState(goal_pos))
                {
                    my_path.Add(succ);
                    break;
                }
                else
                {
                    //Search with dubin or reed-shepp
                    if (true)
                    {
                        succ = Dubin()
                    }
                    for(int i = 0; i < 6; i++)
                    {
                        Vector3 succ = pred + new Vector3(UnityEngine.Random.Range(-50.0f, 50.0f), 0, UnityEngine.Random.Range(-30.0f, 30.0f));
                        //ensure successor is on grid and traversable
                        if (succ)
                        {
                            if (!CloseList.Contains(succ))
                            {
                                succ = Update();

                                //if the successor is in the same cell but the C value is larger
                                if (true)
                                { }
                                //if successor is in the same cell and the C value is lower, set predecessor to predecessor of predecessor
                                else if (false) 
                                { }

                                OpenList.Add(succ);

                            }

                        }
                    }

                }

            }



            //
            for (int i = 0; i < 3; i++)
            {
                Vector3 waypoint = start_pos + new Vector3(UnityEngine.Random.Range(-50.0f, 50.0f), 0, UnityEngine.Random.Range(-30.0f, 30.0f));
                my_path.Add(waypoint);
            }
            my_path.Add(goal_pos);


            // Plot your path to see if it makes sense
            // Note that path can only be seen in "Scene" window, not "Game" window
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

            // this is how you access information about the terrain from the map
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

        }


    }
}
