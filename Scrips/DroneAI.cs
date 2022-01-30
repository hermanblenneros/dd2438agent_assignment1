using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(DroneController))]
public class DroneAI : MonoBehaviour
{

    private DroneController m_Drone; // the drone controller we want to use

    public GameObject terrain_manager_game_object;
    TerrainManager terrain_manager;

    private void Start()
    {
        // get the drone controller
        m_Drone = GetComponent<DroneController>();
        terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();

        // Getting start and goal pos
        Vector3 start_pos = terrain_manager.myInfo.start_pos;
        Vector3 goal_pos = terrain_manager.myInfo.goal_pos;

        // Accessing grid info
        float x_low = terrain_manager.myInfo.x_low;
        float x_high = terrain_manager.myInfo.x_high;
        int x_N = terrain_manager.myInfo.x_N;
        float x_res = (x_high - x_low)/x_N;
        float z_low = terrain_manager.myInfo.z_low;
        float z_high = terrain_manager.myInfo.z_high;
        int z_N = terrain_manager.myInfo.z_N;
        float z_res = (z_high - z_low)/z_N;
        float[,] travGrid = terrain_manager.myInfo.traversability;
        Debug.Log("x_low: " + x_low);
        Debug.Log("x_high: " + x_high);
        Debug.Log("x_N: " + x_N);
        Debug.Log("x_res: " + x_res);
        Debug.Log("z_low: " + z_low);
        Debug.Log("z_high: " + z_high);
        Debug.Log("z_N: " + z_N);
        Debug.Log("z_res: " + z_res);

        // Upsample traversability grid
        int[,] temp = new int[(int)(x_high-x_low), z_N];
        int[,] upsampGrid = new int[(int)(x_high-x_low), (int)(z_high-z_low)];
        // Upsample in the x-direction
        for(int i = 0; i < x_N; i++)
        {
            for(int j = 0; j < x_res; j++)
            {
                for(int k = 0; k < z_N; k++)
                {
                    float val = travGrid[i,k];
                    temp[j + (int)x_res*i, k] = (int)val;
                }
            }
        }
        // Upsample in the z-direction
        for(int i = 0; i < z_N; i++)
        {
            for(int j = 0; j < z_res; j++)
            {
                for(int k = 0; k < (int)(x_high-x_low); k++)
                {
                    float val = travGrid[k,i];
                    upsampGrid[k,j + (int)z_res*i] = (int)val;
                }
            }
        }
        // Dilate the obstacles
        int[,] dilatedObs = new int[(int)(x_high-x_low), (int)(z_high-z_low)];

        for(int i = 1; i < (int)(x_high-x_low) - 1; i++)
        {
            for(int j = 1; j < (int)(x_high-x_low) - 1; j++)
            {
                int neighbours = upsampGrid[i-1,j] + upsampGrid[i-1,j-1] + upsampGrid[i, j-1] + upsampGrid[i+1,j-1] + upsampGrid[i+1,j] + upsampGrid[i+1,j+1] + upsampGrid[i,j+1] + upsampGrid[i-1, j+1];
                if(neighbours >= 1)
                {
                    dilatedObs[i,j] = 1;
                }
            }
        }

        List<Vector3> my_path = new List<Vector3>();

        // Plan your path here
        // ...
        my_path.Add(start_pos);

        for (int i = 0; i < 3; i++)
        {
            Vector3 waypoint = start_pos + new Vector3(UnityEngine.Random.Range(-50.0f, 50.0f), 0, UnityEngine.Random.Range(-30.0f, 30.0f));
            my_path.Add(waypoint);
        }
        my_path.Add(goal_pos);



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

        // this is how you access information about the terrain
        int i = terrain_manager.myInfo.get_i_index(transform.position.x);
        int j = terrain_manager.myInfo.get_j_index(transform.position.z);
        float grid_center_x = terrain_manager.myInfo.get_x_pos(i);
        float grid_center_z = terrain_manager.myInfo.get_z_pos(j);

        Debug.DrawLine(transform.position, new Vector3(grid_center_x, 0f, grid_center_z), Color.white, 1f);

        // this is how you control the car
        m_Drone.Move(0.4f * Mathf.Sin(Time.time * 1.9f), 0.1f);

    }

 

    // Update is called once per frame
    void Update()
    {
        
    }
}
