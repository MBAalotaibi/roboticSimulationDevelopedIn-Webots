// MyLab5Controller.java
/*
 * MyLab5Controller Class Definition
 * File: MyLab5Controller.java
 * Date: 15th Oct 2022
 * Description: Simple Controller based on the Lab4 controller (2022)
 * Author: Terry Payne (trp@liv.ac.uk)
 */

import com.cyberbotics.webots.controller.Supervisor;
import com.cyberbotics.webots.controller.Camera;


import com.cyberbotics.webots.controller.PositionSensor;////Getting postion sensors to track the distance robot traveled using the number of turns
import java.lang.Math; 


public class MyLab5Controller {

  public static enum GlobalState {//Global states in robot for bug 1 
    GO_TO_GOAL,
    WALL_FOLLOW};

  public static void main(String[] args) {

    Supervisor robot = new Supervisor();
    int timeStep = (int) Math.round(robot.getBasicTimeStep());
    
    /////////////////////////////////////////////////////////////////
    ////////////////////////Setting  target and start coordinates
    PioneerNav2.MoveState state; // current naviagation state
    double[] start = {-4.5,3,0};
    double[] target = {2.74859 ,-3.25946 };
    GlobalState global_state = GlobalState.GO_TO_GOAL;///Setting starting state to go to goal
    state = PioneerNav2.MoveState.ARC;
    double  start_time = robot.getTime();/////getting start time
    
    /////////////////////////////////////////////////////////////////
    Pose robot_pose = new Pose(start[0], start[1], start[2]);///Giving starting coordinates to pose class
    //PioneerNav2 nav = new PioneerNav2(robot, robot_pose);
    //PioneerProxSensors1 prox_sensors = new PioneerProxSensors1(robot, "sensor_display", robot_pose);
    Camera camera = robot.getCamera("camera");
    
    if (camera != null)
      camera.enable(timeStep);
    //////////////////////////////////////////////////initailzing position sensors
    PositionSensor left_position_sensor = robot.getPositionSensor("left wheel sensor");
    left_position_sensor.enable(timeStep);
    PositionSensor right_position_sensor = robot.getPositionSensor("right wheel sensor");
    right_position_sensor.enable(timeStep);
    ///////////////////////////////////////////////////////////////////////
    PioneerProxSensors1 prox_sensors = new PioneerProxSensors1(robot, "sensor_display", robot_pose);
    PioneerNav2 nav = new PioneerNav2(robot, robot_pose, prox_sensors);

    double time_elapsed = 0;
    double target_time = 0;
    
    ////////////////////Second timer
    double time_elapsed_2 = 0;
    double target_time_2 = 0;
    ////////////////////////////////
    double robot_velocity = 0.3;
    
    /////////////////////////////////////////////////////
    double s_y = target[1] - robot_pose.getY();
    double s_x = target[0] - robot_pose.getX();
    double angle = Math.atan2(s_y, s_x) - robot_pose.getTheta();//Getting the heading to target
    
    ///////////////////////////////////////////////////
    // define schedule
    //PioneerNav2.MoveState[] schedule = { PioneerNav2.MoveState.FOLLOW_WALL };
    ////int schedule_index = -1; // we increment before selecting the current action
    
    
    double min_distance = Math.sqrt(Math.pow((target[0] - start[0]), 2) + Math.pow(target[1] - start[1], 2)); ///Variabble to save minimum distance
    double[] coords_min = {start[0],start[1]};//Coordintes at minimum distance found
    double distance_to_travel = min_distance;//Distance variable given to the move ford state
    double[] wall_follow_start_cord = {0,0}; //Location of wall follow starting
    
    double theta = 0;
    double cord_at_min_distance[] = {0,0};///Coordinates at the minimum distance to target
    
    
    
    
   
    while (robot.step(timeStep) != -1) {
      // Testing out the front proximity sensors
     
      robot_pose = nav.get_real_pose();
      prox_sensors.set_pose(robot_pose);
      prox_sensors.paint();  // Render sensor Display
      System.out.println("::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::");
      System.out.print("Global state :");
      System.out.println(global_state);
      System.out.print("Navigation state :");
      System.out.println(state);
      System.out.print("Distance traveled :");
      System.out.print(0.0957*(right_position_sensor.getValue() + left_position_sensor.getValue())/2);
      System.out.println(" meters");
      if (Math.sqrt(Math.pow((target[0] - robot_pose.getX()), 2) + Math.pow(target[1] - robot_pose.getY(), 2)) < 0.05)
      { ///If close to the target then termicate the code
           nav.stop();
           state = PioneerNav2.MoveState.STOP; 
           System.out.print("Time taken :");
           System.out.print(robot.getTime() - start_time);
           System.out.println(" seconds");
           System.out.print("Distance traveled :");
           System.out.print(0.0957*(right_position_sensor.getValue() + left_position_sensor.getValue())/2);
           System.out.println(" meters");
           break;
             }
      
      if (global_state == GlobalState.GO_TO_GOAL)//state where the robot turns and moves towards the goal
      {
        double y = target[1] - robot_pose.getY();
        double x = target[0] - robot_pose.getX();
        theta = Math.atan2(y, x) - robot_pose.getTheta();//Getting the heading difference with the target
        if (Math.abs(theta)>0.02)//IF the heading heading differentce and taget differntce is bigger then turning the robot
        {
          state = PioneerNav2.MoveState.ARC ;
        }else//Else moving towards the target
        {
          state = PioneerNav2.MoveState.FORWARD ;
          if (prox_sensors.get_value(4)  <0.2)//IF there is an obstacle ahead getting ready to do wallfollowing
          {
            time_elapsed = target_time+1; //Removing form the MoveState.FORWARD by completeing its time
            global_state = GlobalState.WALL_FOLLOW;//Changing global state to wall following
            
            target_time_2 = 3200; //Setting tareget time soo that the robot clears the starting area
            time_elapsed_2 = 0;
          }
        }
      } 
       if (global_state == GlobalState.WALL_FOLLOW)
      {
        state = PioneerNav2.MoveState.FOLLOW_WALL;//Setting navigation state to follow wall 
         if (time_elapsed_2 > target_time_2)  ///If time elapsed and found another coord where the angle is equal to the start angle then switching states
         { 
         
         double  a_y = target[1] - robot_pose.getY();
         double a_x = target[0] - robot_pose.getX();
         double alpha  = Math.atan2(a_y, a_x) ;//Getting the current angle with the target   
         if ( Math.abs(angle -alpha)<0.01)
         {
               global_state = GlobalState.GO_TO_GOAL;///Switching the state to go to goal since the robot is ready to start doing wall following again
        }
        }
        else
        {
           time_elapsed_2 += timeStep;  
        }
      }
      
      if (time_elapsed > target_time) {
        time_elapsed = 0;
            
        // select next action in schedule if not stopped
        //schedule_index = (schedule_index + 1) % schedule.length;
       
        if (state == PioneerNav2.MoveState.FOLLOW_WALL)
        {
        target_time = 0; // always refresh!
        nav.follow_wall(robot_velocity, 0.25, true);
        }else
        if (state == PioneerNav2.MoveState.FORWARD) {
          target_time = nav.forward(distance_to_travel, robot_velocity);
        } else 
        if (state == PioneerNav2.MoveState.ARC) {
          target_time = nav.arc(theta, 0.0, robot_velocity);
        } else
        if (state == PioneerNav2.MoveState.STOP) {
          nav.stop();
          target_time = 60 * 1000; // This doesn't really stop, but pauses for 1 minute
          
        }
        
      } else
        time_elapsed += timeStep;    // Increment by the time state
      
    };
    // Enter here exit cleanup code.
    nav.stop();
    robot.step(timeStep);
  }
  
}
