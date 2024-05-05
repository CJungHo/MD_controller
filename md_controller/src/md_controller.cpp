#include "md_controller/com.hpp"

Communication Com;  
MotorVar Motor;

geometry_msgs::msg::TransformStamped odom_tf;
sensor_msgs::msg::JointState joint_states;

BYTE SendCmdRpm = OFF;
int rpm_ = 0;

void CmdRpmCallBack(const std_msgs::msg::Int32::SharedPtr msg) {
    rpm_ = msg->data;

    SendCmdRpm = ON;
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("md_controller_node");

    // create TF
    rclcpp::Time stamp_now;
    tf2_ros::TransformBroadcaster tf_broadcaster_(node);

    //subscriber
    auto rpm_sub = node->create_subscription<std_msgs::msg::Int32>("/cmd_rpm",1000, CmdRpmCallBack);

    //Motor driver settup-------------------------------------------------------------------------------
    node->declare_parameter("MDUI", 184);
    node->declare_parameter("MDT", 183);
    node->declare_parameter("Port", "/dev/ttyUSB0");
    node->declare_parameter("Baudrate", 57600);
    node->declare_parameter("ID", 1);
    node->declare_parameter("GearRatio", 15);
    node->declare_parameter("poles", 10);

    node->get_parameter("MDUI", Com.nIDMDUI);
    node->get_parameter("MDT", Com.nIDMDT);
    node->get_parameter("Port", Com.nPort);
    node->get_parameter("Baudrate", Com.nBaudrate);
    node->get_parameter("ID", Motor.ID);
    node->get_parameter("GearRatio", Motor.GearRatio);
    node->get_parameter("poles", Motor.poles);

    Motor.PPR       = Motor.poles*3*Motor.GearRatio;           //poles * 3(HALL U,V,W) * gear ratio
    Motor.Tick2RAD  = (360.0/Motor.PPR)*PI / 180;

    IByte iData;
    int nArray[2];
    static BYTE fgInitsetting, byCntInitStep, byCntComStep, byCnt2500us, byCntStartDelay, byCntCase[5];
    
    byCntInitStep     = 1;
    Motor.InitMotor   = ON;
    fgInitsetting     = OFF;
    Motor.InitError   = 0;
    Motor.last_rad    = 0;
    Motor.last_tick   = 0;

    InitSerial();   //communication initialization in com.cpp
    while (rclcpp::ok()) {
        
        ReceiveDataFromController(Motor.InitMotor);
        if(++byCnt2500us == 50)
        {
            byCnt2500us = 0;
            
            if(fgInitsetting == ON)
            {
                switch(++byCntComStep)
                {
                case 1:{ //create tf & update motor position
                    geometry_msgs::msg::TransformStamped transformStamped;
                    transformStamped.header.stamp = node->now();
                    transformStamped.header.frame_id = "world";
                    transformStamped.child_frame_id = "motor_joint";

                    transformStamped.transform.translation.x = 0.0;
                    transformStamped.transform.translation.y = 0.0;
                    transformStamped.transform.translation.z = 0.15;

                    Motor.current_tick     = Com.position;

                    Motor.last_diff_tick   = Motor.current_tick - Motor.last_tick;
                    Motor.last_tick        = Motor.current_tick;
                    Motor.last_rad        += Motor.Tick2RAD * (double)Motor.last_diff_tick;
                    // printf("%f\n", Motor.Tick2RAD);

                    tf2::Quaternion q;
                    q.setRPY(0, 0, -Motor.last_rad);
                    transformStamped.transform.rotation.x = q.x();
                    transformStamped.transform.rotation.y = q.y();
                    transformStamped.transform.rotation.z = q.z();
                    transformStamped.transform.rotation.w = q.w();

                    tf_broadcaster_.sendTransform(transformStamped);
                    auto end = std::chrono::high_resolution_clock::now();
                    break;
                }
                case 2: //Control motor & request motor info
                    if(++byCntCase[byCntComStep] == TIME_100MS)
                    {
                        byCntCase[byCntComStep] = 0;

                        if(SendCmdRpm)
                        {
                            iData = Short2Byte(rpm_ * Motor.GearRatio); // #1,2 Wheel RPM
                            nArray[0] = iData.byLow;
                            nArray[1] = iData.byHigh;
                            PutMdData(PID_VEL_CMD, Com.nIDMDT, Motor.ID, nArray);

                            //n대의 모터드라이버에게 동시에 main data를 요청할 경우 data를 받을 때 데이터가 섞임을 방지.
                            nArray[0] = PID_MAIN_DATA;
                            PutMdData(PID_REQ_PID_DATA, Com.nIDMDT, Motor.ID, nArray);  // Main data request

                            SendCmdRpm = OFF;
                        }
                        else
                        {
                            //n대의 모터드라이버에게 동시에 main data를 요청할 경우 data를 받을 때 데이터가 섞임을 방지.
                            nArray[0] = PID_MAIN_DATA;
                            PutMdData(PID_REQ_PID_DATA, Com.nIDMDT, Motor.ID, nArray);  // Main data request
                            //------------------------------------------------------------------
                        }
                        
                    }
                    byCntComStep=0;
                    break;  
                }
            }
            else
            {
                if(byCntStartDelay <= 200) byCntStartDelay++;
                else
                {
                    switch (byCntInitStep)
                    {
                    case 1: //Motor connect check
                        nArray[0] = PID_MAIN_DATA;
                        PutMdData(PID_REQ_PID_DATA, Com.nIDMDT, Motor.ID, nArray);

                        if(Motor.InitMotor == ON)
                            Motor.InitError++;
                        else
                            byCntInitStep++;

                        if(Motor.InitError > 10){
                            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"ID %d MOTOR INIT ERROR!!", Motor.ID); 
                            return 0;
                        }
                    
                        break;
                    case 2:
                        byCntInitStep++;
                        break;

                    case 3: //Motor torque ON
                        nArray[0] = 0;
                        nArray[1] = 0;
                        PutMdData(PID_VEL_CMD, Com.nIDMDT, Motor.ID, nArray);

                        byCntInitStep++;
                        break;

                    case 4: //Motor POS reset
                        nArray[0] = 0;
                        PutMdData(PID_POSI_RESET, Com.nIDMDT, Motor.ID, nArray);
                        byCntInitStep++;
                        break;

                    case 5:
                        printf("========================================================\n\n");
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "MOTOR INIT END\n");
                        fgInitsetting = ON;

                        break;

                    }
                }
            }
        }

        rclcpp::spin_some(node);
    }

    rclcpp::shutdown();
    return 0;
}
