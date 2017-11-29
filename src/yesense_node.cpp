//////////////////////////////////////////////
//* This is ros driver for Yesense IMU
//* Author: ZheWang
//* Email:wangzhe936@outlook.com
//* Created on 2017.11.28
//////////////////////////////////////////////

#include <iostream>
#include <serial/serial.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>


class Yesense{
    
    public:
        Yesense(const std::string port, uint32_t baudrate, std::string frameID);
        ~Yesense();
        bool checkAndOpen();
        sensor_msgs::Imu processDataToMsg();
        void DecodeIMUData(unsigned char *reTemp);
        void readData(int size,unsigned char * data);
    private:       
        sensor_msgs::Imu imu;
        serial::Serial yesense_port;
        float accl[3], angv[3], ang[3];
    public:
        bool mutex;
    
};

Yesense::Yesense(const std::string port, uint32_t baudrate, std::string frameID)
    :mutex(false)
{
    imu.header.frame_id = frameID;
    yesense_port.setPort(port);
    yesense_port.setBaudrate(baudrate);
}

Yesense::~Yesense()
{
    yesense_port.close();
}

bool Yesense::checkAndOpen()
{
    if(yesense_port.isOpen())
    {
        ROS_WARN("Another program is using this port by this has stopped it's use!");
        yesense_port.close();
    }
    yesense_port.open();
    return true;
}

sensor_msgs::Imu Yesense::processDataToMsg()
{
    unsigned char headerAndTail[7];
    unsigned char data[8][18];
    yesense_port.read(headerAndTail,1);
    if (headerAndTail[0]==0x59) 
    {		
        // ROS_WARN("NEW MSG!");
        mutex = true;
        readData(5,&headerAndTail[1]);
        for(int j = 0; j < 6; j++)
        {
                readData(14,data[j]);
        }
        for(int j = 6; j < 8; j++)
        {
                readData(18,data[j]);
        }
        
        DecodeIMUData(data[0]);
        DecodeIMUData(data[3]);
        DecodeIMUData(data[5]);

        yesense_port.read(&headerAndTail[6], 1); 
    }
    return imu;
}

void Yesense::DecodeIMUData(unsigned char *reTemp)
{
    switch(reTemp[0])
    {
           case 0x10:
               accl[0] = (long(reTemp [5]<<24| reTemp [4]<<16| reTemp [3]<<8|reTemp [2]))*0.000001;
               accl[1] = (long(reTemp [9]<<24| reTemp [8]<<16| reTemp [7]<<8|reTemp [6]))*0.000001;
               accl[2] = (long(reTemp [13]<<24| reTemp [12]<<16| reTemp [11]<<8|reTemp [10]))*0.000001;
            //    ROS_INFO("Acceleration x %f y %f z %f", accl[0], accl[1], accl[2]);
               imu.header.stamp = ros::Time::now();
               imu.linear_acceleration.x = accl[0];
               imu.linear_acceleration.y = accl[1];
               imu.linear_acceleration.z = accl[2];
               imu.linear_acceleration_covariance[0] = accl[0];
               imu.linear_acceleration_covariance[4] = accl[1];
               imu.linear_acceleration_covariance[8] = accl[2];
               break;
           case 0x20:
               angv[0] = (long(reTemp [5]<<24| reTemp [4]<<16| reTemp [3]<<8|reTemp [2]))*0.000001;
               angv[1] = (long(reTemp [9]<<24| reTemp [8]<<16| reTemp [7]<<8|reTemp [6]))*0.000001;
               angv[2] = (long(reTemp [13]<<24| reTemp [12]<<16| reTemp [11]<<8|reTemp [10]))*0.000001;
            //    ROS_INFO("Angular velocity x %f y %f z %f", angv[0], angv[1], angv[2]);
               imu.angular_velocity.x = angv[0];
               imu.angular_velocity.y = angv[1];
               imu.angular_velocity.z = angv[2];
               imu.angular_velocity_covariance[0] = angv[0];
               imu.angular_velocity_covariance[4] = angv[1];
               imu.angular_velocity_covariance[8] = angv[2];
               break;
           case 0x40:
               ang[0] = (long(reTemp [5]<<24| reTemp [4]<<16| reTemp [3]<<8|reTemp [2]))*0.000001;
               ang[1] = (long(reTemp [9]<<24| reTemp [8]<<16| reTemp [7]<<8|reTemp [6]))*0.000001;
               ang[2] = (long(reTemp [13]<<24| reTemp [12]<<16| reTemp [11]<<8|reTemp [10]))*0.000001;
            //    ROS_INFO("Angle x %f y %f z %f", ang[0], ang[1], ang[2]);
               float fCosHRoll = cos(ang[0] * .5f);
               float fSinHRoll = sin(ang[0] * .5f);
               float fCosHPitch = cos(ang[1] * .5f);
               float fSinHPitch = sin(ang[1] * .5f);
               float fCosHYaw = cos(ang[2] * .5f);
               float fSinHYaw = sin(ang[2] * .5f);
               imu.orientation.w = fCosHRoll * fCosHPitch * fCosHYaw + fSinHRoll * fSinHPitch * fSinHYaw;
               imu.orientation.x = fCosHRoll * fSinHPitch * fCosHYaw + fSinHRoll * fCosHPitch * fSinHYaw;
               imu.orientation.y = fCosHRoll * fCosHPitch * fSinHYaw - fSinHRoll * fSinHPitch * fCosHYaw;
               imu.orientation.z = fSinHRoll * fCosHPitch * fCosHYaw - fCosHRoll * fSinHPitch * fSinHYaw;
               imu.orientation_covariance[0] = ang[0];
               imu.orientation_covariance[4] = ang[1];
               imu.orientation_covariance[8] = ang[2];
               break;
    }
}

void Yesense::readData(int size,unsigned char * data)
{
    int head = 0, end = 0 ,size_temp = size;
    while(head<size)
    {
            end = yesense_port.read(&data[head], size_temp); 
            if(end == size_temp)
                    head += size_temp;
            else if(end != EOF){
                int mid = head;
                head += end;
                if((head + size_temp) >= size)
                    size_temp = size-head;
            }
    }
}

int main(int argc, char ** argv)
{
    ros::init(argc,argv,"Yesense_node");
    ros::NodeHandle n("~");
    std::string port,frame_id;
    int baudrate;
    if(!n.getParam("port", port))
        port = "/dev/ttyUSB0";
    if(!n.getParam("baudrate", baudrate))
        baudrate = 460800;
    if(!n.getParam("frame_id", frame_id))
        frame_id = "imu";
    Yesense yesense(port,baudrate,frame_id);
    yesense.checkAndOpen();
    ros::Publisher imu_data_pub = n.advertise<sensor_msgs::Imu>("/Yesense/imu_data", 10);
    ros::Rate loopRate(150);//This rate can't lower than 110
    while(ros::ok())
    {
        sensor_msgs::Imu imu_msg = yesense.processDataToMsg();
        if(yesense.mutex)
        {
            imu_data_pub.publish(imu_msg);
            yesense.mutex = false;
        }
        ros::spinOnce();
        loopRate.sleep();
    }
    return 0;
}
