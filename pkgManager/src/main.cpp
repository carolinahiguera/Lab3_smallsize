#include <iostream>
#include <fstream>
#include <sys/types.h>
#include <sys/stat.h>
#include "robocup_ssl_client.h"
#include "timer.h"
#include "messages_robocup_ssl_detection.pb.h"
#include "messages_robocup_ssl_geometry.pb.h"
#include "messages_robocup_ssl_wrapper.pb.h"

using namespace std;

struct Geometry{
    int length;
    int width;
}field;
int num_cameras;

bool belongCamera(int cam_id, float pos_x, float pos_y);
void addBall_Pkg(SSL_DetectionBall ball, SSL_DetectionFrame* detectionFrame);
void addRobotYellow_Pkg(SSL_DetectionRobot robot, SSL_DetectionFrame* detectionFrame);
void addRobotBlue_Pkg(SSL_DetectionRobot robot, SSL_DetectionFrame* detectionFrame);
//-------------------------------------------------------------------------
int main(int argc, char *argv[])
{
    if(argc == 1){
        cerr << "Error: missing number of cameras --> " << argv[0] << " X" << endl;
        return 1;
    }
    ofstream fd_o;
    char *myfifo = "/tmp/ssl_fifo";
	  mkfifo(myfifo, 0666);
    //cout << "Waiting for client..." << endl;
    fd_o.open(myfifo, ofstream::out);
    //cout << "Client connected." << endl;
    num_cameras = atoi(argv[1]);
    bool have_geometry = false;
    bool iniFrame = true;
    int currentFrame, previousFrame=0;
    int cam_id;
    //Geometry field;
    RoboCupSSLClient client;
    SSL_WrapperPacket packet;
    SSL_WrapperPacket fullPacket;

    client.open(true);

    //wait until 1rst package has geometry data
    while(!have_geometry) {
        if (client.receive(packet)) {
            have_geometry = packet.has_geometry();
        }
    }
    //save geometry information
    const SSL_GeometryData & geom = packet.geometry();
    SSL_GeometryData geom1 = geom;
    geom1.CopyFrom(geom);
    const SSL_GeometryFieldSize & fieldPkg = geom.field();
    field.length = fieldPkg.field_length();
    field.width = fieldPkg.field_width();

    int tCount = 0;

    //receive packages
    SSL_DetectionFrame* detectionFrame = fullPacket.mutable_detection();
    while(true) {
        if (client.receive(packet)) {
            if(packet.has_detection()){
                SSL_DetectionFrame detection = packet.detection();
                cam_id = detection.camera_id();
                if(cam_id==0)
                  currentFrame = detection.frame_number();
                //cout << "frame: " << currentFrame << endl;
                //cout << "camera: " << cam_id << endl;
                //cout << "num balls: " << detection.balls_size() << endl;
                //cout << "num blues: " << detection.robots_blue_size() << endl;
                //cout << "num yellows: " << detection.robots_yellow_size() << endl << endl;
                if(previousFrame!=currentFrame && !iniFrame){
                    iniFrame = true;
                    if(!fd_o.is_open()){
                        fd_o.open(myfifo, ofstream::out);
                    }
                    if (!fullPacket.SerializeToOstream(&fd_o)) {
                      	cerr << "Failed to write package on FIFO." << endl;
                      	return -1;
                   	}
                    fd_o.close();
                    // cout << fullPacket.DebugString() << endl;
                    fullPacket.Clear();
                    detectionFrame = fullPacket.mutable_detection();
                }
                previousFrame = currentFrame;
                //save camera info
                iniFrame = false;
                detectionFrame->set_frame_number(detection.frame_number());
                detectionFrame->set_t_capture(detection.t_capture());
                detectionFrame->set_t_sent(0);
                detectionFrame->set_camera_id(0);
                //add to packet detected balls
                for (int i = 0; i < detection.balls_size(); i++) {
                    SSL_DetectionBall ball = detection.balls(i);
                    if(belongCamera(cam_id,ball.x(),ball.y())){
                        addBall_Pkg(ball, detectionFrame);
                    }
                }
                //add to packet detected yellow robots
                for (int i = 0; i < detection.robots_yellow_size(); i++) {
                    SSL_DetectionRobot robot = detection.robots_yellow(i);
                    if(belongCamera(cam_id,robot.x(),robot.y())){
                        addRobotYellow_Pkg(robot, detectionFrame);
                    }
                }
                //add to packet detected blue robots
                for (int i = 0; i < detection.robots_blue_size(); i++) {
                    SSL_DetectionRobot robot = detection.robots_blue(i);
                    if(belongCamera(cam_id,robot.x(),robot.y())){
                        addRobotBlue_Pkg(robot, detectionFrame);
                    }
                }
                /*if(packet.has_geometry()){
                  cout << "OJO" << endl;
                    if(!fullPacket.has_geometry()){
                        SSL_GeometryData *geo = fullPacket.mutable_geometry();
                        geo->CopyFrom(packet.geometry());
                    }
                }*/
                tCount++;
                if(tCount%180){
                  SSL_GeometryData *geo = fullPacket.mutable_geometry();
                  geo->CopyFrom(geom1);
                }
                //check if reached last camera to send packet

                if(cam_id==(num_cameras-1)){

//                    cout << "full packet" << endl;
//                    cout << "num balls: " << detectionFrame->balls_size() << endl;
//                    cout << "num blues: " << detectionFrame->robots_blue_size() << endl;
//                    cout << "num yellows: " << detectionFrame->robots_yellow_size() << endl << endl;

                    iniFrame = true;
                    if(!fd_o.is_open()){
                        fd_o.open(myfifo, ofstream::out);
                    }
                    if (!fullPacket.SerializeToOstream(&fd_o)) {
                      	cerr << "Failed to write package on FIFO." << endl;
                      	return -1;
                   	}
                    fd_o.close();
                    // cout << fullPacket.DebugString() << endl;
                    fullPacket.Clear();
                    detectionFrame = fullPacket.mutable_detection();
                }
            }
        }
    }

    return 0;
}

//-------------------------------------------------------------------------
bool belongCamera(int cam_id, float pos_x, float pos_y){
    if(num_cameras==1) return true;
    if(num_cameras==2){
        if(pos_x<0.0 && cam_id==0) return true;
        else if(pos_x>=0.0 && cam_id==1) return true;
        else return false;
    }
    if(num_cameras==4){
        if(cam_id==2 && pos_x<0.0 && pos_y<0.0) return true;
        else if(cam_id==3 && pos_x<0.0 && pos_y>=0.0) return true;
        else if(cam_id==1 && pos_x>=0.0 && pos_y<0.0) return true;
        else if(cam_id==0 && pos_x>=0.0 && pos_y>=0.0) return true;
        else return false;
    }
}

void addBall_Pkg(SSL_DetectionBall ball, SSL_DetectionFrame* detectionFrame){
    SSL_DetectionBall* detectedBall = detectionFrame->add_balls();
    detectedBall->set_confidence(ball.confidence());
    detectedBall->set_area(ball.area());
    detectedBall->set_x(ball.x());
    detectedBall->set_y(ball.y());
    detectedBall->set_z(ball.z());
    detectedBall->set_pixel_x(ball.pixel_x());
    detectedBall->set_pixel_y(ball.pixel_y());
}

void addRobotYellow_Pkg(SSL_DetectionRobot robot, SSL_DetectionFrame* detectionFrame){
    SSL_DetectionRobot* detectedRobot = detectionFrame->add_robots_yellow();
    detectedRobot->set_confidence(robot.confidence());
    detectedRobot->set_robot_id(robot.robot_id());
    detectedRobot->set_x(robot.x());
    detectedRobot->set_y(robot.y());
    detectedRobot->set_orientation(robot.orientation());
    detectedRobot->set_pixel_x(robot.pixel_x());
    detectedRobot->set_pixel_y(robot.pixel_y());
    detectedRobot->set_height(robot.height());
}

void addRobotBlue_Pkg(SSL_DetectionRobot robot, SSL_DetectionFrame* detectionFrame){
    SSL_DetectionRobot* detectedRobot = detectionFrame->add_robots_blue();
    detectedRobot->set_confidence(robot.confidence());
    detectedRobot->set_robot_id(robot.robot_id());
    detectedRobot->set_x(robot.x());
    detectedRobot->set_y(robot.y());
    detectedRobot->set_orientation(robot.orientation());
    detectedRobot->set_pixel_x(robot.pixel_x());
    detectedRobot->set_pixel_y(robot.pixel_y());
    detectedRobot->set_height(robot.height());
}
