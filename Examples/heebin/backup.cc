#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <thread>
#include<algorithm>
#include<System.h>
#include <arpa/inet.h>

#include <boost/lockfree/queue.hpp>
#include <boost/format.hpp>
#include <boost/bind.hpp>
#include <boost/serialization/singleton.hpp>



using std::thread;
using namespace cv;
using namespace std;

void socket_thread(int newsockfd);
void error(const char *msg);
void fromPi();
void toUnity();

struct rt_packet{
	float data[16];
	unsigned int addr;
};

namespace heebin_queue{
	boost::lockfree::queue<struct rt_packet*>* Q;
}


void socket_thread(int newsockfd){
	
	char time_buff[8];
	char len_buff[16];	
   	char buff[80000];
	int recieved_bytes;

	char addr_buff[4];
	unsigned int client_addr;
	recieved_bytes = 0;
         for(int i=0; i<4; i+=recieved_bytes){
               recieved_bytes = recv(newsockfd, addr_buff+i, 4-i, 0);
         }	

	client_addr = *((unsigned int*) addr_buff);
	

	ORB_SLAM2::System SLAM("Vocabulary/ORBvoc.txt", "Examples/heebin/heebin.yaml",
	 		ORB_SLAM2::System::MONOCULAR, true);


	char sample = 'a';
	send(newsockfd, &sample, sizeof(char), 0);
	

	while(true){
		bzero(len_buff, 16);
		bzero(buff, 40000);
		recieved_bytes = 0;
		for(int i=0; i<8; i+=recieved_bytes){
			recieved_bytes = recv(newsockfd, time_buff+i, 8-i, 0);
		}
		double tframe = *((double*)time_buff);

		recieved_bytes = 0;
		for(int i=0; i<16; i+= recieved_bytes){
			recieved_bytes = recv(newsockfd, len_buff+i, 16-i, 0);
		}

		int len = atoi(len_buff);
		recieved_bytes = 0;
	//	cout<<len<<endl;
		for(int i=0; i<len; i+= recieved_bytes){
			recieved_bytes = recv(newsockfd, buff+i, len-i, 0);
		}

		std::vector<char> data(buff, buff + len);


		Mat jpegimage = imdecode(data,CV_LOAD_IMAGE_COLOR);

		
		cv::Mat Tcw = SLAM.TrackMonocular(jpegimage,tframe);
		

		if(Tcw.total()!=0){
			struct rt_packet* packet_data = (struct rt_packet*)malloc(sizeof(struct rt_packet));
			
			cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
			cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);
			float refined[16];
			bzero(refined, 16*sizeof(float));
			for(int i=0; i<3; i++){
				for(int j=0; j<3; j++){
					refined[i*4+j]=Rwc.at<float>(i,j);
				}
			}
			refined[3] = ((float*) twc.data)[0];
			refined[7] = ((float*) twc.data)[1];
			refined[11] = ((float*) twc.data)[2];
			refined[15] = 1.0f;
		
			memcpy(&(packet_data->data), refined, sizeof(float) *16);
			memcpy(&(packet_data->addr), &client_addr, sizeof(unsigned int));
			heebin_queue::Q->push(packet_data);
		}

       		//cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        	//cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);
		//cout<<"-----heebin result-----\n"<<Tcw<<endl;
		

	//	cout<<tframe<<endl;
//		imshow( "Server", jpegimage );
//		waitKey(1);
	}

//     SLAM.Shutdown();
     close(newsockfd);

}


void error(const char *msg)
{
    perror(msg);
    exit(1);
}


void fromPi(){

     int sockfd, newsockfd, portno;
     socklen_t clilen;
     struct sockaddr_in serv_addr, cli_addr;
     //int n;

     sockfd = socket(AF_INET, SOCK_STREAM, 0);
     if (sockfd < 0) error("ERROR opening socket");
     bzero((char *) &serv_addr, sizeof(serv_addr));
     portno = 4966;
     serv_addr.sin_family = AF_INET;
     serv_addr.sin_addr.s_addr = INADDR_ANY;
     serv_addr.sin_port = htons(portno);
     if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) 
              error("ERROR on binding");

     listen(sockfd,5);


     clilen = sizeof(cli_addr);

	while(true){
	     	newsockfd = accept(sockfd, 
        		         (struct sockaddr *) &cli_addr, 
                		 &clilen);
    		 if (newsockfd < 0) error("ERROR on accept");
	

		thread t(socket_thread, newsockfd);
		t.detach();
	
	}

	

     close(sockfd);

}

void toUnity(){


       int s;
        struct sockaddr_in server_addr;

 
        if((s = socket(PF_INET, SOCK_STREAM, 0)) < 0)
	{
                error("can't create socket\n");
                return;
        }
 
        bzero((char *)&server_addr, sizeof(server_addr));
 
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = inet_addr("192.168.0.21");
        server_addr.sin_port = htons(6488);
 
        if(connect(s, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
        {
               error("can't connect.\n");
		return;
                
        }
 
	struct rt_packet* data;
	while(true){
		if(heebin_queue::Q->pop(data)){
			send(s, data, sizeof(struct rt_packet), 0);		
			free(data);
		}
	}


	close(s);
}


int main(int argc, char **argv){

	heebin_queue::Q = new boost::lockfree::queue<struct rt_packet*>(512);
	thread t2(toUnity);
	thread t(fromPi);
	t.join();
	t2.join();	

}





