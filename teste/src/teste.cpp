#include <math.h>
#include <time.h>
#include <string>
#include <fstream>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
 #include <opencv2/highgui/highgui.hpp>
 #include <cv_bridge/cv_bridge.h>
 #include <image_transport/image_transport.h>

#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Image.h"

#include "std_msgs/Float32.h"

using namespace std;
using namespace cv;

geometry_msgs::Pose poseDrone;
geometry_msgs::Pose target;
geometry_msgs::Twist vel;
std_msgs::Empty empty;

image_transport::Subscriber image_sub;

//Publishers
ros::Publisher pubGotoXY;
ros::Publisher pubLand; 
ros::Publisher pubTakeOff; 
ros::Publisher pubSetVelocity; 

//Subscribers
ros::Subscriber checkPose;
ros::Subscriber imageRaw;

//cv_bridge::CvImagePtr image;
static const std::string OPENCV_WINDOW = "Image window";

//cv_bridge::CvImagePtr* cv_ptr;

Mat image, gray, bw, image2, image3, image4, image5, image6, image7, thr;
vector<Vec2f> lines, lines2; // will hold the results of the detection hughlines
vector<Vec4i> linesP, linesP2; // will hold the results of the detection hughlinesP

//PID
float kpX = 0.00004;
float kdX = 0.0004;
float kiX = 0.0000009;
float kpY = 0.000006;
float kdY = 0.00002;
float kiY = 0.00000009;
float kpZ = 0.004;
float kdZ = 0.5;
float kiZ = 0.0009;
float kpR = 0.04;
float kdR = 0.5;
float kiR = 0.004;

float erropX = 0;
float errodX = 0;
float erroiX = 0;
float erropY = 0;
float errodY = 0;
float erroiY = 0;
float erropZ = 0;
float errodZ = 0;
float erroiZ = 0;
float erropR = 0;
float errodR = 0;
float erroiR = 0;

int go = 0, state = 0, testGoto = 0;
std_msgs::Float32  rotation;
//rotation.data = 0;


std_msgs::Float32 toEuler(geometry_msgs::Quaternion q);
geometry_msgs::Quaternion toQuaternion(double roll, double pitch, double yaw);

int espera = 0;
vector<float> angles; 


float posXIni = 0;
float posYini = -96;


struct Xpto
{
    geometry_msgs::Pose pose;
	vector<float> angles;
};

vector<Xpto> vecXpto; 

void gotoXY(geometry_msgs::Pose msg)
{
	pubGotoXY.publish(msg);
	//go = 1;
	erropX = 0;
	errodX = 0;
	erroiX = 0;
	erropY = 0;
	errodY = 0;
	erroiY = 0;
	erropZ = 0;
	errodZ = 0;
	erroiZ = 0;
	erropR = 0;
	errodR = 0;
	erroiR = 0;
}

void land(void)
{
	pubLand.publish(empty);
}

void takeOff(void)
{
	pubTakeOff.publish(empty);
}

void setVelocity(geometry_msgs::Twist msg)
{
	pubSetVelocity.publish(msg);
}

void CheckPoseCallback(const geometry_msgs::Pose msg)
{
	poseDrone = msg;
}

void calculaVel()
{
	if(image.empty())
	{
		return;
	}
	
	//cv::imshow("imageColor", image);
	
	image2 = image.clone();
	
   for(int i=0; i<image2.rows; i++){
		for(int j=0; j<image2.cols; j++){
			Vec3b valor = image2.at<Vec3b>(i, j);
			
			// Essa merda usa padrao BGR (blue green red)
			if(valor[0]>=0 && valor[0]<60 && valor[1]>50 && valor[1]<=256 && valor[2]>100 && valor[2]<=256){
				valor[0]=255;
				valor[1]=255;
				valor[2]=255;
			}
			else{
				valor[0]=0;
				valor[1]=0;
				valor[2]=0;
			}
			
			image2.at<Vec3b>(i, j) = valor;
		}
	}
	Canny(image2, image3, 50, 200, 3);
	
	//cv::imshow("imageBW", image2);
	
	//cv::imshow("Canny", image3);
	
	image4 = image2.clone();
	image6 = image2.clone();
	bitwise_not(image3, image5);
	
	
	vector<vector<Point>> contour;
	vector<Vec4i> hierarchy;
	
	HoughLines(image3, lines, 1, CV_PI/180, 120, 0, 0); // runs the actual detection
	findContours(image3, contour, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
	
	

	
    for( size_t i = 0; i < lines.size(); i++ )
    {
		int contador = 0;
		
		if(lines[i][1] > CV_PI/2)
		{
			lines[i][1] = lines[i][1] - CV_PI;
			lines[i][0] = -lines[i][0];
		}
		
		float rho = lines[i][0], theta = lines[i][1];
		
		if(lines2.size() == 0)
		{
			lines2.push_back(lines[i]);
		}
		else
		{
			for( size_t j = 0; j < lines2.size(); j++ )
			{
					if(theta > lines2[j][1]-0.1 && theta < lines2[j][1]+0.1)
					{
						contador++;
					}
			}
			if(contador == 0)
			{
					lines2.push_back(lines[i]);
			}
		}
		
        Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        line( image2, pt1, pt2, Scalar(0,0,0), 10, LINE_AA);
		
		//cout << i << "rho " << rho << "  theta " << theta << "-------:" << "X0 "<< x0 << "Y0" << y0 << endl;
    }
	
	if(lines2.size() > 1 && state == 0)
	{
		state = 1;
		rotation = toEuler(poseDrone.orientation);
	}
	
	Canny(image2, image5, 50, 200, 3);
    HoughLinesP(image5, linesP, 1, CV_PI/180, 50, 120, 9); // runs the actual detection
	
	//cv::imshow("imagem2", image5);
	
    for( size_t i = 0; i < linesP.size(); i++ )
    {
		int contador = 0;
		Vec4i l = linesP[i];
		float theta = atan2(l[1]-l[3],l[0]-l[2]);
		
		if(linesP2.size() == 0)
		{
			linesP2.push_back(linesP[i]);
		}
		else
		{
			for( size_t j = 0; j < linesP2.size(); j++ )
			{
					Vec4i l2 = linesP2[j];
					if(theta < atan2(l2[1]-l2[3],l2[0]-l2[2])+0.1 && theta > atan2(l2[1]-l2[3],l2[0]-l2[2])-0.1)
					{
						//if((l[0] + 256) > l2[2] && l2[2] > l[0])
						if((l[0] > l2[0] && l[0] < l2[2]) || (l[2] > l2[0] && l[2] < l2[2]))
						{
							contador ++;
						}
						else if(l[0] < l2[0] && l[2] > l2[2])
						{
							contador++;
							linesP2[j] = linesP[i];
						}
						//else if((l[1] + 256) > l2[3] && l2[3] > l[1])
						else if((l[1] > l2[1] && l[1] < l2[3]) || (l[3] > l2[1] && l[3] < l2[3]))
						{
							contador++;
						}
						else if(l[1] < l2[1] && l[3] > l2[3])
						{
							contador++;
							linesP2[j] = linesP[i];
						}
					}
			}
			if(contador == 0)
			{
					linesP2.push_back(linesP[i]);
			}
		}
        
		//cout << "theta " << theta << " x0: " << l[0] << " X1: " << l[2] << " Y0: " << l[1] << " Y1: " << l[3] << endl;
		//cout << "x0: " << l[0] << " Y0: " << l[1] <<  "x1: " << l[2] << " Y1: " << l[3] <<  endl;
    }
	
	
	//Pode ser usado para mostrar na image4 a linha para seguir
	for( size_t i = 0; i < lines2.size(); i++ )
	{
		float rho = lines2[i][0], theta = lines2[i][1];
		Point pt1, pt2, pt3, pt4;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
		
		//pt3.x = cvRound(x0 + 256*(-b));
        //pt3.y = cvRound(y0 + 256*(a));
		
		//pt4.x = cvRound(x0 - (256-x0)*(-b));
        //pt4.y = cvRound(y0 + (y0 - 256)*(a));
		if(theta > -0.1 && theta < 0.1)
		{
			pt3.y = cvRound(256);
			pt4.x = cvRound(0);
		}
		else
		{
			pt3.y = cvRound(0);
			pt4.x = cvRound(256);
		}
		if(theta < 0)
		{
			pt4.y = cvRound(y0 + 50 - (x0 - 256)*(a)); //VERMELHO
			pt3.x = cvRound(x0 + (y0 - 256)*(b)); // VERDE
		}
		else
		{
			pt4.y = cvRound(y0 + (x0 - 256)*(a)); //VERMELHO
			pt3.x = cvRound(x0 + (y0 - 256)*(-b)); // VERDE
		}
		//256 - x0 . (- b) 
		
        //pt2.x = cvRound(x0 + 256*(-b));
        //pt2.y = cvRound(y0 - 1000*(a));
		
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
		
		// Essa merda usa padrao BGR (blue green red)
		
        line( image2, pt1, pt2, Scalar(0,0,0), 10, LINE_AA);
		//circle(image4, Point(256,256), 2, Scalar(0,255,255), 3); // MEIO verde
		//circle(image4, Point(pt3.x,pt3.y), 2, Scalar(0,255,0), 3); // X 'e verde
		//circle(image4, Point(x0,y0), 2, Scalar(0,0,255), 3); // Y 'e azul
		//circle(image4, Point(pt4.x,pt4.y), 2, Scalar(255,0,0), 3);
		line( image4, pt1, pt2, Scalar(0,0,255), 1, LINE_AA);
		
		//cout << i << "rho " << rho << "  theta " << theta << "-------:" << " X0 "<< x0 << " Y0 " << y0 << " X3 "<< pt3.x << " Y4 " << pt4.y << endl;
		//cout << i << "rho " << rho << "  theta " << theta << "-------:" << " X0 "<< x0 << " Y0 " << y0 << " X3 "<< pt3.x << " Y3 " << pt3.y << "   "  << 256*(-b) << endl;
	}
	//cv::imshow("image4", image4);
	//cv::imshow("imagem", image2);
	
	
	//Pode ser usado para mostrar na image6 as linhas para seguir
	for( size_t i = 0; i < linesP2.size(); i++ )
	{
		Vec4i l = linesP2[i];
		line( image6, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 1, LINE_AA);
		circle(image6, Point(l[0], l[1]), 2, Scalar(255,0,0), 3);
		float theta = atan2(l[1]-l[3],l[0]-l[2]);
		//cout << "theta " << theta << " x0: " << l[0] << " X1: " << l[2] << " Y0: " << l[1] << " Y1: " << l[3] << endl;
	}
	//cv::imshow("image6", image6);
	
	for( size_t i = 0; i < linesP.size(); i++ )
	{
		Vec4i l = linesP[i];
		line( image6, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,255,0), 1, LINE_AA);
		circle(image6, Point(l[0], l[1]), 2, Scalar(0,0,255), 3);
		circle(image6, Point(l[2], l[3]), 2, Scalar(255,0,0), 3);
		//float theta = atan2(l[1]-l[3],l[0]-l[2]);
		//cout << "theta " << theta << " x0: " << l[0] << " X1: " << l[2] << " Y0: " << l[1] << " Y1: " << l[3] << endl;
	}
	//cv::imshow("teste", image6);
	
	
	if(lines2.size() == 1 && (go == 5 || go == 0))
	{
		float rho = lines2[0][0], theta = lines2[0][1];
		if(theta > - 0.1 && theta < 0.1)
		{
			//cout << " Size 1" << endl;
			
			
			double a = cos(theta), b = sin(theta);
			double x0 = a*rho, y0 = b*rho;
			
			if(theta < 0)
			{
				//pt4.y = cvRound(y0 - (x0 - 256)*(a)); //VERMELHO
				erropX = x0 + (y0 - 256)*(b) - 256; // VERDE
			}
			else
			{
				//pt4.y = cvRound(y0 + (x0 - 256)*(a)); //VERMELHO
				erropX = x0 + (y0 - 256)*(-b) - 256; // VERDE
			}
			
			//erropX = cvRound(x0 - (256-x0)*(-b))- 256;
			
			float z = poseDrone.position.z;
			erropZ = 7 - z;
			
			if(lines[0][1] > CV_PI/2)
			{
				lines[0][1] = lines[0][1] - CV_PI;
			}
			
			erropR = -lines2[0][1]; 

			vel.linear.x = kpX*(erropX) + kdX*(erropX - errodX) + kiX*(erroiX);
			//vel.linear.x = -vel.linear.x;
			if(vel.linear.x > 0.1)
			{
				vel.linear.x = 0.1;
			}
			else if(vel.linear.x < -0.1)
			{
				vel.linear.x = -0.1;
			}
			vel.linear.y = 0.001;

			//vel.linear.z = 0;
			vel.linear.z = kpZ*(erropZ) + kdZ*(erropZ - errodZ) + kiZ*(erroiZ);
			vel.angular.x = 0;
			vel.angular.y = 0;
			vel.angular.z = kpR*(erropR) + kdR*(erropR - errodR) + kiR*(erroiR);

			//cout << "ROTACAO ERRO: " << erropR <<" P: " << kpR*(erropR)  << " D: " <<  kdR*(erropR - errodR) << " I: " << kiR*(erroiR) << " V R: " << vel.angular.z << endl;
			//cout << "X ERRO: " << erropX << " P: " << kpX*(erropX)  << " D: " <<  kdX*(erropX - errodX) << " I: " << kiX*(erroiX) << " V X: " << vel.linear.x << endl;

			errodX = erropX;
			errodZ = erropZ;
			errodR = erropR;

			erroiX += erropX;
			erroiZ += erropZ;
			erroiR += erroiR;
			
			setVelocity(vel);
			go = 0;
		}
		else
		{
			//cout << "Size 1 ERRADO" << endl;
		}
	}
	else if(lines2.size() > 1 || go == 3 || go == 4 || go == 5)
	{
		//cout << "Lines > 1" << endl;
		float z = poseDrone.position.z;
		float x = poseDrone.position.x;
		float y = poseDrone.position.y;
		float w = toEuler(poseDrone.orientation).data;
		
		if(sqrt(pow(poseDrone.position.x - target.position.x,2) + pow(poseDrone.position.y - target.position.y,2)) < 8 && go == 0)
		{
			//cout << "??" << endl;
			go = 5;
		}
		
		float yyy = 0, xxx = 0;
		
		for( size_t i = 0; i < lines2.size(); i++ )
		{
			
			float rho = lines2[i][0], theta = lines2[i][1];
			//cout << "ThetaX: " << theta << " rho: " << rho <<endl;
			if(theta < 0.1 && theta > -0.1)
			{
				//float rho = lines2[i][0], theta = lines2[i][1];
				double a = cos(theta), b = sin(theta);
				double x0 = a*rho, y0 = b*rho;
				
				if(theta < 0)
				{
					//pt4.y = cvRound(y0 - (x0 - 256)*(a)); //VERMELHO
					xxx = x0 + (y0 - 256)*(b) -256; // VERDE
				}
				else
				{
					//pt4.y = cvRound(y0 + (x0 - 256)*(a)); //VERMELHO
					xxx = x0 + (y0 - 256)*(-b) - 256; // VERDE
				}
				
				//xxx = cvRound(x0 - (256-x0)*(-b)) - 256;
				//cout << "Theta: " << theta << " rho: " << rho << " XXX " << xxx <<endl;
				//cout << "XXX" << xxx << endl;
				//break;
			}
		}
		
		//cout << "-----------------------------------" << endl;
		for( size_t i = 0; i < lines2.size(); i++ )
		{
			float rho = lines2[i][0], theta = lines2[i][1];
			if(theta > 0.1 && theta < CV_PI -0.1 || theta < -0.1 && theta > -CV_PI +0.1)
			{
				//float rho = lines2[i][0], theta = lines2[i][1];
				double a = cos(theta), b = sin(theta);
				double x0 = a*rho, y0 = b*rho;
				
				if(theta < 0)
				{
					yyy = y0 - (x0 - 256 - xxx)*(a) - 206; //VERMELHO
					//pt3.x = cvRound(x0 + (y0 - 256)*(b)); // VERDE
				}
				else
				{
					yyy = y0 + (x0 - 256 - xxx)*(a) - 256; //VERMELHO
					//pt3.x = cvRound(x0 + (y0 - 256)*(-b)); // VERDE
				}
				//yyy = cvRound(y0 - (256+y0)*(a)) - 256;
				
				//yyy = tan(theta+w)*(256-xxx) + 256;
				if(theta < 0)
				{
					//yyy = tan(-(theta+w))*(256-xxx) + 256;
				}
				//cout << "i" << i  << "   ThetaY: " << theta << " rho: " << rho << " yyy: " << yyy << " X0: " << x0 << " Y0: " << y0 << " (x0 - 256)*(a): " << (x0 - 256)*(a) << " POS MAPA: " << yyy + 256 <<endl;
				//cout << "yyy" << yyy << endl;
				//break;
				//testGoto = 0;
			}
		}

		
		float testY = z*-yyy*tan(25*CV_PI/180)/256;
		float testX = z*-xxx*tan(25*CV_PI/180)/256;
		
		//cout << "TesteX: " << testX  << " TesteY: " << testY  << endl;
		//cout << "TesteX: " << testX*cos(-w) + testY*sin(-w)<< " TesteY: " << testY*cos(-w) + testX*sin(w) << " w: " << toEuler(poseDrone.orientation).data << endl;
		
		if(go == 0)
		{
			float w = toEuler(poseDrone.orientation).data;
			cout << "GO 0" << endl;
			target.position.x = x + testX*cos(-w) + testY*sin(-w);
			target.position.y = y + testY*cos(-w) + testX*sin(w);
			target.position.z = 7.0;
			target.orientation = poseDrone.orientation;
			
			//cout << "TesteX: " << testX  << " TesteY: " << testY  << endl;
			//cout << "TesteX: " << testX*cos(-w) + testY*sin(-w)<< " TesteY: " << testY*cos(-w) + testX*sin(-w) << " w: " << toEuler(poseDrone.orientation).data << endl;
			
			go = 2;
			//go = 3;

			gotoXY(target);
		}
		else if(go == 2)
		{
			//cout << "Erro 2 X: " << poseDrone.position.x - target.position.x << " Y: "  << poseDrone.position.y - target.position.y << endl;
			if(sqrt(pow(poseDrone.position.x - target.position.x,2) + pow(poseDrone.position.y - target.position.y,2)) < 0.06)
			{
				if(espera > 40)
				{
					espera = 0;
					cout << " CHEGOU 2" << endl;
					target.position.x = x + testX*cos(-w) + testY*sin(-w);
					target.position.y = y + testY*cos(-w) + testX*sin(w);
					target.position.z = 7.0;
					target.orientation = poseDrone.orientation;
					
					//cout << "TesteX: " << testX  << " TesteY: " << testY  << endl;
					//cout << "TesteX: " << testX*cos(-w) + testY*sin(-w)<< " TesteY: " << testY*cos(-w) + testX*sin(-w) << " w: " << toEuler(poseDrone.orientation).data << endl;
					go = 3;
					gotoXY(target);
				}
				else 
				{
					espera++;
				}
			}
			
		}
		else if(go == 3)
		{
			//cout << "Erro 3 X: " << poseDrone.position.x - target.position.x << " Y: "  << poseDrone.position.y - target.position.y << endl;
			//espera++;
			if(sqrt(pow(poseDrone.position.x - target.position.x,2) + pow(poseDrone.position.y - target.position.y,2)) < 0.06)
			{
				
				if(espera > 50)
				{
					if(espera > 100)
					{
						cout << "GO 3" << endl;
						
						int hold = -1;
						for(int i = 0; i < vecXpto.size(); i++)
						{
							if(vecXpto[i].pose.position.x < poseDrone.position.x + 4 && vecXpto[i].pose.position.x > poseDrone.position.x - 4 && vecXpto[i].pose.position.y < poseDrone.position.y + 4 && vecXpto[i].pose.position.y > poseDrone.position.y - 4)
							{
								hold = i;
							}
						}
						
						if(hold == -1)
						{
							cout << "	Intersec Nova!!" << endl;
							Xpto xpto;
							xpto.pose = poseDrone;
							xpto.angles = angles;
							
							for( size_t j = 0; j < angles.size(); j++ )
							{
								cout << "		Angles" << j << "  " << angles[j] << endl; 
							}
							
							vecXpto.push_back(xpto);
							angles.clear();
							float w = toEuler(poseDrone.orientation).data;
							
							float theta = vecXpto.back().angles.back();
							vecXpto.back().angles.pop_back();
							target.orientation = toQuaternion(0,0,theta+w);
							
							//cout << "Drone: " << w << " Theta: " << theta << " Target: " << endl;
							
							gotoXY(target);
							
							
							
							if(vecXpto.back().angles.size() == 0)
							{
								vecXpto.pop_back();
							}
						}
						else
						{
							float w = toEuler(poseDrone.orientation).data;
							float ww = toEuler(vecXpto[hold].pose.orientation).data;
							int h = -1;
							if(testGoto == 0)
							{
								cout << "VEM PELO LINES 1" << endl; 
								for( size_t j = 0; j < vecXpto[hold].angles.size(); j++ )
								{
									float www;
									if(w < 0)
									{
										www = CV_PI + w;
									}
									else
									{
										www = w - CV_PI;
									}
									if((vecXpto[hold].angles[j] + ww - www) < 0.1 && (vecXpto[hold].angles[j] + ww - www) > -0.1)
									{
										h = j;
									}
									cout << "WWW: " << www << " DEL: " << h << endl;
								}
								if(h != -1)
								{
									vecXpto[hold].angles.erase(vecXpto[hold].angles.begin()+h);
								}
								if(vecXpto[hold].angles.size() == 0)
								{
									cout << "IF Remove Slot " << endl;
									vecXpto.erase(vecXpto.begin()+hold);
									float theta = vecXpto.back().angles.back();
									vecXpto.back().angles.pop_back();									// aqui precisa da orientacao do pose do ponto
									float ww = toEuler(vecXpto.back().pose.orientation).data;
									target.orientation = toQuaternion(0,0,theta+ww);
									target.position = vecXpto.back().pose.position;
									if(vecXpto.back().angles.size() == 0)
									{
										vecXpto.pop_back();
									}
									gotoXY(target);
								}
								else
								{
									
									float theta = vecXpto[hold].angles.back();
									vecXpto[hold].angles.pop_back();
									target.position = vecXpto[hold].pose.position;
									float ww = toEuler(vecXpto[hold].pose.orientation).data;
									target.orientation = toQuaternion(0,0,theta+ww);
									cout << "ELSE: " << theta+ww << " Theta (Angles): " << theta << " WW(Pose): " << ww<< endl;
									if(vecXpto[hold].angles.size() == 0)
									{
										cout << "Remove Slot " << endl;
										vecXpto.erase(vecXpto.begin()+hold);
									}
									gotoXY(target);
										//precisa aqui
								}
								angles.clear();
							}
							else
							{
								cout << "VEM PELO GOTO" << endl;
								
								float theta = vecXpto[hold].angles.back();
								vecXpto[hold].angles.pop_back();
								target.orientation = toQuaternion(0,0,theta+ww);
								
								cout << "Drone: " << w << " Theta: " << theta << " Target: " << endl;
								
								gotoXY(target);
								angles.clear();
								if(vecXpto[hold].angles.size() == 0)
								{
									vecXpto.erase(vecXpto.begin()+hold);
								}
							}
						}
						
						
						espera = 0;
						go = 4;
						
						for( size_t a = 0; a < vecXpto.size(); a++ )
						{
							cout << a << " X: " << vecXpto[a].pose.position.x << " Y: " << vecXpto[a].pose.position.y << endl;
							for( size_t b = 0; b < vecXpto[a].angles.size(); b++ )
							{
								cout << "	-" << b << " Angles: " << vecXpto[a].angles[b] << endl;
							}
						}
						
						//target.orientation = toQuaternion(0,0,theta);
						
					}
					else
					{
						for( size_t i = 0; i < linesP2.size(); i++ )
						{
							Vec4i l = linesP2[i];
							float theta = atan2(l[1]-l[3],l[0]-l[2]);
							theta = theta - CV_PI/2;
							if(theta < -CV_PI)
							{
								theta =CV_PI + (CV_PI + theta);
							}
							if(l[0] < 200)
							{
								theta = theta-CV_PI;
							}
							
							if((theta > -0.1 && theta < 0.1) || (theta < -CV_PI +0.1) || (theta > CV_PI-0.1))
							{
								if(theta > 0)
								{
									if(l[1] > 300)
									{
										 continue;
									}
								}
								else
								{
									if(l[3] > 200)
									{
										continue; //AQUI PRECISA
									}
								}
								
							}
							if((theta < -CV_PI +0.1) || (theta > CV_PI-0.1))
							{
								continue;
							}
							
							
							//cout << "Thetas: " << theta << endl;
							int k = 0;
							for( size_t j = 0; j < angles.size(); j++ )
							{
								if((theta >= angles[j]-0.3 && theta <= angles[j] +0.3))
								{
									//cout << "???" << angles.size() << endl;
									k=1;
									break;
								}
							}
							if(k == 0)
							{
								//cout << "???" << endl;
								angles.push_back(theta);
							}
						}
						
						espera++;
					}
				}
				else
				{
					espera++;
				}
			}
		}
		else if(go == 4)
		{
			if(vecXpto.size() == 0)
			{
				//land();
				go = 99;
			}
			//cout << "Erro 4 R:" << toEuler(poseDrone.orientation).data - toEuler(target.orientation).data << endl;
			if((toEuler(poseDrone.orientation).data - toEuler(target.orientation).data < CV_PI+0.05 && toEuler(poseDrone.orientation).data - toEuler(target.orientation).data > CV_PI-0.05) || (toEuler(poseDrone.orientation).data - toEuler(target.orientation).data < -CV_PI+0.05 && toEuler(poseDrone.orientation).data - toEuler(target.orientation).data > -CV_PI-0.05) || (toEuler(poseDrone.orientation).data - toEuler(target.orientation).data < +0.05 && toEuler(poseDrone.orientation).data - toEuler(target.orientation).data > -0.05))
			{
				
				if(espera == 50)
				{
					espera = 0;
					cout << "GO 4" << endl;
					vel.linear.x = 0;
					vel.linear.y = 0.003;
					vel.linear.z = 0;
					vel.angular.x = 0;
					vel.angular.y = 0;
					vel.angular.z = 0;
					
					erropX = 0;
					errodX = 0;
					erroiX = 0;
					erropR = 0;
					errodR = 0;
					erroiR = 0;
					
					//setVelocity(vel);
					go = 5;
				}
				else 
				{
					espera ++;
				}
			}
			
		}
		else if(go == 5)
		{
			//cout << "GO 5" << endl;
			int hold = -1;
			for( size_t i = 0; i < lines2.size(); i++ )
			{
				
				float theta = lines2[i][1];
				//cout << "i: " << i << "theta: " << theta << endl;
				if(theta > - 0.1 && theta < 0.1)
				{
					//seria melhor pegar o com theta menor
					hold = i;
					break;
				}
			}
			if(hold != -1)
			{
				float rho = lines2[hold][0], theta = lines2[hold][1];
				double a = cos(theta), b = sin(theta);
				double x0 = a*rho, y0 = b*rho;
				
				float z = poseDrone.position.z;
				erropZ = 7 - z;
				
				if(theta < 0)
				{
					//pt4.y = cvRound(y0 - (x0 - 256)*(a)); //VERMELHO
					erropX = x0 + (y0 - 256)*(b) - 256; // VERDE
				}
				else
				{
					//pt4.y = cvRound(y0 + (x0 - 256)*(a)); //VERMELHO
					erropX = x0 + (y0 - 256)*(-b) - 256; // VERDE
				}
		
				
				//cout << "hold: " << hold << "theta: " << lines2[hold][1] << " X > " << cos(lines2[hold][1])*lines2[hold][0] << endl;
				//erropX = cos(lines2[hold][1])*lines2[hold][0] - 256;
				erropR = -lines2[hold][1]; 
				vel.linear.x = kpX*(erropX) + kdX*(erropX - errodX) + kiX*(erroiX);
				//vel.linear.x = -vel.linear.x;
				if(vel.linear.x > 0.1)
				{
					vel.linear.x = 0.1;
				}
				else if(vel.linear.x < -0.1)
				{
					vel.linear.x = -0.1;
				}
				vel.angular.z = kpR*(erropR) + kdR*(erropR - errodR) + kiR*(erroiR);
			}
			else
			{
				vel.linear.x = 0;
				vel.angular.z = 0;
			}
			
			if(erropR > CV_PI)
				erropR - 2*CV_PI;
			else if(erropR < -CV_PI)
				erropR + 2*CV_PI;

			
			vel.linear.y = 0.001;

			//vel.linear.z = 0;
			vel.linear.z = kpZ*(erropZ) + kdZ*(erropZ - errodZ) + kiZ*(erroiZ);
			vel.angular.x = 0;
			vel.angular.y = 0;
			
			
			errodX = erropX;
			errodR = erropR;
			
			if ((errodR*errodR) > 0.3 )
			{
				//vel.angular.z = 0.05;
			} 

			erroiX += erropX;
			erroiR += erroiR;
			setVelocity(vel);
			testGoto = 0;
		}
		else if(go == 99)
		{
			if(sqrt(pow(poseDrone.position.x - target.position.x,2) + pow(poseDrone.position.y - target.position.y,2)) < 0.06)
			{
				land();
				cout << "ACABOU" << endl;
				return;
			}
		}
			
		
	}
	if(contour.size() == 0 && go == 0)
	{

		Xpto xpto = vecXpto.back();
		target.position.x = xpto.pose.position.x;
		target.position.y = xpto.pose.position.y;
		target.position.z = xpto.pose.position.z;
		target.orientation.x = 0.0;
		target.orientation.y = 0.0;
		target.orientation.z = 0.0;
		target.orientation.w = 1.0;
		cout << "FIM DE LINHA" << endl;
		go = 3;
		testGoto = 1;

		gotoXY(target);
	}
	if(go == 99)
	{
		if(sqrt(pow(poseDrone.position.x - target.position.x,2) + pow(poseDrone.position.y - target.position.y,2)) < 0.06)
		{
			land();
			cout << "ACABOU" << endl;
			return;
		}
	}
	
	lines2.clear();
	linesP2.clear();
	
}

void ImageRawCallback(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
    image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	if(go == 1)
	{
		if(sqrt(pow(poseDrone.position.x - target.position.x,2) + pow(poseDrone.position.y - target.position.y,2)) < 0.5)
		{		
			go = 0;
		}
	}
	else
	{
		//cout << "Calcula Vel: " << endl; 
		calculaVel();
	}
}

std_msgs::Float32 toEuler(geometry_msgs::Quaternion q)
{
    std_msgs::Float32 rotZRads;
    // yaw (z-axis rotation, in rad).
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);

    rotZRads.data = std::atan2(siny_cosp, cosy_cosp);
    
    //cout << rotZRads << endl;
    //cout << rotZRads.data*180/M_PI << endl;
    
    return rotZRads;
}

geometry_msgs::Quaternion toQuaternion(double roll, double pitch, double yaw) // roll (x), pitch (Y), yaw (z)
{
    // Abbreviations for the various angular functions

    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);

    geometry_msgs::Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

int main(int argc, char **argv)
{ 
    ros::init(argc, argv, "manager");
    ros::NodeHandle nh(""), nh_param("~");
    ros::Rate loop_rate(10);
	image_transport::ImageTransport it(nh);

	pubGotoXY = nh.advertise<geometry_msgs::Pose>("/drone/gotoXY", 1);
	pubLand = nh.advertise<std_msgs::Empty>("/drone/land", 1);
	pubTakeOff = nh.advertise<std_msgs::Empty>("/drone/takeOff", 1);
	pubSetVelocity = nh.advertise<geometry_msgs::Twist>("/drone/setVelocity", 1);
	
    checkPose = nh.subscribe<geometry_msgs::Pose>("/drone/checkPose", 1, &CheckPoseCallback);
	image_sub= it.subscribe("/visionSensorData/image_raw", 1, &ImageRawCallback);
	
	//cv::namedWindow(OPENCV_WINDOW);
	cv::startWindowThread();
	Xpto xpto;
	xpto.pose.position.x = posXIni;
	xpto.pose.position.y = posYini;
	xpto.pose.position.z = 7;
	xpto.pose.orientation = toQuaternion(0,0,0);
	xpto.angles.push_back(0);

	vecXpto.push_back(xpto);
	int estado = 0;
    while(ros::ok())
    {
        ros::spinOnce();
		
		
		
        loop_rate.sleep();
    }    
    return 0;
}
