/***********************************************************************
 universidade federal de itajuba - UNIFEI
 mestrado em ciencia e tecnologia da computacao
 wander mendes martins - wandermendes@unifei.edu.br
 rafael gomes braga    - rafaelbraga@unifei.edu.br
 alaxandre carlos brandão ramos - ramos@@unifei.edu.br
************************************************************************
Este programa executa as seguintes tarefas:
    1) recebe uma imagem da camera embarcada em um VANT
    2) a trasnforma em escalas de cinza
    3) aplica binarização baseada em limiar médio
    4) divide a imagem em 3 quadrantes horizontais de mesmo tamanho
    5) identifica o quadrante com maior número de pixels brancos
    6) envia comandos ao DRONE para se dirigir a este quadrante
***********************************************************************/
// g++ -o ex16 ex16.cpp $(pkg-config opencv --cflags --libs)

#include <stdio.h>
#include <stdlib.h>	
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Bool.h>

//----------------------------------------------------------------------
using namespace cv;
using namespace std;

// ROS Objects
image_transport::Subscriber image_sub;  // Subscriber to the front camera
ros::Subscriber enable_control_sub; 	// Subscriber to enable control
ros::Publisher cmd_vel_pub; 			// Publishes geometry_msgs/Twist messages
ros::Publisher cmd_vel_stamped_pub;     // Publishes geometry_msgs/TwistStamped messages

//28/03/17
double velY, velX;
int thresh = 100;
int max_thresh = 255;
RNG rng(12345);
int Paint = 155;

// Enable control flag
bool enableControl;

//28/03/17
int option = 1;
int edgeThresh = 1;
int edgeThreshScharr=1;
int edgeOriginal=1;
int vmostrarimagens = 1; //0=don't show images; 1=show images

Mat image, gray, blurImage, edge1, edge2, cedge;

//headers
void KOALLA(Mat Image);
int FreeSide(Mat Image);
void ShowImage(Mat Source, string WindowName);
void imageCallback(const sensor_msgs::ImageConstPtr& msg);
void enableControlCb(const std_msgs::BoolConstPtr &msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "koalla");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_sub = it.subscribe("/erlecopter/front/image_front_raw", 1, imageCallback);
    enable_control_sub = nh.subscribe("koalla/enable_control", 1, enableControlCb);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("koalla/cmd_vel", 10);
    cmd_vel_stamped_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
    
    ros::spin();
    
    /*ros::Rate rate(0.5);
    while (nh.ok())
    {
        ros::spinOnce();
        rate.sleep();
    }*/
    return 0;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{    
    // Variables
    Mat image, ImageGrayScale;        

    try
    {
		ROS_INFO_STREAM("\n\nEnable control: " << enableControl << ";");
		
		// Get the msg image
        image = cv_bridge::toCvShare(msg, "bgr8")->image;
	    ShowImage(image,"Camera View");
      

   	    /// Convert image to gray and blur it
	    cvtColor( image, ImageGrayScale, CV_BGR2GRAY );
	    ShowImage(ImageGrayScale, "Gray Scale");
	    /// Blur the image
	    blur( ImageGrayScale, ImageGrayScale, Size(3,3) );
	    ShowImage(ImageGrayScale, "Blur");
	      
	    Mat canny_output;
	    vector<vector<Point> > contours;
	    vector<Vec4i> hierarchy;

	    /// Detect edges using canny
	    Canny( ImageGrayScale, canny_output, thresh, thresh*2, 3 );

	    /// Find contours 
	    ///The function retrieves contours from the binary image using the algorithm @cite Suzuki85 . The contours
		///are a useful tool for shape analysis and object detection and recognition. 
		///See squares.cpp in the OpenCV sample directory.
		//(imgproc.hpp)
	    findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

		/// Draw contours
		Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
		for( int i = 0; i< contours.size(); i++ )
		 {
		   Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		   ///The function draws contour outlines in the image if \f$\texttt{thickness} \ge 0\f$ or fills the area
		   ///bounded by the contours if \f$\texttt{thickness}<0\f$ . The example below shows how to retrieve
		   ///connected components from the binary image and label them: :
		   //(imgproc.hpp)
		   drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
		 }

		/// Show in a window
		ShowImage(drawing,"Contours");
	  
        /// Apply the Koalla Algorithm
	    KOALLA(drawing);

        ///=============================================================
        /// Move VANT			    
        ///=============================================================
        if ( enableControl == true )
        {
			// Create Twist msg
			geometry_msgs::Twist twist;
			twist.linear.x = velX;
			twist.linear.y = velY;
			
			// Create a TwistStamped msg
			geometry_msgs::TwistStamped twistStamped;
			twistStamped.header.frame_id = "base_link";
			twistStamped.twist.linear.x = velY;
			twistStamped.twist.linear.y = -velX;
			
			// Publish the two messages
			cmd_vel_pub.publish(twist);
			cmd_vel_stamped_pub.publish(twistStamped);
		}
        
        
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }         
}

//----------------------------------------------------------------------
void ShowImage(Mat Source, string WindowName){
	// Show a Image
	if (vmostrarimagens==0)
		return;
	 
	namedWindow(WindowName, WINDOW_AUTOSIZE );
	imshow(WindowName, Source);
	waitKey(50);
}

//----------------------------------------------------------------------
void KOALLA(Mat Image){
	 /// The Koalla Algorithm
		Mat ImageWithEdge = Image;
		Mat FinalImage    = Image;
        		
		// SCAN THE IMAGE - FROM A SIDE TO ANOTHER
		// ALL SPACE IS A FREE SPACE UNTIL FIND THE EDGE
		// AFTER THIS, ALL SPACE IS A OBSTACLE 
        int Color = 0;
        
		int x,y;		
        // TOP DOWN
	    for( x = 0; x < ImageWithEdge.cols; x++ )
        {
			 y = 0;
			 while (y < ImageWithEdge.rows && FinalImage.at<Vec3b>(y,x)[0] == Color) 
			 {
				 if (   (FinalImage.at<Vec3b>(y,x)[0] == Color) &&
						(FinalImage.at<Vec3b>(y,x)[1] == Color) &&
						(FinalImage.at<Vec3b>(y,x)[2] == Color)    )
					{
						 FinalImage.at<Vec3b>(y,x)[0] = Paint;	 
						 FinalImage.at<Vec3b>(y,x)[1] = Paint;	 
						 FinalImage.at<Vec3b>(y,x)[2] = Paint;
					 }
	 			 y++;
			}	 
         }	
         	  
        // DOWN UP
	    for( x = 0; x < ImageWithEdge.cols; x++ )
        {
			 y = ImageWithEdge.rows-1;
			 while (y >=  0 && FinalImage.at<Vec3b>(y,x)[0] == Color) 
			 {
				  if (  (FinalImage.at<Vec3b>(y,x)[0] == Color) &&
						(FinalImage.at<Vec3b>(y,x)[1] == Color) &&
						(FinalImage.at<Vec3b>(y,x)[2] == Color)    )
					{
						 FinalImage.at<Vec3b>(y,x)[0] = Paint;	 
						 FinalImage.at<Vec3b>(y,x)[1] = Paint;	 
						 FinalImage.at<Vec3b>(y,x)[2] = Paint;
					 }
  				   y--;
			}	 
         }		  
         
        // LEFT RIGHT
	    for( y = 0; y < ImageWithEdge.rows; y++ )
        {
			 x = 0;
			 while (x < ImageWithEdge.cols && (FinalImage.at<Vec3b>(y,x)[0] == Color || FinalImage.at<Vec3b>(y,x)[0] == Paint))  
			 {
				  if (  (FinalImage.at<Vec3b>(y,x)[0] == Color) &&
						(FinalImage.at<Vec3b>(y,x)[1] == Color) &&
						(FinalImage.at<Vec3b>(y,x)[2] == Color)    )
					{
						 FinalImage.at<Vec3b>(y,x)[0] = Paint;	 
						 FinalImage.at<Vec3b>(y,x)[1] = Paint;	 
						 FinalImage.at<Vec3b>(y,x)[2] = Paint;
					 }
				   x++;
			}	 
         }		  
         
        // RIGH LEFT
	    for( y = 0; y < ImageWithEdge.rows; y++ )
        {
			 x = ImageWithEdge.cols -1;
			 while (x >=0  && (FinalImage.at<Vec3b>(y,x)[0] == Color || FinalImage.at<Vec3b>(y,x)[0] == Paint))  
			 {
				  if (  (FinalImage.at<Vec3b>(y,x)[0] == Color) &&
						(FinalImage.at<Vec3b>(y,x)[1] == Color) &&
						(FinalImage.at<Vec3b>(y,x)[2] == Color)    )
					{
						 FinalImage.at<Vec3b>(y,x)[0] = Paint;	 
						 FinalImage.at<Vec3b>(y,x)[1] = Paint;	 
						 FinalImage.at<Vec3b>(y,x)[2] = Paint;
					 }
				   x--;
			}	 
         }		  
	     
	     ShowImage(FinalImage,"Koalla");

        //default is go 
  		velY = 0;
		velX = 0.2;
		
		int freeside = FreeSide(FinalImage);
		
		ROS_INFO("freeside = %d\n", freeside);
		        
		 switch (freeside) {
		    case -1: {
						printf("\nParar");
						velY = 0;
						velX = 0;
						break;
					};
			case 0: {
						printf("\nMover para a esquerda");
						velY = 0.5;
						velX = 0.8; //= 0 MEANS DON'T GO TO FRONT
						break;
					};
            case 1:	{
						printf("\nMover para frente");
						velY = 0;
						velX = 0.8;
						break;
					};
			case 2: {
                        printf("\nMover para direita");
                        velY = -0.5;
                        velX = 0.8; //= 0 MEANS DON'T GO TO FRONT
                        break;
                    };
         }       
           
}

//----------------------------------------------------------------------
int FreeSide(Mat Image){
	//==============================================================
	// Identify the side with more space free
	// and return
	// -1 => there are not space free to cross
	//  n >> quadrante n is free, 0 <= n <= 2
	//==============================================================
	//23/03/17-wander
	int limiar; // valor limite de cinza.
				// Acima dele pintar acender o pixel (255-branco), e
				// abaixo dele apagar o pixel (0-preto)

	int vTercaParte;
	vTercaParte = Image.cols / 3;
	int side[3] = {0,0,0};   
	double MaxObstacle = 1; //0.8;
	double FreePercent = 1;
	    	
	// count the almost painted pixels (free space) in each side
	for( int y = 0; y < Image.rows; y++ )
	 for( int x = 0; x < Image.cols; x++ )
	  for( int c = 0; c < 3; c++ )
	    if (Image.at<Vec3b>(y,x)[0] != Paint) 
	    {
			if ( x < vTercaParte) 
			   side[0]++; // Left Side
			else   
			if ( (x >= vTercaParte) && (x < 2*vTercaParte) )
			   side[1]++; // Center
			else
			if (x >= 2*vTercaParte) 
			   side[2]++; // Right Side
		}
                                       
	//Show the almost of black pixels (obstacles) in each side
	printf("\nQuadrante da esquerda com %i pixels free\n",side[0]);
	printf("Quadrante do centro com %i pixels free\n"    ,side[1]);
	printf("Quadrante da direita com %i pixels free\n"   ,side[2]);
    /*
        =========================================
        ||			 ||           ||           ||
        ||   side    ||   side    ||   side    ||
        ||     0     ||     1	  ||     2     ||
        ||			 ||           ||           || 
        =========================================
    */
    // DECIDE MORE FREE WAY TO DRONE FLY
	// (THE BEST WAY IS THE FEWER OBSTACLES AREA (LESS BLACK PIXELS)
	/*MaxObstacle = MaxObstacle * Image.rows * Image.cols; 
    int MoreFree = -1;
    for( int i = 0; i <= 2; i++ )
      if ((side[i] > FreePercent * MoreFree) && (side[i] > MaxObstacle))
		 MoreFree = i;*/
		 
	//int MoreFree = 0;
	//for( int i = 1; i <= 2; i++ )
    //  if (side[i] > side[MoreFree])
	//	 MoreFree = i;
		 
	
	int MoreFree;
	MaxObstacle = Image.rows * Image.cols * 80/100;
	if (  (side[0] > MaxObstacle)
		& (side[1] > MaxObstacle)
		& (side[2] > MaxObstacle)
	   )
	{
		MoreFree = -1;
	}
    else
    { 
		if ((side[0] < side[1]) && (side[0] < side[2]) )
			MoreFree = 0;
		else
			if ((side[1] < side[0]) && (side[1] < side[2]) )
				MoreFree = 1;
			else
				if ((side[2] < side[0]) && (side[2] < side[1]) )
					MoreFree = 2;       
				else					
					MoreFree = 1;
    }
		 
	return MoreFree;
}


void enableControlCb(const std_msgs::BoolConstPtr &msg)
{
    enableControl = msg->data;
}

/*
 * STEREO CORRESPONDENCE MIRCEA PAUL MURESAN
 * https://www.youtube.com/watch?time_continue=71&v=OUbUFn71S4s
 * OpenCV GSOC 2015 
 *
 * EGGN 512 
 * https://www.youtube.com/watch?v=kxsvG4sSuvA
 * 
KOALLA - Algoritmo para evitar obstáculos usando visão computacional

- Instalação:

  	cd ~/simulation/ros_catkin_ws/src
	git clone https://github.com/viscap/koalla.git
	cd ~/simulation/ros_catkin_ws
	catkin_make --pkg koalla
	
- Execução:

- Executar o simulador da Pixhawk (SITL):

	source ~/simulation/ros_catkin_ws/devel/setup.bash
	cd ~/simulation/ardupilot/ArduCopter
	../Tools/autotest/sim_vehicle.sh -j 4 -f Gazebo
	
	
- Em outro terminal, executar o Gazebo, utilizando o arquivo launch do pacote koalla

	source ~/simulation/ros_catkin_ws/devel/setup.bash
	roslaunch koalla koalla.launch
	
	
	OBS: Para abrir mundos diferentes do padrão, passar endereço do arquivo world como argumento:
	roslaunch koalla koalla.launch world_name:=[endereco_da_pasta_home]/simulation/ros_catkin_ws/src/koalla/worlds/world1.world
	

- Na janela do Gazebo, mudar a velocidade da simulaçao: 

	Do lado esquerdo, clicar em Physics
	Mudar o valor do parametro max_step_size de 0,0025 para 0,001

	
- Decolar o drone - executar os seguintes comandos no MavProxy:

	mode GUIDED
	arm throttle
	takeoff 1
	
	
- Mandar o ROS salvar a posição do drone ao longo do tempo em um arquivo bag

	cd ~/simulation/ros_catkin_ws/src/koalla/bagfiles
	rosbag record -O [nome_do_arquivo] /mavros/local_position/pose
	

- O Koala passará a controlar o drone assim que ele mudar para o modo LOITER: Digitar no MavProxy:

	mode loiter


- Após realizar a simulação, lembrar de parar o rosbag dando um Ctrl + C no terminal em que ele foi executado.

#####################################
* 
Webcam com microfone prata

Modelo 1819

• conexão: USB. Resolução: 1,3 MP

• conecte e use (driver free). Microfone embutido, foto e vídeo.

• LED para ambientes escuros

• interatividade: Skype; MSN; Yahoo; Messenger. Compatível com todas as versões do Windows

• base clip: fixação para notebook e tela LCD e mesa.

• foco ajustável

• tipo de embalagem: blister

• cód. de barras: 7898940742 22 6

* http://www.pisc.com.br/?p=112
* user manual: http://www.pisc.com.br/manuais/manual_webcam.pdf
* download driver: http://www.pisc.com.br/drivers/webcam_driver.zip
* ################################
* 
* 
* Installation in Linux {#tutorial_linux_install}
=====================

These steps have been tested for Ubuntu 10.04 but should work with other distros as well.

Required Packages
-----------------

-   GCC 4.4.x or later
-   CMake 2.8.7 or higher
-   Git
-   GTK+2.x or higher, including headers (libgtk2.0-dev)
-   pkg-config
-   Python 2.6 or later and Numpy 1.5 or later with developer packages (python-dev, python-numpy)
-   ffmpeg or libav development packages: libavcodec-dev, libavformat-dev, libswscale-dev
-   [optional] libtbb2 libtbb-dev
-   [optional] libdc1394 2.x
-   [optional] libjpeg-dev, libpng-dev, libtiff-dev, libjasper-dev, libdc1394-22-dev
-   [optional] CUDA Toolkit 6.5 or higher

The packages can be installed using a terminal and the following commands or by using Synaptic
Manager:
@code{.bash}
[compiler] sudo apt-get install build-essential
[required] sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
[optional] sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev
@endcode
Getting OpenCV Source Code
--------------------------

You can use the latest stable OpenCV version or you can grab the latest snapshot from our [Git
repository](https://github.com/opencv/opencv.git).

### Getting the Latest Stable OpenCV Version

-   Go to our [downloads page](http://opencv.org/downloads.html).
-   Download the source archive and unpack it.

### Getting the Cutting-edge OpenCV from the Git Repository

Launch Git client and clone [OpenCV repository](http://github.com/opencv/opencv). If you need
modules from [OpenCV contrib repository](http://github.com/opencv/opencv_contrib) then clone it too.

For example
@code{.bash}
cd ~/<my_working_directory>
git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_contrib.git
@endcode
Building OpenCV from Source Using CMake
---------------------------------------

-#  Create a temporary directory, which we denote as \<cmake_build_dir\>, where you want to put
    the generated Makefiles, project files as well the object files and output binaries and enter
    there.

    For example
    @code{.bash}
    cd ~/opencv
    mkdir build
    cd build
    @endcode
-#  Configuring. Run cmake [\<some optional parameters\>] \<path to the OpenCV source directory\>

    For example
    @code{.bash}
    cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local ..
    @endcode
    or cmake-gui

    -   set full path to OpenCV source code, e.g. /home/user/opencv
    -   set full path to \<cmake_build_dir\>, e.g. /home/user/opencv/build
    -   set optional parameters
    -   run: “Configure”
    -   run: “Generate”

    @note
    Use `cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local ..` , without spaces after -D if the above example doesn't work.

-#  Description of some parameters
    -   build type: `CMAKE_BUILD_TYPE=Release\Debug`
    -   to build with modules from opencv_contrib set OPENCV_EXTRA_MODULES_PATH to \<path to
        opencv_contrib/modules/\>
    -   set BUILD_DOCS for building documents
    -   set BUILD_EXAMPLES to build all examples

-#  [optional] Building python. Set the following python parameters:
    -   PYTHON2(3)_EXECUTABLE = \<path to python\>
    -   PYTHON_INCLUDE_DIR = /usr/include/python\<version\>
    -   PYTHON_INCLUDE_DIR2 = /usr/include/x86_64-linux-gnu/python\<version\>
    -   PYTHON_LIBRARY = /usr/lib/x86_64-linux-gnu/libpython\<version\>.so
    -   PYTHON2(3)_NUMPY_INCLUDE_DIRS =
        /usr/lib/python\<version\>/dist-packages/numpy/core/include/

-#  [optional] Building java.
    -   Unset parameter: BUILD_SHARED_LIBS
    -   It is useful also to unset BUILD_EXAMPLES, BUILD_TESTS, BUILD_PERF_TESTS - as they all
        will be statically linked with OpenCV and can take a lot of memory.

-#  Build. From build directory execute make, recomend to do it in several threads

    For example
    @code{.bash}
    make -j7 # runs 7 jobs in parallel
    @endcode
-#  [optional] Building documents. Enter \<cmake_build_dir/doc/\> and run make with target
    "html_docs"

    For example
    @code{.bash}
    cd ~/opencv/build/doc/
    make -j7 html_docs
    @endcode
-#  To install libraries, from build directory execute
    @code{.bash}
    sudo make install
    @endcode
-#  [optional] Running tests

    -   Get the required test data from [OpenCV extra
        repository](https://github.com/opencv/opencv_extra).

    For example
    @code{.bash}
    git clone https://github.com/opencv/opencv_extra.git
    @endcode
    -   set OPENCV_TEST_DATA_PATH environment variable to \<path to opencv_extra/testdata\>.
    -   execute tests from build directory.

    For example
    @code{.bash}
    <cmake_build_dir>/bin/opencv_test_core
    @endcode

@note
   If the size of the created library is a critical issue (like in case of an Android build) you
    can use the install/strip command to get the smallest size as possible. The *stripped* version
    appears to be twice as small. However, we do not recommend using this unless those extra
    megabytes do really matter.
 */
 
 
 
 
 /*

- Execução:

- Executar o simulador da Pixhawk (SITL):

	source ~/simulation/ros_catkin_ws/devel/setup.bash
	cd ~/simulation/ardupilot/ArduCopter
	../Tools/autotest/sim_vehicle.sh -j 4 -f Gazebo
	
	
- Em outro terminal, executar o Gazebo, utilizando o arquivo launch do pacote koalla

	source ~/simulation/ros_catkin_ws/devel/setup.bash
	roslaunch koalla koalla_gazebo.launch
	
	
	OBS: Para abrir mundos diferentes do padrão, passar endereço do arquivo world como argumento:
	EXEMPLO:
	roslaunch koalla koalla.launch world_name:=[endereco_da_pasta_home]/simulation/ros_catkin_ws/src/koalla/worlds/world1.world
	OU:
	roslaunch koalla koalla.launch world_name:=[endereco_da_pasta_home]/simulation/ros_catkin_ws/src/koalla/worlds/outdoor_village.world
	

- Na janela do Gazebo, mudar a velocidade da simulaçao: 

	Do lado esquerdo, clicar em Physics
	Mudar o valor do parametro max_step_size de 0,0025 para 0,001

	
- Decolar o drone - executar os seguintes comandos no MavProxy:

	mode GUIDED
	arm throttle
	takeoff 1
	

- O Koala passará a controlar o drone assim que for publicada uma mensagem contendo "true" no tópico /koalla/enable_control

	rostopic pub -1 /koalla/enable_contr std_msgs/Bool "data: true"


- Mandar o ROS salvar a posição do drone ao longo do tempo em um arquivo bag

	cd ~/simulation/ros_catkin_ws/src/koalla/bagfiles
	rosbag record -O [nome_do_arquivo] /mavros/local_position/pose


- Após realizar a simulação, lembrar de parar o rosbag dando um Ctrl + C no terminal em que ele foi executado.

 
 */
