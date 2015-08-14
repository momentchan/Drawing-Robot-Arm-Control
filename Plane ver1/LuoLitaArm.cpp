#include "LuoLitaArm.h"

using namespace std;
using namespace Eigen;

//-----------------------------
//------Global Variable--------
//-----------------------------

// function prototype for periodic timer function
void RTFCNDCL TimerHandler1( void * nContext );

// Draw Point Information
float paperSize = 25;
float imageSize = 400;
float centerX = -0.26;
float centerY = 0.60;
int insertNum = 0;

float diff_x = 0.11;
float diff_y = -0.08;

// write out trace point
ofstream outputFile;
bool write = false;

//Sketch
vector<vector<Eigen::Vector2f>> sketchPositionXY;
//vector<vector<float>> sketch_pos_y;
//vector<vector<float>> sketch_pos_x;

//Fill
vector<int> colorIndex;
vector<vector<vector<float>>> fill_pos_x;
vector<vector<vector<float>>> fill_pos_y;


int sketchNumbers = 0;
int fillNumbers = 0;
int sketchIndex = 0;
int fillIndex = 0;

// First Point Position
float first_x = 0.4958f;
float first_y = 0.2738;//-0.05415f;;
float first_z = -0.1526f;

char kbCmd_3;
float z_axis_color = -0.0961;
float z_axis_black = -0.15;

float speed=0;
bool initialDone = false;

vector<Eigen::Vector3f> trace;

//*** add finger control
Finger finger;
Vector6f t;
Matrix4f T;
float pen_pos_x;
float pen_pos_y;

int color_index;
Vector2f ColorPosition( int color_index )
{
	int dy_num;
	int dx_num;

	dx_num = (color_index-1) /8;
	dy_num = (color_index-1) - dx_num *8;

	Vector2f dxy;
	dxy(0) = (float)(dx_num*diff_x);
	dxy(1) = (float)(dy_num*diff_y);
	return dxy;
}
//***
void InitialPose_keyboard(){
	switch( kbCmd ){
		case '-':
			if(speed>0.2) speed-=(float)0.1;
			setDefaultArmSpeed(speed);
			break;
		case '+':
			if(speed<1.2) speed+=(float)0.1;
			setDefaultArmSpeed(speed);
			break;
		case 'a':
			t << -0.001f,0,0,0,0,0;
			Move_L_Rel(t,0);
			first_x = T07Cmd(0,3);
			break;
		case 'd':
			t << 0.001f,0,0,0,0,0;
			Move_L_Rel(t,0);
			first_x = T07Cmd(0,3);
			break;
		case 'w':
			t << 0,0.001f,0,0,0,0;
			Move_L_Rel(t,0);
			first_y = T07Cmd(1,3);
			break;
		case 's':
			t << 0,-0.001f,0,0,0,0;
			Move_L_Rel(t,0);
			first_y = T07Cmd(1,3);
			break;
		case 'r':
			t << 0,0,0.01f,0,0,0;
			Move_L_Rel(t,0);
			while(MOVL){};
			first_z = T07Cmd(2,3);
			break;
		case 'f':
			t << 0,0,-0.01f,0,0,0;
			Move_L_Rel(t,0);
			while(MOVL){};
			first_z = T07Cmd(2,3);
			break;
		case 'z':
			
			T <<  -1.0f,  0.0f,  0.0f,  first_x, //
				   0.0f,  1.0f,  0.0f,  first_y, //
				   0.0f,  0.0f, -1.0f,  first_z, //-0.2726
				   0.0f,  0.0f,  0.0f,  1.0f;
			Move_L_Abs(T,0);
			
			while(MOVL){system("cls");
					DisplayLoop(speed);}
			break;
		case 'j':
			t << -0.11f,0,0,0,0,0;
			Move_L_Rel(t,0);
			break;
		case 'l':
			t << 0.11f,0,0,0,0,0;
			Move_L_Rel(t,0);
			break;
		case 'i':
			t << 0,0.08f,0,0,0,0;
			Move_L_Rel(t,0);
			break;
		case 'k':
			t << 0,-0.08f,0,0,0,0;
			Move_L_Rel(t,0);
			break;
		case 'b':
			t << 0,0,-0.12f,0,0,0;
			Move_L_Rel(t,0);
			while(MOVL){};
			finger.close();
			t << 0,0,0.12f,0,0,0;
			Move_L_Rel(t,0);
			break;
		case 'n':
			finger.move(80);
			break;
		case 'o':
			outputFile.open("trace.txt");
			write=true;
			t << 0.10f,0,0,0,0,0;
			Move_L_Rel(t,0);
			while(MOVL){};
			write=false;
			outputFile.close();
			break;
		case 'm':
			Move_Contour();
			break;
		case 'p':
			z_axis_black = -0.20;
			//draw edge
			T <<  -1.0f,  0.0f,  0.0f,  sketchPositionXY[sketchIndex][0](0), //
					0.0f,  1.0f,  0.0f,  sketchPositionXY[sketchIndex][0](1), //
					0.0f,  0.0f, -1.0f,  z_axis_black+(float)0.01, //
					0.0f,  0.0f,  0.0f,  1.0f;
			Move_L_Abs(T,0);
			while(MOVL){system("cls");	DisplayLoop(speed);}
			T <<  -1.0f,  0.0f,  0.0f,  sketchPositionXY[sketchIndex][0](0), //
					0.0f,  1.0f,  0.0f,  sketchPositionXY[sketchIndex][0](1), //
					0.0f,  0.0f, -1.0f,  z_axis_black, //
					0.0f,  0.0f,  0.0f,  1.0f;
			Move_L_Abs(T,0);
			while(MOVL){system("cls");	DisplayLoop(speed);}

			Move_Contour();
			
			//Move_L_Abs(T,0);
			while(MOVL){ system("cls");	DisplayLoop(speed);	}

			T <<  -1.0f,  0.0f,  0.0f,  sketchPositionXY[sketchIndex][sketchPositionXY[sketchIndex].size()-1](0), //
					0.0f,  1.0f,  0.0f,  sketchPositionXY[sketchIndex][sketchPositionXY[sketchIndex].size()-1](1), //
					0.0f,  0.0f, -1.0f,  z_axis_black+(float)0.01, //
					0.0f,  0.0f,  0.0f,  1.0f;
			Move_L_Abs(T,0);
			while(MOVL){system("cls");
						DisplayLoop(speed);}
			//sketchIndex++;
			break;
	}
	kbCmd_3 = kbCmd;
	kbCmd = ' ';
}
void MyLoop_keyboard(){
	//Vector6f t;
	//Matrix4f T;
	
	switch( kbCmd ){
		case 'c':
			sketchIndex = 0;
			fillIndex = 0;
			break;
		case 'v':
			fillIndex = 0;
			break;
		case '-':
			if(speed>0.2) speed-=(float)0.1;
			setDefaultArmSpeed(speed);
			break;
		case '+':
			if(speed<1.2) speed+=(float)0.1;
			setDefaultArmSpeed(speed);
			break;
		case 'w':
			t << 0,0.05f,0,0,0,0;
			Move_L_Rel(t,0);
			break;
		case 's':
			t << 0,-0.05f,0,0,0,0;
			Move_L_Rel(t,0);
			break;
		case 'a':
			t << -0.05f,0,0,0,0,0;
			Move_L_Rel(t,0);
			break;
		case 'd':
			t << 0.05f,0,0,0,0,0;
			Move_L_Rel(t,0);
			break;
		case 'r':
			t << 0,0,0.001f,0,0,0;
			Move_L_Rel(t,0);
			while(MOVL){};
			z_axis_color = T07Cmd(2,3);
			break;
		case 'f':
			t << 0,0,-0.001f,0,0,0;
			Move_L_Rel(t,0);
			while(MOVL){};
			z_axis_color = T07Cmd(2,3);
			break;
		case 't':
			t << 0,0,0.001f,0,0,0;
			Move_L_Rel(t,0);
			while(MOVL){};
			z_axis_black = T07Cmd(2,3);
			break;
		case 'g':
			t << 0,0,-0.001f,0,0,0;
			Move_L_Rel(t,0);
			while(MOVL){};
			z_axis_black = T07Cmd(2,3);
			break;
		case 'y':
			T <<  -1.0f,  0.0f,  0.0f,  centerX, //
				   0.0f,  1.0f,  0.0f,  centerY, //
				   0.0f,  0.0f, -1.0f,  0.0f, //
				   0.0f,  0.0f,  0.0f,  1.0f;
			Move_L_Abs(T,0);
			while(MOVL){};
			break;
		case 'h':
			T <<  -1.0f,  0.0f,  0.0f,  centerX, //
				   0.0f,  1.0f,  0.0f,  centerY, //
				   0.0f,  0.0f, -1.0f,  -0.085f, //
				   0.0f,  0.0f,  0.0f,  1.0f;
			Move_L_Abs(T,0);
			break;
		case 'u': // tune short pen
			// Go to position of pen
			T <<  -1.0f,  0.0f,  0.0f,  first_x, //
				0.0f,  1.0f,  0.0f,  first_y, //
				0.0f,  0.0f, -1.0f,  first_z, //-0.2726
				0.0f,  0.0f,  0.0f,  1.0f;
			Move_L_Abs(T,0);
			while(MOVL){system("cls");	DisplayLoop(speed);}

			// Grasp pen
			t << 0,0,-0.12f,0,0,0;
			Move_L_Rel(t,0);
			while(MOVL){};
			finger.close();
			t << 0,0,0.12f,0,0,0;
			Move_L_Rel(t,0);
			while(MOVL){};

			// Move to via point position
			T <<  -1.0f,  0.0f,  0.0f,  first_x, //
				0.0f,  1.0f,  0.0f,  first_y+0.1, //
				0.0f,  0.0f, -1.0f,  first_z+0.13, //-0.2726
				0.0f,  0.0f,  0.0f,  1.0f;
			Move_L_Abs(T,0);
			while(MOVL){system("cls");	DisplayLoop(speed);}

			// Go to center
			T <<  -1.0f,  0.0f,  0.0f,  centerX, //
				0.0f,  1.0f,  0.0f,  centerY, //
				0.0f,  0.0f, -1.0f,  0.0f, //
				0.0f,  0.0f,  0.0f,  1.0f;
			Move_L_Abs(T,0);
			while(MOVL){system("cls"); DisplayLoop(speed);}
			T <<  -1.0f,  0.0f,  0.0f,  centerX, //
				   0.0f,  1.0f,  0.0f,  centerY, //
				   0.0f,  0.0f, -1.0f,  -0.12f, //
				   0.0f,  0.0f,  0.0f,  1.0f;
			Move_L_Abs(T,0);
			break;
		case 'o':
			// Go to position of pen
			T <<  -1.0f,  0.0f,  0.0f,  first_x, //
				0.0f,  1.0f,  0.0f,  first_y, //
				0.0f,  0.0f, -1.0f,  first_z, //-0.2726
				0.0f,  0.0f,  0.0f,  1.0f;
			Move_L_Abs(T,0);
			while(MOVL){system("cls");	DisplayLoop(speed);}

			// Grasp pen
			t << 0,0,-0.12f,0,0,0;
			Move_L_Rel(t,0);
			while(MOVL){};
			finger.close();

			// Move to via point position
			T <<  -1.0f,  0.0f,  0.0f,  first_x, //
					0.0f,  1.0f,  0.0f,  first_y+0.1, //
					0.0f,  0.0f, -1.0f,  first_z+0.13, //-0.2726
					0.0f,  0.0f,  0.0f,  1.0f;
			Move_L_Abs(T,0);
			while(MOVL){system("cls");	DisplayLoop(speed);}

			// Go to center
			T <<  -1.0f,  0.0f,  0.0f,  centerX, //
				0.0f,  1.0f,  0.0f,  centerY, //
				0.0f,  0.0f, -1.0f,  0.0f, //
				0.0f,  0.0f,  0.0f,  1.0f;
			Move_L_Abs(T,0);
			while(MOVL){system("cls"); DisplayLoop(speed);}

			//draw edge
			while(sketchIndex < sketchNumbers){
				T <<  -1.0f,  0.0f,  0.0f,  sketchPositionXY[sketchIndex][0](0), //
					0.0f,  1.0f,  0.0f,  sketchPositionXY[sketchIndex][0](1), //
					0.0f,  0.0f, -1.0f,  z_axis_black+(float)0.01, //
					0.0f,  0.0f,  0.0f,  1.0f;
				Move_L_Abs(T,0);
				while(MOVL){system("cls");	DisplayLoop(speed);}

				T <<  -1.0f,  0.0f,  0.0f,  sketchPositionXY[sketchIndex][0](0), //
					0.0f,  1.0f,  0.0f,  sketchPositionXY[sketchIndex][0](1), //
					0.0f,  0.0f, -1.0f,  z_axis_black, //
					0.0f,  0.0f,  0.0f,  1.0f;
				Move_L_Abs(T,0);
				while(MOVL){system("cls");  DisplayLoop(speed);}

				for(unsigned int i=0;i<sketchPositionXY[sketchIndex].size();i++){
					T <<  -1.0f,  0.0f,  0.0f,  sketchPositionXY[sketchIndex][i](0), //
						   0.0f,  1.0f,  0.0f,  sketchPositionXY[sketchIndex][i](1), //
						   0.0f,  0.0f, -1.0f,  z_axis_black, //
						   0.0f,  0.0f,  0.0f,  1.0f;
					//Move_Contour(T,0);
					Move_L_Abs(T,0);
					while(MOVL){
						system("cls"); 
						DisplayLoop(speed);
					}
				}

				T <<  -1.0f,  0.0f,  0.0f,  sketchPositionXY[sketchIndex][sketchPositionXY[sketchIndex].size()-1](0), //
						0.0f,  1.0f,  0.0f,  sketchPositionXY[sketchIndex][sketchPositionXY[sketchIndex].size()-1](1), //
						0.0f,  0.0f, -1.0f,  z_axis_black+(float)0.01, //
						0.0f,  0.0f,  0.0f,  1.0f;
				Move_L_Abs(T,0);
				while(MOVL){system("cls");	DisplayLoop(speed);}
				sketchIndex++;
			}
			break;
		case 'p':
			// fill every areas
			if(fillIndex < fillNumbers){
				Vector2f dxy = ColorPosition(fillIndex+1);
				// Go to position of pen
				T <<  -1.0f,  0.0f,  0.0f,  first_x+dxy(0), //
				   0.0f,  1.0f,  0.0f,  first_y+dxy(1), //
				   0.0f,  0.0f, -1.0f,  first_z, //-0.2726
				   0.0f,  0.0f,  0.0f,  1.0f;
				Move_L_Abs(T,0);
				while(MOVL){system("cls");	DisplayLoop(speed);}

				// Grasp pen
				t << 0,0,-0.12f,0,0,0;
				Move_L_Rel(t,0);
				while(MOVL){};
				finger.close();
				t << 0,0,0.20f,0,0,0;
				Move_L_Rel(t,0);
				while(MOVL){};

				// Move to via point position
				T <<  -cos(0.25*M_PI),  sin(0.25*M_PI),  0.0f,  first_x, //
					sin(0.25*M_PI),  cos(0.25*M_PI),  0.0f,  first_y+0.1, //
					0.0f,  0.0f, -1.0f,  first_z+0.13, //-0.2726
					0.0f,  0.0f,  0.0f,  1.0f;
				Move_L_Abs(T,0);
				while(MOVL){system("cls");	DisplayLoop(speed);}

				// Go to center
				T <<  -cos(0.25*M_PI),  sin(0.25*M_PI),  0.0f,  centerX, //
				   sin(0.25*M_PI),  cos(0.25*M_PI),  0.0f,  centerY, //
				   0.0f,  0.0f, -1.0f,  0.0f, //
				   0.0f,  0.0f,  0.0f,  1.0f;
				Move_L_Abs(T,0);
				while(MOVL){system("cls"); DisplayLoop(speed);}
								
				T <<  -cos(0.25*M_PI),  sin(0.25*M_PI),  0.0f,  fill_pos_x[fillIndex][0][0], //
					   sin(0.25*M_PI),  cos(0.25*M_PI),  0.0f,  fill_pos_y[fillIndex][0][0], //
					   0.0f,  0.0f, -1.0f,  z_axis_color+(float)0.01, //
					   0.0f,  0.0f,  0.0f,  1.0f;
				Move_L_Abs(T,0);
				while(MOVL){system("cls"); DisplayLoop(speed);}

				for(unsigned int i=0;i<fill_pos_x[fillIndex].size();i++){
					// move to the top of first point of every area
					T <<  -cos(0.25*M_PI),  sin(0.25*M_PI),  0.0f,  fill_pos_x[fillIndex][i][0], //
					      sin(0.25*M_PI),  cos(0.25*M_PI),  0.0f,  fill_pos_y[fillIndex][i][0], //
					       0.0f,  0.0f, -1.0f,  z_axis_color+(float)0.005, //
					       0.0f,  0.0f,  0.0f,  1.0f;
					Move_L_Abs(T,0);
					while(MOVL){system("cls"); DisplayLoop(speed);};
					
					for(unsigned int j=0;j<fill_pos_x[fillIndex][i].size();j++){ 
					  T <<  -cos(0.25*M_PI),  sin(0.25*M_PI),  0.0f,  fill_pos_x[fillIndex][i][j], //
						   sin(0.25*M_PI),  cos(0.25*M_PI),  0.0f,  fill_pos_y[fillIndex][i][j], //
						   0.0f,  0.0f, -1.0f,  z_axis_color, //
						   0.0f,  0.0f,  0.0f,  1.0f;
					  Move_L_Abs(T,0);
					  while(MOVL){system("cls"); DisplayLoop(speed);}
					}
				}
				T <<  -cos(0.25*M_PI),  sin(0.25*M_PI),  0.0f,  fill_pos_x[fillIndex][fill_pos_x[fillIndex].size()-1][0], //
				      sin(0.25*M_PI),  cos(0.25*M_PI),  0.0f,  fill_pos_y[fillIndex][fill_pos_x[fillIndex].size()-1][0], //
				       0.0f,  0.0f, -1.0f,  z_axis_color+(float)0.01, //
					   0.0f,  0.0f,  0.0f,  1.0f;
				Move_L_Abs(T,0);
				while(MOVL){system("cls");	DisplayLoop(speed);}
				fillIndex++;

				// Move to pen box position
				T <<  -1.0f,  0.0f,  0.0f,  first_x, //
					0.0f,  1.0f,  0.0f,  first_y+0.13, //
					0.0f,  0.0f, -1.0f,  first_z+0.13, //-0.2726
					0.0f,  0.0f,  0.0f,  1.0f;
				Move_L_Abs(T,0);
				while(MOVL){system("cls");	DisplayLoop(speed);}
				
				// Drop pen
				t << 0,0,-0.11f,0,0,0;
				Move_L_Rel(t,0);
				while(MOVL){};
				finger.move(80);
				t << 0,0,0.11f,0,0,0;
				Move_L_Rel(t,0);
			}
			break;
		case 'b':
			finger.close();
			break;
		case 'n':
			finger.move(80);
			break;
	}

	kbCmd_3 = kbCmd;
	kbCmd = ' ';
}

void readTrace(){
	// Read sketch points
	ifstream file("trace.txt");
	string str;
	while (getline(file, str))
	{
		vector<string> sep = split(str, ' ');
		Vector3f point;
		point(0)=stof(sep[0]);
		point(1)=stof(sep[1]);
		point(2)=stof(sep[2]);
		trace.push_back(point);
	}
	cout<<"trace point"<<trace.size()<<endl;
}
void readDrawPoints(){
	//count how many files in drawPoints directory
	WIN32_FIND_DATA fd;	
	HANDLE h = FindFirstFile(TEXT("drawPoints/sketch*.txt"), &fd);
	if (h != INVALID_HANDLE_VALUE) {
		do {
			sketchNumbers++;
		} while (FindNextFile(h, &fd));
		FindClose(h);
	}
	h = FindFirstFile(TEXT("drawPoints/fill*.txt"), &fd);
	if (h != INVALID_HANDLE_VALUE) {
		do {
			fillNumbers++;
		} while (FindNextFile(h, &fd));
		FindClose(h);
	}

	// Read sketch points
	for(int i=0; i<sketchNumbers; i++){
		string file_num = int2str(i);
		string file_name = "drawPoints/sketch";
		file_name.append(file_num);
		file_name.append(".txt");		
		ifstream file(file_name);
		string str;
		vector<Eigen::Vector2f> sub_sketchPositionXY;
		while (getline(file, str))
		{
			vector<string> sep = split(str, ' ');   
			float x = (-1)*(stoi(sep[0])-200)*(float)(paperSize/imageSize)/100+centerX;
			float y = (stoi(sep[1])-200)*float(paperSize/imageSize)/100+centerY;
			// insert point
			if(sub_sketchPositionXY.size()>0){
				// second point
				float pre_pos_x = sub_sketchPositionXY[sub_sketchPositionXY.size()-1](0);
				float pre_pos_y = sub_sketchPositionXY[sub_sketchPositionXY.size()-1](1);
				float dx = (x-pre_pos_x)/(insertNum+1);
				float dy = (y-pre_pos_y)/(insertNum+1);

				for(int j=1;j<insertNum+1;j++){
					sub_sketchPositionXY.push_back(Vector2f(pre_pos_x+dx*j,pre_pos_y+dy*j));
				}
				sub_sketchPositionXY.push_back(Vector2f(x,y));
			}
			// For the first point
			else
			{
				sub_sketchPositionXY.push_back(Vector2f(x,y));
			}
		}
		sketchPositionXY.push_back(sub_sketchPositionXY);
	}
	
	/*for(int i=0;i<sketchPositionXY[0].size();i++){
		cout<<sketchPositionXY[0][i](0)<<" "<<sketchPositionXY[0][i](0)<<endl;
	}
	cout<<sketchPositionXY[0].size();*/

	// Read fill points
	for(int i=0; i<fillNumbers; i++){
		string file_num = int2str(i);
		string file_name = "drawPoints/fill";
		file_name.append(file_num);
		file_name.append(".txt");		
		ifstream file(file_name);
		string str;

		vector<vector<float>> sub_fill_pos_x;
		vector<vector<float>> sub_fill_pos_y;

		// Reading the color index;
		getline(file, str);
		vector<string> sep = split(str, ' ');
		colorIndex.push_back(stoi(sep[0]));

		while (getline(file, str))
		{
			vector<string> sep = split(str, ' ');
			float x1 = (-1)*(stoi(sep[0])-200)*(float)(paperSize/imageSize)/100+centerX;
			float y1 = (stoi(sep[1])-200)*(float)(paperSize/imageSize)/100+centerY;
			float x2 = (-1)*(stoi(sep[2])-200)*(float)(paperSize/imageSize)/100+centerX;
			float y2 = (stoi(sep[3])-200)*(float)(paperSize/imageSize)/100+centerY;
			vector <float> pos_x;
			vector <float> pos_y;
			pos_x.push_back(x1);
			pos_y.push_back(y1);
			pos_x.push_back(x2);
			pos_y.push_back(y2);
			//cout<<x1<<" "<<y1<<" "<<x2<<" "<< y2<<endl;
			sub_fill_pos_y.push_back(pos_y);
			sub_fill_pos_x.push_back(pos_x);
		}
		fill_pos_y.push_back(sub_fill_pos_y);
		fill_pos_x.push_back(sub_fill_pos_x);
	}

}
int main( int argc, char **argv, char **envp)
{
	readDrawPoints();
	//readTrace();
	bool run =true;//
	if(run){
	init_LuoLita_1();

	LitaHand.GripperMove_Abs_To(75.0f, 1);

    // for periodic timer code
    LARGE_INTEGER  liPeriod_1ms;   // timer period
    HANDLE         hTimer1;     // timer handle
	//LARGE_INTEGER  liPeriod_10ms;   // timer period
    //HANDLE         hTimer2;     // timer handle

    //  RTX periodic timer code:
    //  TO DO: Set default timer period to your desired time.
    //         The period needs to be an even multiple of the HAL
    //         period found in the control panel.
    //         This example uses a period of 500 micro seconds.

    liPeriod_1ms.QuadPart  = 10000;
	//liPeriod_10ms.QuadPart = 100000;

	Init_IMPCard();
	// clear timer
	//RtDeleteTimer( hTimer1 );
	//RtDeleteTimer( hTimer2 );

    // Create a periodic timer
    if (! (hTimer1 = RtCreateTimer(
                                   NULL,            // security
                                   0,               // stack size - 0 uses default
                                   TimerHandler1,    // timer handler
                                   NULL,            // NULL context (argument to handler)
                                   RT_PRIORITY_MAX, // priority
                                   CLOCK_2) ))      // RTX HAL timer
    {
        //
        // TO DO:  exception code here
        // RtWprintf(L"RtCreateTimer error = %d\n",GetLastError());
        ExitProcess(1);
    }

    if (! RtSetTimerRelative( hTimer1, &liPeriod_1ms, &liPeriod_1ms ))
    {
        //RtWprintf(L"RtSetTimerRelative error = %d\n",GetLastError());
		// TO DO: exception code here
        ExitProcess(1);
    }
	
	init_LuoLita_2();
	LitaHand.GripperMove_Abs_To( 55.0f, 200);

	/////////////////////////////////////////
	kbCmd = 'j';
	MainLoop_keyboard();

	//Initial finger pose
	finger.init();
	finger.open();
	finger.setMode('p');
	finger.move(80);
	setDefaultArmSpeed(0.5);
	kbCmd = ' ';
	while(!initialDone)
	{
		if ( _kbhit() )
			 kbCmd = _getche();

		if ( kbCmd == kb_ESC )
			break;
		InitialPose_keyboard();
		Sleep(29);
		system("cls");
		DisplayLoop(speed);
		cout<<endl<<"first_x: "<<first_x<<"first_y: "<<first_y<<"first_z: "<<first_z<<endl;
	}
	cout<< endl << " Initialization is done . " <<endl;
	Sleep(2000);

	T <<  -1.0f,  0.0f,  0.0f,  first_x, //
			0.0f,  1.0f,  0.0f,  first_y+0.1, //
			0.0f,  0.0f, -1.0f,  first_z+0.13, //-0.2726
			0.0f,  0.0f,  0.0f,  1.0f;
	Move_L_Abs(T,0);
	while(MOVL){system("cls");	DisplayLoop(speed);}

	T <<  -1.0f,  0.0f,  0.0f,  centerX, //
			0.0f,  1.0f,  0.0f,  centerY, //
			0.0f,  0.0f, -1.0f,  0.0f, //
			0.0f,  0.0f,  0.0f,  1.0f;
	Move_L_Abs(T,0);
	while(MOVL){};

	kbCmd = ' ';
	while(1)
	{
		if ( _kbhit() )
			 kbCmd = _getche();

		if ( kbCmd == kb_ESC )
		{
			break;
		}
		MyLoop_keyboard();
		Sleep(29);
		system("cls");
		DisplayLoop(speed);
	}
	/////////////////////////////////////////
	LitaHand.GripperMove_Abs_To( 75.0f, 200);
	ByeBye();
	while(1)
	{
		if ( _kbhit() )
		{
			 kbCmd = _getch();
			 cout << kbCmd << endl;
			 if ( kbCmd == kb_ESC )
			 {
			     break;
		 	 }
		}
		Sleep(30);
		system("cls");
		DisplayLoop(speed);
	}

	LitaHand.GripperGoHome();
	Sleep(100);
	//

	//if(!RtDeleteTimer( hTimer2 ) )
	//{
 //       //RtWprintf(L"RtDeleteTimer error = %d\n",GetLastError());
	//	// TO DO:  your exception code here
 //       ExitProcess(1);
	//}
	//RtDeleteTimer( hTimer2 );
	
	if(!RtDeleteTimer( hTimer1 ) )
	{
        //RtWprintf(L"RtDeleteTimer error = %d\n",GetLastError());
		// TO DO:  your exception code here
		Close_IMPCard();
        ExitProcess(1);
	}
	//RtDeleteTimer( hTimer1 );
	
	//Close_IMPCard();
    Close_IMPCard();
	Sleep(1000);

	OutputData();

	ExitProcess(0);
	}
}
// main end

void RTFCNDCL TimerHandler1( PVOID context )
{
    // TO DO:  your timer handler code here

	ServoLoop();
}





string int2str(int i) {
	string s;
	stringstream ss(s);
	ss << i;
	return ss.str();
}

vector<string> split(string str, char delimiter) {
	vector<string> internal;
	stringstream ss(str); // Turn the string into a stream.
	string tok;
	while (getline(ss, tok, delimiter)) {
		internal.push_back(tok);
	}
	return internal;
}

void setDefaultArmSpeed(float percentage)
{
    Ang_Vel_limit = ( 0.01f*100 * deg2rad*RVmax / 1000.0f ) *SAMPLING_TIME_C * percentage;
    Ang_Acc_limit = ((0.01f*100 * deg2rad*RAmax / 1000.0f ) *SAMPLING_TIME_C) /1000 *SAMPLING_TIME_C * percentage;
    Ang_Dec_limit = Ang_Acc_limit * percentage;
    Lin_Vel_limit = ( 0.01f*160 * 0.001f*LVmax / 1000.0f ) *SAMPLING_TIME_C * percentage;
    Lin_Acc_limit = ((0.01f*320 * 0.001f*LAmax / 1000.0f ) *SAMPLING_TIME_C) /1000 *SAMPLING_TIME_C * percentage;
    Lin_Dec_limit = Lin_Acc_limit * percentage;
    Jn_Vel_limit = ( 0.01f*25* deg2rad*RVmax / 1000.0f ) *SAMPLING_TIME_C * percentage;
    Jn_Acc_limit = ((0.01f*25* deg2rad*RAmax / 1000.0f ) *SAMPLING_TIME_C) /1000 *SAMPLING_TIME_C * percentage;
    Jn_Dec_limit = Jn_Acc_limit * percentage;
}