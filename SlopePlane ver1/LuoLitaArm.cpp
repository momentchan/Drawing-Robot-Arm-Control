#include "LuoLitaArm.h"

using namespace std;
using namespace Eigen;

//-----------------------------
//------Global Variable--------
//-----------------------------

// function prototype for periodic timer function
void RTFCNDCL TimerHandler1( void * nContext );
float pi_f = (float) M_PI;

// Draw Point Information
float paperSize = 25;
float imageSize = 400;
float boardAngle = 7.025f/18.0f*pi_f;
float tuneAngle = 7.025f;
float displace = 0.002f;

int insertNum = 0;

float diff_x = -0.08f;
float diff_y = -0.11f;

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
float first_x = 0.2603f;//0.4958f;
float first_y = 0.7604f;//0.2738f;//-0.05415f;
float first_z = 0.02226f;//0.01726f;//-0.1526f;

float via_X = 0.5f;
float via_Y = 0.4f;
float via_Z = 0.035f;

char kbCmd_3;
float x_axis_color = 0.704f;//0.702f;
float z_axis_color = 0.03228f;//0.033f;
float x_axis_black = 0.704f;
float z_axis_black = 0.03228f;
float centerY = 0.005;//0.60f;

float speed=0.4f;
bool initialDone = false;
float blackDrop = 0.072f;
vector<Eigen::Vector3f> trace;

//*** add finger control
Finger finger;
Vector6f t;
Matrix4f T;
float pen_pos_x;
float pen_pos_y;

float wrist_rotate = -90;

float cos_ang = (float) cos(0.25*pi_f);
float sin_ang = (float) sin(0.25*pi_f);

int color_index;
Vector2f ColorPosition( int color_index )
{
	int dy_num;
	int dx_num;

	dy_num = (color_index-1) /8;
	dx_num = (color_index-1) - dy_num *8;

	Vector2f dxy;
	dxy(0) = (float)(dx_num*diff_x);
	dxy(1) = (float)(dy_num*diff_y);
	return dxy;
}
void GoToPoint(float x, float y, float z, float theta_roll, float theta_pitch, float theta_yaw){
	theta_roll /= 180.0;
	theta_roll *= pi_f;
	theta_pitch /= 180.0;
	theta_pitch *= pi_f;
	theta_yaw /= 180.0;
	theta_yaw *= pi_f;
	T <<  -cos(theta_pitch+theta_yaw),  sin(theta_yaw),  -sin(theta_pitch),  x,
			sin(theta_yaw),  cos(theta_roll+theta_yaw),  -sin(theta_roll),  y,
			sin(theta_pitch),  -sin(theta_roll), -cos(theta_roll+theta_pitch),  z,
			0.0f,  0.0f,  0.0f,  1.0f;
	Move_L_Abs(T,0);
	while(MOVL){system("cls");	DisplayLoop(speed);}
}

void MoveRelative(float x, float y, float z, float r, float p, float yaw){
	t << x,y,z,r,p,yaw;
	Move_L_Rel(t,0);
	while(MOVL){system("cls");	DisplayLoop(speed);}
}
void GraspDropPen(float d, bool state){
	t << 0,0,-d,0,0,0;
	Move_L_Rel(t,0);
	while(MOVL){system("cls");	DisplayLoop(speed);}

	if(state)finger.close();
	else finger.move(80);

	t << 0,0,d,0,0,0;
	Move_L_Rel(t,0);
	while(MOVL){system("cls");	DisplayLoop(speed);}
}

//***
void InitialPose_keyboard(){
	switch( kbCmd ){
		case '-':
			if(speed>0.2) speed-=0.1f;
			setDefaultArmSpeed(speed);
			break;
		case '+':
			if(speed<1.2) speed+=0.1f;
			setDefaultArmSpeed(speed);
			break;
		case 'a':
			MoveRelative(-0.001f, 0, 0, 0, 0, 0);
			first_x = T07Cmd(0,3);
			break;
		case 'd':
			MoveRelative(0.001f, 0, 0, 0, 0, 0);
			first_x = T07Cmd(0,3);
			break;
		case 'w':
			MoveRelative(0, 0.001f, 0, 0, 0, 0);
			first_y = T07Cmd(1,3);
			break;
		case 's':
			MoveRelative(0, -0.001f, 0, 0, 0, 0);
			first_y = T07Cmd(1,3);
			break;
		case 'r':
			MoveRelative(0, 0, 0.01f, 0, 0, 0);
			first_z = T07Cmd(2,3);
			break;
		case 'f':
			MoveRelative(0, 0, -0.01f, 0, 0, 0);
			first_z = T07Cmd(2,3);
			break;
		case 'z':
			// via point
			GoToPoint(via_X,via_Y,via_Z,0,0,0);
			// first pen
			GoToPoint(first_x,first_y,first_z,0,0,0);
			break;
		case 'j':
			MoveRelative(-0.08f, 0, 0,0,0,0);
			break;
		case 'l':
			MoveRelative(0.08f, 0, 0,0,0,0);
			break;
		case 'i':
			MoveRelative(0, 0.11f, 0,0,0,0);
			break;
		case 'k':
			MoveRelative(0, -0.11f, 0,0,0,0);
			break;
		case 'b':
			GraspDropPen(0.12f,1);
			break;
		case 'n':
			finger.move(80);
			break;
		case 'o':
			outputFile.open("trace.txt");
			write=true;
			MoveRelative(0.1f, 0, 0, 0, 0, 0);
			write=false;
			outputFile.close();
			break;
		case 'm':
			Move_Contour();
			break;
		case '1':
			GoToPoint(0.7f,0,-0.05f,0,0,0);
			break;
		case '2':
			T <<  -cos(-10.0f/18.0f*pi_f),  sin(-9.0f/18.0f*pi_f),  -sin(-1.0f/18.0f*pi_f),  0.7f,
			sin(-9.0f/18.0f*pi_f),  cos(-9.0f/18.0f*pi_f),  0.0f,  0.0f,
			sin(-1.0f/18.0f*pi_f),  0.0f, -cos(-1.0f/18.0f*pi_f),  -0.05f,
			0.0f,  0.0f,  0.0f,  1.0f;
			Move_L_Abs(T,0);
			while(MOVL){system("cls");	DisplayLoop(speed);}
			break;
		case '3':
			MoveRelative(0, 0, -0.12f, 0, 0, 0);
			break;
		case '4':
			GoToPoint(x_axis_color,centerY,z_axis_color,0,-10,0);
			break;
		case '5':
			GoToPoint(x_axis_color,centerY,z_axis_color,0,-10,90);
			break;
		case '6':
			MoveRelative(0, 0, 0, 0, 0, -1.0f/18.0f*pi_f);
			break;
		
	}
	kbCmd_3 = kbCmd;
	kbCmd = ' ';
}
void MyLoop_keyboard(){
	float tempX;
	float tempY;
	float tempZ;
	switch( kbCmd ){
		case '1':
			MoveRelative(0, 0, 0, 0, 0.05f*pi_f, 0);
			break;
		case '2':
			MoveRelative(0, 0, 0, 0, -0.05f*pi_f, 0);
			break;
		case 'c':
			sketchIndex = 0;
			fillIndex = 0;
			break;
		case 'v':
			fillIndex = 0;
			break;
		case '-':
			if(speed>0.2) speed -= 0.1f;
			setDefaultArmSpeed(speed);
			break;
		case '+':
			if(speed<1.2) speed += 0.1f;
			setDefaultArmSpeed(speed);
			break;
		case 'w':
			MoveRelative(0, 0.05f, 0, 0, 0, 0);
			break;
		case 's':
			MoveRelative(0, -0.05f, 0, 0, 0, 0);
			break;
		case 'a':
			MoveRelative(-0.05f*cos(boardAngle), 0, -0.05f*sin(boardAngle), 0, 0, 0);
			break;
		case 'd':
			MoveRelative(0.05f*cos(boardAngle), 0, 0.05f*sin(boardAngle), 0, 0, 0);
			break;
		case 'r':
			MoveRelative(-0.001f*sin(boardAngle), 0, 0.001f*cos(boardAngle), 0, 0, 0);
			x_axis_color = T07Cmd(0,3);
			z_axis_color = T07Cmd(2,3);
			break;
		case 'f':
			MoveRelative(0.001f*sin(boardAngle), 0, -0.001f*cos(boardAngle), 0, 0, 0);
			x_axis_color = T07Cmd(0,3);
			z_axis_color = T07Cmd(2,3);
			break;
		case 't':
			MoveRelative(-0.001f*sin(boardAngle), 0, 0.001f*cos(boardAngle), 0, 0, 0);
			x_axis_black = T07Cmd(0,3);
			z_axis_black = T07Cmd(2,3);
			break;
		case 'g':
			MoveRelative(0.001f*sin(boardAngle), 0, -0.001f*cos(boardAngle), 0, 0, 0);
			x_axis_black = T07Cmd(0,3);
			z_axis_black = T07Cmd(2,3);
			break;
		case 'y':
			GoToPoint(x_axis_color-displace*sin(boardAngle),centerY,z_axis_color+displace*cos(boardAngle),0,-10,wrist_rotate);
			break;
		case 'h':
			/*cin>>tuneAngle;
			boardAngle = tuneAngle/18.0f*pi_f;*/
			cin>>blackDrop;
			break;
		case 'u': // tune short pen
			x_axis_black = x_axis_color;
			z_axis_black = z_axis_color;
			// go to via point
			GoToPoint(via_X,via_Y,via_Z,0,0,0);

			// Go to position of pen
			GoToPoint(first_x,first_y,first_z,0,0,0);

			// Grasp pen
			GraspDropPen(blackDrop,1);

			// go to via point
			GoToPoint(via_X,via_Y,via_Z,0,0,0);
			
			// Go to center
			GoToPoint(x_axis_color-displace*sin(boardAngle),centerY,z_axis_color+displace*cos(boardAngle),0,-10,wrist_rotate);
			break;
		case 'o':
			if(sketchIndex < sketchNumbers){
				//draw edge
				// To the top of first point
				tempX = -(sketchPositionXY[sketchIndex][0](0))*cos(boardAngle)+x_axis_black-displace*sin(boardAngle);
				tempY = sketchPositionXY[sketchIndex][0](1);
				tempZ = -(sketchPositionXY[sketchIndex][0](0))*sin(boardAngle)+z_axis_black+displace*cos(boardAngle);
				GoToPoint(tempX,tempY,tempZ,0,-10,wrist_rotate);
				
				//Move_Contour();
				for(unsigned int i=0;i<sketchPositionXY[sketchIndex].size();i++){
					tempX = -(sketchPositionXY[sketchIndex][i](0))*cos(boardAngle)+x_axis_black;
					tempY = sketchPositionXY[sketchIndex][i](1);
					tempZ = -(sketchPositionXY[sketchIndex][i](0))*sin(boardAngle)+z_axis_black;
					GoToPoint(tempX,tempY,tempZ,0,-10,wrist_rotate);
				}

				tempX = -(sketchPositionXY[sketchIndex][sketchPositionXY[sketchIndex].size()-1](0))*cos(boardAngle)+x_axis_black-displace*sin(boardAngle);
				tempY = sketchPositionXY[sketchIndex][sketchPositionXY[sketchIndex].size()-1](1);
				tempZ = -(sketchPositionXY[sketchIndex][sketchPositionXY[sketchIndex].size()-1](0))*sin(boardAngle)+z_axis_black+displace*cos(boardAngle);
				GoToPoint(tempX,tempY,tempZ,0,-10,wrist_rotate);
				sketchIndex++;
			}
			break;
		case 'p':
			// fill every areas
			if(fillIndex < fillNumbers){
				Vector2f dxy = ColorPosition(fillIndex+1);

				// go to via point
				GoToPoint(via_X,via_Y,via_Z,0,0,0);

				// Go to position of pen
				GoToPoint(first_x+dxy(0),first_y+dxy(1),first_z,0,0,0);

				// Grasp pen
				GraspDropPen(0.12f,1);

				// go to via point
				GoToPoint(via_X,via_Y,via_Z,0,0,0);

				// Go to center
				GoToPoint(x_axis_color-displace*sin(boardAngle),centerY,z_axis_color+displace*cos(boardAngle),0,-10,wrist_rotate);

				//// To the top of first point
				//float x = (fill_pos_x[fillIndex][0][0])*cos(boardAngle)+x_axis_color-0.01;
				//float y = fill_pos_y[fillIndex][0][0];
				//float z = -(fill_pos_x[fillIndex][0][0])*sin(boardAngle)+z_axis_color;
				//GoToPoint(tempX,tempY,tempZ,0,-10,wrist_rotate);

				for(unsigned int i=0;i<fill_pos_x[fillIndex].size();i++){
					// move to the top of first point of every area
					tempX = -(fill_pos_x[fillIndex][i][0])*cos(boardAngle)+x_axis_color-displace*sin(boardAngle);
				    tempY = fill_pos_y[fillIndex][i][0];
					tempZ = -(fill_pos_x[fillIndex][i][0])*sin(boardAngle)+z_axis_color+displace*cos(boardAngle);
					GoToPoint(tempX,tempY,tempZ,0,-10,wrist_rotate);

					for(unsigned int j=0;j<fill_pos_x[fillIndex][i].size();j++){ 
						tempX = -(fill_pos_x[fillIndex][i][j])*cos(boardAngle)+x_axis_color;
						tempY = fill_pos_y[fillIndex][i][j];
						tempZ = -(fill_pos_x[fillIndex][i][j])*sin(boardAngle)+z_axis_color;
						GoToPoint(tempX,tempY,tempZ,0,-10,wrist_rotate);
					}
				}
				tempX = -(fill_pos_x[fillIndex][fill_pos_x[fillIndex].size()-1][1])*cos(boardAngle)+x_axis_color-displace*sin(boardAngle);
				tempY = fill_pos_y[fillIndex][fill_pos_x[fillIndex].size()-1][1];
				tempZ = -(fill_pos_x[fillIndex][fill_pos_x[fillIndex].size()-1][1])*sin(boardAngle)+z_axis_color+displace*cos(boardAngle);
				GoToPoint(tempX,tempY,tempZ,0,-10,wrist_rotate);
				fillIndex++;

				// Move to pen box position
				// go to via point
				GoToPoint(via_X,via_Y,via_Z,0,0,0);
				
				// Drop pen
				GraspDropPen(0.11f,0);
			}
			break;
		case 'b':
			finger.close();
			break;
		case 'n':
			finger.move(80);
			break;
		

		case '3':
			GoToPoint(x_axis_color,centerY,z_axis_color,0,0,0);
			break;
		case '4':
			T <<  -cos(-10.0f/18.0f*pi_f),  sin(-9.0f/18.0f*pi_f),  -sin(-1.0f/18.0f*pi_f),  x_axis_color,
			sin(-9.0f/18.0f*pi_f),  cos(-9.0f/18.0f*pi_f),  0.0f,  centerY,
			sin(-1.0f/18.0f*pi_f),  0.0f, -cos(-1.0f/18.0f*pi_f),  z_axis_color,
			0.0f,  0.0f,  0.0f,  1.0f;
			Move_L_Abs(T,0);
			while(MOVL){system("cls");	DisplayLoop(speed);}
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
			float y = (-1)*(stoi(sep[0])-200)*(float)(paperSize/imageSize)/100;//+centerX;
			float x = (stoi(sep[1])-200)*(float)(paperSize/imageSize)/100;//+centerY;
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
			float y1 = (-1)*(stoi(sep[0])-200)*(float)(paperSize/imageSize)/100;//+centerX;
			float x1 = (stoi(sep[1])-200)*(float)(paperSize/imageSize)/100;//+centerY;
			float y2 = (-1)*(stoi(sep[2])-200)*(float)(paperSize/imageSize)/100;//+centerX;
			float x2 = (stoi(sep[3])-200)*(float)(paperSize/imageSize)/100;//+centerY;
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
    liPeriod_1ms.QuadPart  = 10000;
	Init_IMPCard();

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
	setDefaultArmSpeed(speed);
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
		cout<< "x_axis_color: "<< x_axis_color<<" z_axis_color: "<< z_axis_color<<endl;
		cout<< "z_axis_black: "<< x_axis_black<<" z_axis_black: "<< z_axis_black<<endl;
		cout<<"Board Angle: "<<tuneAngle*10<<endl;
	}
	cout<< endl << " Initialization is done . " <<endl;
	Sleep(1000);

	
	// via point
	GoToPoint(via_X,via_Y,via_Z,0,0,0);
	// go to board
	GoToPoint(x_axis_color-displace*sin(boardAngle),centerY,z_axis_color+displace*cos(boardAngle),0,-10,wrist_rotate);
	

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
		cout<<endl;
		cout<< "x_axis_color: "<< x_axis_color<<" z_axis_color: "<< z_axis_color<<endl;
		cout<< "z_axis_black: "<< x_axis_black<<" z_axis_black: "<< z_axis_black<<endl;
		cout<<"Board Angle: "<<tuneAngle*10<<endl;
		cout<<"Black drop distance: "<<blackDrop<<endl;
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