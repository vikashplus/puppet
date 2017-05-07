/****************************************************************************
Copyright (c) 2011 Yuichi Katori All Rights Reserved
License: Gnu Public license (GPL) v3
Author: Yuichi Katori (yuichi.katori@gmail.com)
Project:MATPLOT++ (MATLAB-like plotting tool in C++).
Version:0.3.13
****************************************************************************/

#ifndef _MATPLOTPP_H_
#define _MATPLOTPP_H_

#include <stdlib.h>
#include <vector>
#include <deque>
#include <string>
#include <valarray>
#include <iostream>
#include <math.h>
#include <time.h>
#ifdef _WIN32
	#include <Windows.h>

	//#define DLL_EXPORT

	#ifdef DLL_EXPORT
		#define DECLDIR __declspec(dllexport)
	#else
		#define DECLDIR __declspec(dllimport)
	#endif
#else
	#include "unistd.h"
#endif

#ifndef PI
#define PI 3.14159265358979323846264
#endif

#define fmax max
#define fmin min
#define min_val 1e-17
using namespace std;

typedef vector<int> ivec;
typedef vector<double> dvec;
typedef vector< vector<double> > dmat;
typedef vector< vector<float> > tcvec;
typedef vector< vector< vector<float> > > tcmat;

inline vector<double> linspace(double min,double max,int n){
	vector<double> a;
	if(n<1){n=1;}
	a.resize(n);
	for(int i=0;i<n;++i){a[i]=min+(max-min)*i/(n-1);}
	return a;
};

inline valarray<double> valinspace(double min,double max,int n){
	valarray<double> a; 
	a.resize(n);
	for(int i=0;i<n;++i){a[i]=min+(max-min)*i/(n-1);}
	return a;
};

inline void mpSleep(int time_in_MilliSeconds)
{
	#ifdef WIN32
		Sleep((DWORD)time_in_MilliSeconds);
	#elif __linux__
		usleep((int)time_in_MilliSeconds*1000);
	#endif
};

int mpCreateWindow(int left,int top,int width,int height,char c[]);
int mpCreateWindow(int left,int top,int width,int height);

class Figure{///
public:
	int id;
	//int Status;// 0:minimized, 1:default position, 2:maximized 
	int Position[4];//left top width height
	int Visible;
	vector<int> Children;

	void add_child(int i);
	Figure(int id_){
		id=id_;
		//Status=1;
		//Position[0]=100;
		//Position[1]=100;
		//Position[2]=800;
		//Position[3]=800;
		Visible=1;
	};
};


class Layer{///
public:
	int id;
	int Visible;
	string layername;
	string BackgroundColor;			// background canvas color
	vector<int> Children;
	Layer(int id_);
	void add_child(int i);
};

class Axes{///
private:

protected:

public:
	int id;
	int subplot_id;			// LLPP:: LL-layerID, PP subplot location/ id
	float cta,phi;			// controled by mouse
	float cta0,phi0;		// default value or specified by command line
	double xmin,xmax,ymin,ymax,zmin,zmax; // range of data plotted within the axis
	int num_child;
	// Mouse 
	double XMouse,YMouse;	// mouse click points in axes frame
	bool Mouse;				// has mouse captured/ edited axis ever


	void reset();
	void config();
	int ID();

	int selected();
	void selected(int i);
	void add_child(int i);    
	dvec make_tick(double min,double max, vector<string> &tickLabel);

	int View;// 0:2D, 1:3D 2:colorbar

	vector<vector<float> > ColorMap;// for colorbar

	// Matlab variables //
	// styles
	int Box;//0:Off, 1:On
	string GridLineStyle;
	float LineWidth;
	string TickDir;// {in} | out
	//string TickDirMode;
	//TickLength
	int Visible;//0:Off, 1:On
	int XGrid,YGrid,ZGrid;// {0:Off}, 1:On

	// General Information 
	int Parent;
	vector<int> Children;
	int Selected;			// is axis selected
	int iChildSelected;		// Child selcted (-1:none)

	// location
	float Position[4];		//location
	float Viewport3d[4];	//location
	string XAxisLocation;	//left bottom width height
	string YAxisLocation;	//left bottom width height
	string LegendLocation;  //left right
	// Axes properties
	double XLim[2],YLim[2],ZLim[2];	// Axis range
	int XLimMode,YLimMode,ZLimMode;	// 0:Auto 1:Manual
	int XScale,YScale,ZScale;		// 0:linear | 1:log
	string XDir,YDir,ZDir;			// direction
	float axisPadding;				// padding for selection and zoom
	string Color;					// for ticks and lables

	// Ticks
	dvec XTick,YTick,ZTick;
	string XTickMode,YTickMode,ZTickMode;				//[{auto}|manual]
	vector<string> XTickLabel, YTickLabel, ZTickLabel;	// Tick lables
	int TickLabel;										// 0:Off, {1:On}

	// View
	float CameraPosition[3];
	float CameraTarget[3];
	float CameraUpVector[3];
	string BackgroundColor;			// background canvas color
	// Label
	string Title;
	string XLabel,YLabel,ZLabel;

	double CLim[2];

	Axes(int id_)
	{
		id=id_;
		Selected=0;
		iChildSelected =-1;
		Position[0]=(float)0.13;
		Position[1]=(float)0.11;
		Position[2]=(float)0.775;
		Position[3]=(float)0.815;

		Viewport3d[0]=0.0;
		Viewport3d[1]=0.0;
		Viewport3d[2]=1.0;
		Viewport3d[3]=1.0;

		XMouse=0;
		YMouse=0;
		Mouse=0;
		View=0;
		Visible=1;
		Box=1;
		Children.clear();

		cta0=30; 
		phi0=30;
		cta=cta0; 
		phi=cta0;

		CameraPosition[0]=1; CameraPosition[1]=1; CameraPosition[2]=1;
		CameraTarget[0]=0.;  CameraTarget[1]=0; CameraTarget[2]=0;
		CameraUpVector[0]=0; CameraUpVector[1]=0; CameraUpVector[2]=1;

		LineWidth=1.5;

		GridLineStyle=":";
		XGrid=0;
		YGrid=0;
		ZGrid=0;

		XLim[0]=0;    XLim[1]=10;
		YLim[0]=0;    YLim[1]=10;
		ZLim[0]=0;    ZLim[1]=10;

		LegendLocation="left";
		XLimMode=0; YLimMode=0; ZLimMode=0;
		XAxisLocation="bottom";//top | bottom
		YAxisLocation="left";// left | right
		XScale=0;// {0:linear} | 1:log
		YScale=0;
		ZScale=0;
		axisPadding =.025f;
		Color = "k";

		TickLabel=1;
		TickDir="in";
		XTickMode = "auto";
		YTickMode = "auto";	
		ZTickMode = "auto";	
		Title="Axes";
		Title +=to_string(id_/100);
		BackgroundColor ="w";

		xmin= 1e99;    xmax=-1e99;
		ymin= 1e99;    ymax=-1e99;
		zmin= 1e99;    zmax=-1e99;

		CLim[0]=0;CLim[1]=0;

		num_child=0;
		//MakeTick();
		//Parent=i_figure;
	};
};


class Line{///
public:
	int id;
	int Errorbar;

	void reset();
	void color(float r,float g,float b);

	// Matlab oriented variables //

	dvec XData,YData,ZData;
	dvec YPData,YMData;
	//dmat XData,YData,ZData;
	//dmat EData,UData,LData;
	//dmat VData,WData;

	string legend;
	string Color;
	string LineStyle;// {-} | -- | : | -. | none
	float  LineWidth;
	string Marker;// {none}
	float  MarkerSize;
	string MarkerEdgeColor;
	string MarkerFaceColor;

	int Clipping;
	//string EraseMode;
	int SelectionHighlight;
	int Visible;

	// General Information 
	int Parent;
	int Selected;

	Line(int id_){	
		id=id_;

		legend="";
		Color="b";
		LineStyle="-";
		LineWidth=1.0;

		Marker="none";
		MarkerSize=1.0;
		MarkerEdgeColor="k";
		MarkerFaceColor="w";

		Errorbar=0;
		//PlotStyle=0;
	}   
};
class Surface{///
public:
	int type;
	int id;
	string ColorMap;

	//dvec XData,YData;
	dmat XData,YData,ZData,CDataIndex;
	dvec V,UserData;
	tcmat CData;

	string FaceColor;//ColorSpec    | none | {flat} 
	string EdgeColor;//ColorSpec{k} | none | flat

	string LineStyle;// {-} | -- | : | -. | none
	float  LineWidth;
	string Marker;// {none}
	float  MarkerSize;
	string MarkerEdgeColor;
	string MarkerFaceColor;

	int Parent;

	int NContour;

	Surface(int id_){
		id=id_;

		ColorMap="Gray";
		//Shading="faceted";
		FaceColor="flat"; 
		EdgeColor="b";
		LineStyle="-";
		LineWidth=0.5;
		NContour=10;
		V.clear();

	}
	void get(){
		cout <<"FaceColor: "<< FaceColor <<endl;
		cout <<"EdgeColor: "<< EdgeColor <<endl;
		cout <<"LineStyle: "<< LineStyle <<endl;
		cout <<"LineWidth: "<< LineWidth <<endl;
	}
};
//Note: 
// [m,n] = size(Z), length(x) = n length(y) = m, (x(j),y(i),Z(i,j))

class Patch{///
public:
	int id;
	int type;						// 0:2D-patch , 1:3D-patch

	vector< vector<int> > Faces;	// dono where being used 
	dmat Vertices;					// dono where being used 
	dmat FaceVertexCData;			// dono where being used 
	dmat XData,YData,ZData;			// Patch data
	//dvec ICVec;
	//dmat ICMat;    
	//tcmat CData;
	tcvec CData;			// color data
	int iFaceSelected;		// Selected face
	string EdgeColor,FaceColor;//{ColorSpec}|none|flat|interp 

	string LineStyle; // {-} | -- | : | -. | none
	float  LineWidth;

	Patch(int id_)
	{
		id=id_;
		type=0;
		iFaceSelected =-1;
		LineWidth=1;
		FaceColor="r"; 
		EdgeColor="k";
		LineStyle="-";
	}
};
//Note: XData[iv][if]

class Text{///  
public:
	int id;
	string String;
	float Position[3];
	int Parent;
	int type;				// 0:axis 1:figure
	bool direction;			// 0:Horizontal 1:vertical
	int fontSize;			// Text size
	float fontHeight;
	float fontWidth;
	void* font;				// Text fonts
	string TextColor;		// Text color
	int textPatchID;
	Text(int id_);
};



const int tFigure=1;
const int tAxes=2;
const int tLine=3;
const int tSurface=4;
const int tText=5;
const int tLayer=6;
const int tPatch=7;

/// contour
struct ContourPoint{
	double x,y;
	int xj,yi;
	int xy;
	int done;
};
dmat contourc(dvec x, dvec y, dmat Z, dvec v);

//#ifdef DLL_EXPORT
//class DECLDIR MatPlot
//#else
//class MatPlot 
//#endif
class MatPlot 
{
private:
	int is_debug1;
	int is_debug2;

	vector<vector<float> > cmap;//TODO move to class Figure

	int mode;//0:initialization 1:configuration
	int init_level;// initialization level of objects
	int hObj;// handle number of current object

	int time_layer_clicked,time_layer_clicked_last;

	// Events //
	int window_w,window_h;
	int xButtonDown,yButtonDown;// last clicked mouse position
	float ctaButtonDown,phiButtonDown;
	int xPassive,yPassive;
	typedef enum _activeButtonDown_
	{LEFT=-1, NONE=0, RIGHT=1}_activeButtonDown; // -1:left, 0:none, 1:right
	_activeButtonDown activeButtonDown;
	// pointer to current objects //
	Figure *cf;
	Layer *cfr;
	Axes *ca;
	Line *cl;
	Surface *cs;
	Patch *cp;
	Text *ct;    

	// objects containers //
	vector< Figure > vFigure;
	vector< Layer > vLayer;
	vector< Axes > vAxes; 
	vector< Line > vLine;
	vector< Surface > vSurface;
	vector< Patch > vPatch;
	vector< Text > vText;

	// objects counter //
	int iFigure;
	int iLayer;
	int iAxes;
	int iLine;
	int iSurface;
	int iPatch;
	int iText;

	// Selected object //
	int iAxesSelected;

	// coordinate transform  //
	// figure coordination
	double ctx2(double x);
	double cty2(double y);
	// axes coordination
	double ctx(double x);
	double cty(double y);
	double ct3x(double x);
	double ct3y(double y);
	double ct3z(double z);

	int figure();

	// display //
	void display_figure();    
	void display_layer();
	void display_layer2();

	void display_axes();
	void display_axes_2d();
	void display_axes_3d();
	void display_axes_colorbar();

	void display_line();

	void display_surface();
	void display_surface_2d();
	void display_surface_3d();
	void display_pcolor();
	void display_contour();

	void display_patch();
	void display_patch_2d();
	void display_patch_3d();


	void display_bar();

	void display_text();

	// mouse //
	void Layer_mouse(int button, int state, int x, int y );
	void Axes_mouse(int button, int state, int x, int y );
	void patch_mouse(int button, int state, int x, int y );
	void Axes_zoom(int button, int dir, int x, int y );
	void Axes_motion(int x, int y);
	void Slider_update(int x, int y);

	void surface_config();
	void line_config();
	void patch_config();
	tcvec Index2TrueColor(dvec IC);

public:

	MatPlot();
	~MatPlot();

	virtual void DISPLAY(){};

	void inline debug1(){is_debug1=1;}
	void inline debug2(){is_debug2=1;}

	// GLUT Callback Functions ///
	void display();
	void reshape(int w, int h);
	void mouse(int button, int state, int x, int y );
	void mouseWheel(int button, int dir, int x, int y );
	void motion(int x, int y );
	void passivemotion(int x,int y);
	void keyboard(unsigned char key, int x, int y);
	vector< string > defaultColors;


	// Layer ///
	int layer();
	int layer(string s);    
	int layer(string s,int Visible);
	int get_layer(string layer_name);
	void set_layer(int layer_id);

	// Axes ///	
	int axes();
	int gca();
	// Make axes with id==h active
	bool gca(int h);
	int subplot(int m,int n,int p);
	int subplot(int m,int n,int p, string s);

	int colorbar();

	void axis(double xMin,double xMax,double yMin,double yMax);
	void axis(double xMin,double xMax,double yMin,double yMax,double zMin,double zMax);

	void axis(string s);
	void axis(int s);

	// Grid lines (INPUTS:: 0/1/on/off/x/y/z)
	void grid(string s);
	// Grid lines (INPUTS:: 0/1)
	void grid(int s);

	void ticklabel(string s);
	void ticklabel(int s);

	void title(string s);
	void xlabel(string s);
	void ylabel(string s);
	void zlabel(string s);
	void xtick(dvec xticks_pos, vector<string> xticks);
	void xtick(dvec xticks_pos);
	void ytick(dvec yticks_pos, vector<string> yticks);
	void ytick(dvec yticks_pos);
	void ztick(dvec zticks_pos, vector<string> zticks);
	void ztick(dvec zticks_pos);
	void xlim(double min,double max);
	void ylim(double min,double max);
	void legend(vector<string> legends);
	void mouse_capture(double xmouse,double ymouse);   

	// set, General Object Handling ============
	// LineStyle: | {-} | -. | -- | :
	// colors:: |k|r|g|b|c|m|y|k|dr|dg|db|dc|dm|dy|
	//			|lr|lb|lg|lc|lm|ly|ur|ubr|ub|ug|uy|uc|up|uo|um|
	// COLOR(= Color & MarkerEdgeColor)
	// Axes-------
	//		XLabel/ YLabel/ ZLabel
	//		TickDir: | {in} | out |
	//		TickColor
	//		YAxisLocation/	XAxisLocation: | left | bottom | width | height |
	//		LegendLocation: | left | right |
	//		BackgroundColor
	//		XScale/ YScale/ ZScale: | {linear} | log |
	// Line-------
	//		Color: 
	//		Marker: |.|+|x|#|^|v|0|*|s|<|>|
	//		LineStyle
	//		MarkerEdgeColor
	//		MarkerFaceColor
	//		COLOR
	//		LineWidth
	//		MarkerSize
	//		Legend
	// Surface----
	//		COLOR
	//		LineStyle
	//		EdgeColor
	//		FaceColor
	//		LineWidth
	// Patch------
	//		COLOR
	//		LineStyle
	//		EdgeColor
	//		FaceColor
	//		LineWidth
	// Text-------
	//		TextColor
	//		COLOR
	//		LineStyle
	//		EdgeColor
	//		FaceColor
	//		LineWidth
	// Layer------
	//		BackgroundColor
	//

	void set(string v);
	void set(int h, string v);
	void set(float v);  
	void set(string p,float v);
	void set(string p,string v);
	void set(string p, float r, float g, float b);
	void set(int h,string p,string v);
	void set(int h,string p,float v);      
	void set(int h,string p, float r, float g, float b);
	int gco();
	// get
	dvec get(string p);
	dvec get(int h, string p);

	// Line ///

	int begin();//do not use
	void end();//do not use
	void vertex(double x,double y);
	void vertex(double x,double y,double z);

	int line();
	int line(const dvec y);
	int line(const dvec x, const dvec y);
	int line(const dvec x, const dvec y, const dvec z);
	int line(const double* y, const int n);
	int line(const double* x, const double* y, const int n);
	int line(const double* x, const double* y, const double* z, const int n);
	
	// set set(...) for style details
	int plot(dvec y);
	int plot(dvec x, dvec y);
	int plot(dvec y, float width, string style); 
	int plot(dvec x, dvec y, float width, string style);
	int plot(const double* y, const int n);
	int plot(const double* x, const double* y, const int n);
	int plot(const double* y, const int n, const float width, const string style);
	int plot(const double* x, const double* y, const int n, const float width, const string style);
	ivec plot(dmat Y, const float width=2.0);
	ivec plot(dvec x,dmat Y, const float width=2.0);
	ivec plot(dmat X,dmat Y, const float width=2.0);
	int plot(valarray<double> x,valarray<double> y);
	// plot with two Yaxis. 
	// Output:: plot handles
	//		handle[0]: axis0 
	//		handle[1]: axis1 
	//		handle[2]: plot0 
	//		handle[3]: plot1 
	ivec plotyy(dvec x, dvec y0, dvec y1);
	ivec plotyy(dvec x, dvec y0, dvec y1, float width1, float width2, string style1="b-", string style2="r-");
	ivec plotyy(const double* x, const double* y0, const double* y1, const int n);
	ivec plotyy(const double* x, const double* y0, const double* y1, const int n, const float width0, const float width1, const string style0, const string style1);

	int plot3(dvec x,dvec y,dvec z);
	//int plot3(dvec X,dvec Y,dvec Z);

	int semilogx(dvec x,dvec y);
	int semilogy(dvec x,dvec y);
	//int loglog(dvec y);
	int loglog(dvec x,dvec y);    

	//int polar(dvec theta,dvec rho);

	void vertex(double x,double y,double ep,double em);
	int errorbar(dvec x,dvec y,dvec e);
	int errorbar(dvec x,dvec y,dvec ep,dvec em);

	//int quiver(U,V);
	//int quiver(X,Y,U,V);

	//int scatter(X,Y,S,C)
	//int scatter(X,Y,S)
	//int scatter(X,Y)

	// Surface, Contour ///
	dmat peaks(int n);
	//dmat peaks(int m,int n);
	//dmat peaks(int m,int n,string type);

	int surface();
	int surface(dmat Z);
	int surface(dmat Z,dmat C);
	int surface(dmat Z,tcmat C); //!!   
	int surface(dvec x,dvec y,dmat Z);
	int surface(dvec x,dvec y,dmat Z,dmat C);
	int surface(dvec x,dvec y,dmat Z,tcmat C);//!!
	int surface(dmat X,dmat Y,dmat Z);
	int surface(dmat X,dmat Y,dmat Z,dmat C);
	int surface(dmat X,dmat Y,dmat Z,tcmat C);//!!

	int pcolor(dmat C);
	int pcolor(tcmat C);
	int pcolor(dvec x,dvec y,dmat C);
	int pcolor(dvec x,dvec y,tcmat C);
	int pcolor(dmat X,dmat Y,dmat C);
	int pcolor(dmat X,dmat Y,tcmat C);

	int contour(dmat Z);
	int contour(dmat Z,int n);
	int contour(dmat Z,dvec v);
	int contour(dvec x, dvec y, dmat Z);
	int contour(dvec x, dvec y, dmat Z,int n);
	int contour(dvec x, dvec y, dmat Z,dvec v);
	//int contour(dmat X, dmat Y, dmat Z);
	//int contour(dmat X, dmat Y, dmat Z,int n);
	//int contour(dmat X, dmat Y, dmat Z,dvec v);

	//int mesh(dmat Z);
	//int mesh(dmat Z,dmat C);
	//int mesh(dmat Z,tcmat C);    
	int mesh(dvec x, dvec y, dmat Z);
	//int mesh(dvec x, dvec y, dmat Z,dmat C);
	//int mesh(dvec x, dvec y, dmat Z,tcmat C);    
	//int mesh(dmat X,dmat Y,dmat Z);
	//int mesh(dmat X,dmat Y,dmat Z,dmat C);
	//int mesh(dmat X,dmat Y,dmat Z,tcmat C);
	// meshc()
	// meshz()

	int surf(dvec x, dvec y, dmat Z);

	// Patch ///

	int patch();
	int patch(dmat X,dmat Y);
	int patch(dmat X,dmat Y,dvec C);
	int patch(dmat X,dmat Y,tcvec C);    
	int patch(dmat X,dmat Y,dmat Z);//!!
	int patch(dmat X,dmat Y,dmat Z,dvec C);//!!    
	int patch(dmat X,dmat Y,dmat Z,tcvec C);//!!
	//int patch(dmat X,dmat Y,tcmat C);
	//int patch(dmat X,dmat Y,dmat Z,tcmat C);

	int bar(dvec y);
	int bar(dvec y, float width);
	int bar(dvec x,dvec y);
	int bar(dvec x,dvec y, float width);
	int bar(dvec x,dvec y, int start, int end, float width);

	//int bar(Y)
	//int bar(Y,float width);
	//int bar(Y,string style);
	//int bar(Y,float width,string style);

	//int bar(x,Y)
	//int bar(x,Y,float width);
	//int bar(x,Y,string style);
	//int bar(x,Y,float width,string style);

	// Slider ( Note: (1) Always plot first (2) Initialize buffer with slider's intial value)
	int slider(dvec x, dvec &initialized_update_buffer,float width);
	int slider(dvec x, dvec &initialized_update_buffer,float width, vector<string> sliderlabel);
	int slider(dvec x, dvec &initialized_update_buffer, dvec lower_limit, dvec upper_limit, float width);
	int slider(dvec x, dvec &initialized_update_buffer, dvec lower_limit, dvec upper_limit, float width, vector<string> sliderLabel);

	//int hist(y);
	//int hist(y,x);
	//int hist(y,nbins);

	//int pie(dvec x);
	//int pie(dvec x, vector<int> Explode);

	// Text ///
	//TODO: more fonts    
	int text();
	// Write text on an axis
	int text(double x,double y,string s, bool isVertical=false, string TextColor="k", 
		string FaceColor="none", string EdgeColor="none", string LineStyle="-"); //dir 0:Hor 1:Vert
	void set_font(char font_[],int size);
	void ptext(float x,float y,string s, void* font=NULL, string Color="k", bool dir=0); //dir::0:hor, 1:vert
	void ptext3(float x,float y,float z,string s);
	void ptext3c(float x,float y,float z,string s);

	// Colors ///
	void color(float r,float g,float b);
	vector<float> colormap(string c,float t);
	void colormap(string c);
	void colormap(vector<vector<float> > c);

	void gray();
	void jet();
	void hsv();
	void hot();
	void cool();
	void spring();
	void summer();
	void autumn();
	void winter();

	vector<float> map2color(double x,double xmin,double xmax);

	void Shading(string c);
	void shading(string c);
	vector<float> ColorSpec2RGB(string c);
	string rgb2colorspec(vector<float> rgb);

	// print //
	void print();
	//write to a file
	void f_Write(FILE* fp, dvec data, int n, bool newline_at_end);

};


/*#ifdef DLL_EXPORT
class DECLDIR Plot :public MatPlot
#else
class Plot :public MatPlot
#endif*/
class Plot :public MatPlot
{	
public:
	virtual void DISPLAY();

	//Examples
	void test();
	void creating_plot();
	void multiple_plots();
	void style_n_color();
	void multiple_axes();
	void Surface_n_Contours();
	void line_n_surface_3d();
	void create_slider();
	void animation();
	void advanced();
};

//=============================================================================  
// Function
//=============================================================================

// Close the graphs
void Graphics_Close(void);

// Initialize graphics using default parameters
void Graphics_init(int argc, char* argv[]);

// Initialize graphics
void Graphics_init(int argc, char* argv[], const int win_Xpos, 
				   const int  win_Ypos, const int win_Width, const int win_Height, 
				   const char* win_Title );

// Is the plaotting window active?
bool Graphics_Is_Running();

// Returns (clear if requested) the last key captured by the graphics window. '\0' for no key
unsigned char Graphics_Key(bool clear_after_return=true);

#endif /*_MATPLOTPP_H_*/