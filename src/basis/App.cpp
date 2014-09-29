#define _USE_MATH_DEFINES

#include "App.hpp"
#include "base/Main.hpp"
#include "gpu/GLContext.hpp"
#include "io/File.hpp"
#include "io/StateDump.hpp"
#include "base/Random.hpp"
#include "gui/Image.hpp"

#include "extra.h"
#include "surf.h"
#include "parse.h"
#include "Subdiv.hpp"

#include <stdio.h>
#include <conio.h>

#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include <GL/GLU.h>

using namespace FW;
using namespace std;

//------------------------------------------------------------------------

App::App(void)
:   action_				(Action_None),
    common_ctrl_		(CommonControls::Feature_Default & ~CommonControls::Feature_RepaintOnF5),
    cullmode_			(CullMode_None),
    mouse_pressed_		(false),
    curvemode_			(true),
	curvenormalmode_	(false),
    surfacemode_		(true),
    pointmode_			(true),
	subdivisionmode_	(false),
	wireframe_			(false),
    normal_length_		(0.1f),
	current_subdivision_level_(0)
{
    common_ctrl_.showFPS(true);

    common_ctrl_.addButton((S32*)&action_, Action_LoadSWP,					FW_KEY_L,       "Load SWP... (L)");
    common_ctrl_.addButton((S32*)&action_, Action_LoadOBJ,					FW_KEY_M,       "Load OBJ for subdivision... (M)");
    common_ctrl_.addButton((S32*)&action_, Action_ResetView,				FW_KEY_SPACE,   "Reset view... (SPACE)");
    common_ctrl_.addButton((S32*)&action_, Action_WriteOBJ,					FW_KEY_O,		"Write to .obj (Slow!) (O)");
    common_ctrl_.addSeparator();
    common_ctrl_.addButton((S32*)&action_, Action_IncreaseSubdivisionLevel,	FW_KEY_PLUS,	"Refine subdivision (+)");
    common_ctrl_.addButton((S32*)&action_, Action_DecreaseSubdivisionLevel,	FW_KEY_MINUS,	"Coarsen subdivision (-)");
    common_ctrl_.addSeparator();
    common_ctrl_.addToggle(&curvemode_, 									FW_KEY_C,	   "Draw curve (C)");
    common_ctrl_.addToggle(&curvenormalmode_,								FW_KEY_N,	   "Draw normals (N)");
    common_ctrl_.addSeparator();
    common_ctrl_.addToggle(&surfacemode_,									FW_KEY_S,	"Draw surface (S)");
    common_ctrl_.addSeparator();
    common_ctrl_.addToggle(&wireframe_,										FW_KEY_W,		"Draw wireframe (W)");
    common_ctrl_.addSeparator();
    common_ctrl_.addToggle(&pointmode_,										FW_KEY_P,    "Draw control points (P)");
    common_ctrl_.addSeparator();

    window_.setTitle("basis");
    window_.addListener(this);
    window_.addListener(&common_ctrl_);

    window_.getGL()->swapBuffers();

    initRendering();

    camera_.SetDimensions(600, 600);
    camera_.SetDistance(10);
    camera_.SetCenter(Vec3f(0, 0, 0));
}

//------------------------------------------------------------------------

bool App::handleEvent(const Window::Event& ev) {
    if (ev.type == Window::EventType_Close) {
        window_.showModalMessage("Exiting...");
        delete this;
        return true;
    } else if (ev.type == Window::EventType_KeyDown) {
        keyDownFunc(ev);
    } else if (ev.type == Window::EventType_KeyUp) {
        keyUpFunc(ev);
    } else if (ev.type == Window::EventType_Mouse) {
        motionFunc(ev);
    }

    Action action  = action_;
    action_ = Action_None;
    String name;

    switch (action)
    {
    case Action_None:
        break;

    case Action_LoadSWP:
        name = window_.showFileLoadDialog("Load");
        if (name.getLength()) {
            loadSWP(name.getPtr());
			curvemode_ = true;
			surfacemode_ = true;
			subdivisionmode_ = false;
			current_subdivision_level_ = 0;
			makeDisplayLists();
		}
        break;

    case Action_LoadOBJ:
        name = window_.showFileLoadDialog("Load OBJ Mesh", "obj:Wavefront OBJ" );
        if (name.getLength()) {
            loadOBJ(name.getPtr());
			curvemode_ = false;
			surfacemode_ = false;
			subdivisionmode_ = true;
			current_subdivision_level_ = 0;
			makeDisplayLists();
		}
        break;

    case Action_ResetView:
        camera_.SetRotation(Mat4f());
        camera_.SetCenter(Vec3f());
		camera_.SetDistance(10);
        break;

    case Action_WriteOBJ:
        for (auto i = 0u; i < surface_names_.size(); ++i) {
            string filename = "surface_" + surface_names_[i] + string(".obj");
            ofstream out(filename);
            if (!out) {
                common_ctrl_.message(sprintf("Could not open file %s, skipping", filename.c_str()));
                out.close();
                continue;
            } else {
                outputObjFile(out, surfaces_[i]);
                common_ctrl_.message(sprintf("Wrote %s", filename.c_str()));
            }
        }
        break;

	// change subdiv level
	case Action_IncreaseSubdivisionLevel:
		++current_subdivision_level_;
		// compute new mesh if we haven't done that already
		// (also, we need to have the initial model loaded)
		if (!subdivided_meshes_.empty()
			&& current_subdivision_level_ >= (int)subdivided_meshes_.size()) {
			MeshWithConnectivity MWC;

			MWC.fromMesh(*subdivided_meshes_.back());

			// this is where magic happens
			MWC.LoopSubdivision();

			auto meshPNC = new Mesh<VertexPNC>();
			MWC.toMesh(*meshPNC);

			subdivided_meshes_.push_back(unique_ptr<MeshBase>(meshPNC));
		}
		break;

	case Action_DecreaseSubdivisionLevel:
		current_subdivision_level_ = max(0, current_subdivision_level_-1);
		break;

    default:
        FW_ASSERT(false);
        break;
    }

    window_.setVisible(true);
    if (ev.type == Window::EventType_Paint) {
        drawScene();
    }
    window_.repaint();
    return false;
}

//------------------------------------------------------------------------

void App::keyDownFunc(const Window::Event& ev) {
    int x = ev.mousePos[0];
    int y = ev.mousePos[1];

    if (ev.key == FW_KEY_MOUSE_LEFT) {
        mouse_pressed_ = true;
        camera_.MouseClick(Camera::LEFT, x, y);
    } else if (ev.key == FW_KEY_MOUSE_MIDDLE) {
        mouse_pressed_ = true;
        camera_.MouseClick(Camera::MIDDLE, x, y);
    } else if (ev.key == FW_KEY_MOUSE_RIGHT) {
        mouse_pressed_ = true;
        camera_.MouseClick(Camera::RIGHT, x, y);
    }
}

//------------------------------------------------------------------------

void App::keyUpFunc(const Window::Event& ev) {
    int x = ev.mousePos[0];
    int y = ev.mousePos[1];
    if (ev.key == FW_KEY_MOUSE_LEFT || ev.key == FW_KEY_MOUSE_MIDDLE || ev.key == FW_KEY_MOUSE_RIGHT) {
        camera_.MouseRelease(x,y);
        mouse_pressed_ = false;
    }
}

//------------------------------------------------------------------------

// Called when mouse is moved.
void App::motionFunc(const Window::Event& ev) {
    if (mouse_pressed_) {
        int x = ev.mousePos[0];
        int y = ev.mousePos[1];
        camera_.MouseDrag(x,y);        
    }
}

//------------------------------------------------------------------------

// Called when the window is resized
// w, h - width and height of the window in pixels.
void App::reshapeFunc(int w, int h) {
    camera_.SetDimensions(w,h);

    camera_.SetViewport(0,0,w,h);
    camera_.ApplyViewport();

    // Set up a perspective view, with square aspect ratio
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    camera_.SetPerspective(50);
    camera_.ApplyPerspective();
}

//------------------------------------------------------------------------

// This function is responsible for displaying the object.
void App::drawScene(void) {
    // Remove any shader that may be in use.
    glUseProgram(0);

    // Clear the rendering window
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    Vec2i size = window_.getSize();
    reshapeFunc(size[0], size[1]);

    glMatrixMode( GL_MODELVIEW );  
    glLoadIdentity();              

    // Light color (RGBA)
    GLfloat Lt0diff[] = {1.0,1.0,1.0,1.0};
    GLfloat Lt0pos[] = {3.0,3.0,5.0,1.0};
    glLightfv(GL_LIGHT0, GL_DIFFUSE, Lt0diff);
    glLightfv(GL_LIGHT0, GL_POSITION, Lt0pos);

    camera_.ApplyModelview();

    // Call the relevant display lists.
    if (surfacemode_)
        glCallList(surface_lists_[wireframe_ ? 2 : 1]);

    if (curvemode_)
        glCallList(curve_lists_[1]);

    if (curvenormalmode_)
        glCallList(curve_lists_[2]);

	if (subdivisionmode_) {
		if (wireframe_)
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		else
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

		// This is a wee bit dirty; I've just bolted the Framework's mesh rendering code on
		// top of the custom system used in this assignment. Oh well. -Jaakko
		Mat4f objectToCamera, projection;
		glGetFloatv (GL_MODELVIEW_MATRIX, objectToCamera.getPtr());
		glGetFloatv (GL_PROJECTION_MATRIX, projection.getPtr());

		subdivided_meshes_[current_subdivision_level_]->draw( window_.getGL(), objectToCamera, projection );
		glUseProgram(0);
		// clean up, we only want to draw the meshes in wireframe
		glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
	}


    // This draws the coordinate axes when you're rotating, to
    // keep yourself oriented.
    if (mouse_pressed_)
    {
        glPushMatrix();
        glTranslated(camera_.GetCenter()[0], camera_.GetCenter()[1], camera_.GetCenter()[2]);
        glCallList(axis_list_);
        glPopMatrix();
    }

    if (pointmode_)
        glCallList(point_list_);
}

//------------------------------------------------------------------------

// Initialize OpenGL's rendering modes
void App::initRendering() {
    glEnable(GL_DEPTH_TEST);   // Depth testing must be turned on
    glEnable(GL_LIGHTING);     // Enable lighting calculations
    glEnable(GL_LIGHT0);       // Turn on light #0.

    // Setup polygon drawing
    glShadeModel(GL_SMOOTH);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    // Clear to black
    glClearColor(0,0,0,1);

    // Base material colors (they don't change)
    GLfloat diffColor[] = {0.4, 0.4, 0.4, 1};
    GLfloat specColor[] = {0.9, 0.9, 0.9, 1};
    GLfloat shininess[] = {50.0};

    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, diffColor);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specColor);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, shininess);

    curve_lists_[1] = glGenLists(1);
    curve_lists_[2] = glGenLists(1);
    surface_lists_[1] = glGenLists(1);
    surface_lists_[2] = glGenLists(1);
    axis_list_ = glGenLists(1);
    point_list_ = glGenLists(1);
}

//------------------------------------------------------------------------

// Load in objects from standard input into the class member variables: 
// control_points_, curves_, curve_names_, surfaces_, m_surfaceNames.  If
// loading fails, this will exit the program.
void App::loadSWP(string filename) {
    ifstream in(filename);

    cout << endl << "*** loading and constructing curves and surfaces ***" << endl;

    if (!parseFile(in, control_points_, curves_, curve_names_, surfaces_, surface_names_)) {
        cerr << "\aerror in file format\a" << endl;
        in.close();
        exit(-1);              
    }

    in.close();

    cerr << endl << "*** done ***" << endl;
}

//------------------------------------------------------------------------

void App::loadOBJ(string filename) {
    window_.showModalMessage(sprintf("Loading mesh from '%s'...", filename.c_str()));

    String oldError = clearError();
	MeshBase* mesh = importMesh((FW::String)filename.c_str());
    String newError = getError();

    if (restoreError(oldError)) {
        delete mesh;
        common_ctrl_.message(sprintf("Error while loading '%s': %s", filename.c_str(), newError.getPtr()));
        return;
    }

	// get rid of the old meshes if necessary
	subdivided_meshes_.clear();

	// first, weld vertices
	Mesh<VertexP> meshP(*mesh);
	meshP.collapseVertices();

	// center mesh to origin and normalize scale
	Vec3f bbmin, bbmax;
	meshP.getBBox(bbmin, bbmax);
	float scale = 10.0f / (bbmax-bbmin).length();
	Vec3f ctr = 0.5f*(bbmin+bbmax);
	Mat4f T = Mat4f::translate( -ctr );
	Mat4f S = Mat4f::scale(Vec3f(scale, scale, scale));
	meshP.xformPositions(S*T);

	// then, recompute normals to make them smooth
	Mesh<VertexPNC>* meshPNC = new Mesh<VertexPNC>(meshP);
	meshPNC->recomputeNormals();
	for (int i = 0; i < meshPNC->numVertices(); ++i)
		meshPNC->mutableVertex(i).c = Vec4f(1.0f, 1.0f, 1.0f, 1.0f);
	delete mesh;

	subdivided_meshes_.push_back(std::unique_ptr<MeshBase>(meshPNC));

	common_ctrl_.message(sprintf("Loaded mesh from '%s'", filename.c_str()));
}


//------------------------------------------------------------------------

void App::writeObjects(string prefix) {
    cerr << endl << "*** writing obj files ***" << endl;

    for (auto i = 0u; i < surface_names_.size(); ++i) {
        if (surface_names_[i] != ".") {
            string filename = prefix + "_" + surface_names_[i] + ".obj";
            ofstream out(filename);
            if (!out) {
                cerr << "\acould not open file " << filename << ", skipping"<< endl;
                out.close();
                continue;
            } else {
                outputObjFile(out, surfaces_[i]);
                cerr << "wrote " << filename <<  endl;
            }
        }
    }
}

//------------------------------------------------------------------------

void App::makeDisplayLists()
{    
    // Compile the display lists

    glNewList(curve_lists_[1], GL_COMPILE);
    {
        for (auto i = 0u; i < curves_.size(); ++i)
            drawCurve(curves_[i], 0.0);
    }
    glEndList();

    glNewList(curve_lists_[2], GL_COMPILE);
    {
        for (auto i = 0u; i < curves_.size(); ++i)
            drawCurve(curves_[i], -normal_length_);
    }
    glEndList();

    glNewList(surface_lists_[1], GL_COMPILE);
    {
        for (auto i = 0u; i < surfaces_.size(); ++i)
            drawSurface(surfaces_[i], true);
    }
    glEndList();

    glNewList(surface_lists_[2], GL_COMPILE);
    {
        for (auto i = 0u; i < surfaces_.size(); ++i) {
            drawSurface(surfaces_[i], false);
            drawNormals(surfaces_[i], normal_length_);
        }
    }
    glEndList();

    glNewList(axis_list_, GL_COMPILE);
    {
        // Save current state of OpenGL
        glPushAttrib(GL_ALL_ATTRIB_BITS);

        // This is to draw the axes when the mouse button is down
        glDisable(GL_LIGHTING);
        glLineWidth(3);
        glPushMatrix();
        glScaled(5.0,5.0,5.0);
        glBegin(GL_LINES);
        glColor4f(1,0.5,0.5,1); glVertex3d(0,0,0); glVertex3d(1,0,0);
        glColor4f(0.5,1,0.5,1); glVertex3d(0,0,0); glVertex3d(0,1,0);
        glColor4f(0.5,0.5,1,1); glVertex3d(0,0,0); glVertex3d(0,0,1);

        glColor4f(0.5,0.5,0.5,1);
        glVertex3d(0,0,0); glVertex3d(-1,0,0);
        glVertex3d(0,0,0); glVertex3d(0,-1,0);
        glVertex3d(0,0,0); glVertex3d(0,0,-1);

        glEnd();
        glPopMatrix();

        glPopAttrib();
    }
    glEndList();

    glNewList(point_list_, GL_COMPILE);
    {
        // Save current state of OpenGL
        glPushAttrib(GL_ALL_ATTRIB_BITS);

        // Setup for point drawing
        glDisable(GL_LIGHTING);    
        glColor4f(1,1,0.0,1);
        glPointSize(4);
        glLineWidth(1);

        for (auto i = 0u; i < control_points_.size(); ++i) {
            glBegin(GL_POINTS);
            for (auto j = 0u; j < control_points_[i].size(); ++j)
                glVertex(control_points_[i][j]);
            glEnd();

            glBegin(GL_LINE_STRIP);
            for (auto j = 0u; j < control_points_[i].size(); ++j)
                glVertex(control_points_[i][j]);
            glEnd();
        }

        glPopAttrib();
    }
    glEndList();
}

void App::screenshot (const String& name) {
    // Capture image.
    const Vec2i& size = window_.getGL()->getViewSize();
    Image image(size, ImageFormat::R8_G8_B8_A8);
    glUseProgram(0);
    glWindowPos2i(0, 0);
    glReadPixels(0, 0, size.x, size.y, GL_RGBA, GL_UNSIGNED_BYTE, image.getMutablePtr());

    // Display the captured image immediately.
    for (int i = 0; i < 3; ++i) {
        glDrawPixels(size.x, size.y, GL_RGBA, GL_UNSIGNED_BYTE, image.getPtr());
        window_.getGL()->swapBuffers();
    }
    glDrawPixels(size.x, size.y, GL_RGBA, GL_UNSIGNED_BYTE, image.getPtr());

    // Export.
    image.flipY();
    exportImage(name, &image);
    printf("Saved screenshot to '%s'", name.getPtr());
}

//------------------------------------------------------------------------

void FW::init(void) {
    new App;
}

//------------------------------------------------------------------------
