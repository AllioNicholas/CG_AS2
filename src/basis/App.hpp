#pragma once
#include "gui/Window.hpp"
#include "gui/CommonControls.hpp"
#include "gpu/Buffer.hpp"
#include "3d/Mesh.hpp"

#include "camera.h"
#include "curve.h"
#include "surf.h"

#include <string>
#include <vector>
#include <memory>

namespace FW
{
//------------------------------------------------------------------------

class App : public Window::Listener
{
private:
    enum Action
    {
        Action_None,
        Action_LoadSWP,
        Action_LoadOBJ,
        Action_ResetView,
        Action_WriteOBJ,
		Action_IncreaseSubdivisionLevel,
		Action_DecreaseSubdivisionLevel
    };

    enum CullMode
    {
        CullMode_None = 0,
        CullMode_CW,
        CullMode_CCW,
    };

public:
                        App             (void);
	virtual             ~App            (void) {}

    virtual bool        handleEvent     (const Window::Event& ev);

private:
                        App             (const App&); // forbidden
    App&                operator=       (const App&); // forbidden

    void arcballRotation(int end_x, int end_y);
    void keyDownFunc    (const Window::Event& ev);
    void keyUpFunc      (const Window::Event& ev);
    void motionFunc     (const Window::Event& ev);
    void reshapeFunc    (int w, int h);
    void drawScene      (void);
    void initRendering  (void);
    void loadSWP        (std::string filename);
    void writeObjects   (std::string filename);
	void loadOBJ        (std::string filename);
    void makeDisplayLists(void);
    void screenshot     (const String& name);

    Window              window_;
    CommonControls      common_ctrl_;

    Action              action_;

    CullMode            cullmode_;
    bool	            curvemode_;
	bool	            curvenormalmode_;
    bool		        surfacemode_;
    bool				pointmode_;
	bool				subdivisionmode_;

	bool				wireframe_;
    
    // This is the camera
    Camera              camera_;

    // These are state variables for the UI
    bool                mouse_pressed_;

    // This detemines how big to draw the normals
    const float         normal_length_;

    // These are arrays for display lists for each drawing mode.  The
    // convention is that drawmode 0 is "blank", and other drawmodes
    // just call the appropriate display lists.
    GLuint              curve_lists_[3];
    GLuint              surface_lists_[3];
    GLuint              axis_list_;
    GLuint              point_list_;

    // These vectors store the control points, curves, and surfaces
	// that will end up being drawn.  In addition, parallel vectors
	// store the names for the curves and surfaces (as given by the files).
    std::vector<std::vector<Vec3f>>	control_points_;
    std::vector<Curve>              curves_;
    std::vector<std::string>        curve_names_;
    std::vector<Surface>            surfaces_;
    std::vector<std::string>        surface_names_;

	std::vector<std::unique_ptr<MeshBase>> subdivided_meshes_;
	int								current_subdivision_level_;
};

//------------------------------------------------------------------------
}
