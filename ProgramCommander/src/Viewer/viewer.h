/** Viewer
 * Viewer draws a scene. A viewer knows how to draw difference objects that a scene holds.
 * E.g. Scene could have a moving beacon. Viewer knows how to draw it. Scene holds the kinematic
 * state (pose, velocity, acceleration) of the beacon.
 */

#pragma once

// OpenGL includes
#include <QGLViewer/qglviewer.h>

// anantak includes
#include "Models/ModelsLib1.h"
#include "Viewer/scene.h"
#include "Viewer/dynamic_scene.h"

namespace anantak {

class Viewer : public QGLViewer {
 public:
  
  // Options for viewer - sets drawing object options
  struct Options {
    float position_sphere_size;
    float rotation_axes_line_width;
    float rotation_axes_line_length;
    float ellipsoid_line_width;
    int16_t num_points_on_circle;
    
    float scene_width;
    float scene_length;
    
    float velocity_arrow_length_ratio;
    float velocity_arrow_radius_ratio;
    
    float target_height;
    float target_width;
    float target_depth;
    
    float meters_per_pixel;
    
    int64_t timer_delay;  // in micro seconds
    
    Options(float scale=1.);
  };
  
  // Options
  anantak::Viewer::Options options_;
  
  // Scene that this viewer draws - keep a pointer
  anantak::Scene*         scene_;
  anantak::DynamicScene*  dynamic_scene_;
  
  // Constructor - takes a pointer to the scene
  Viewer(anantak::Scene* s, float scale);
  Viewer(anantak::DynamicScene* s, float scale);
  
  bool Initiate();
  
  virtual ~Viewer();
  
  // Drawing functions
  virtual void DrawSpiral();
  virtual bool DrawEllipsoid();
  virtual bool DrawLine(float length);
  virtual bool DrawArrow(float length, float radius, int nbSubdivisions);
  virtual bool DrawCircleXY();
  virtual bool DrawCircleYZ();
  virtual bool DrawCircleZX();
  
  // Objects
  virtual bool DrawTarget(float height, float width, float depth);
  virtual bool DrawCamera(const anantak::CameraStateDisplay& camera);
  
  // Draw a pose state
  virtual bool DrawPoseState(const anantak::PoseStateDisplay& pose);
  virtual bool DrawVelocityState(const anantak::PoseStateDisplay& pose);
  
  
 protected:
  
  // OpenGL functions
  virtual void draw();
  virtual void init();
  virtual void animate();
  virtual QString helpString() const;
  
  // Vector of points on a circle
  Eigen::Matrix<float,3,Eigen::Dynamic> xy_circle_points_; 
  Eigen::Matrix<float,3,Eigen::Dynamic> yz_circle_points_; 
  Eigen::Matrix<float,3,Eigen::Dynamic> zx_circle_points_;
  
  //// Timer
  //QTimer viewer_timer_;
  //// Update
  //virtual void updateGL();
  
};

}   // namespace
